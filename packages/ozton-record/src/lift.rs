//! Recordable two-motor mirrored lift.

use std::time::{Duration, Instant};

use async_trait::async_trait;
use ozton_control::lift::{LiftController, LiftProfile};
use rkyv::{Archive, Deserialize, Serialize};
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};

use crate::{Interpolate, RecordField, frame::RecordMode};

#[derive(Archive, Serialize, Deserialize, Default, Clone, Debug)]
#[rkyv(crate = ::rkyv)]
pub struct LiftFrame {
    pub direction: i8,
    pub position_degrees: f64,
}

impl Interpolate for LiftFrame {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        Self {
            direction: if amount < 0.5 {
                from.direction
            } else {
                to.direction
            },
            position_degrees: f64::interpolate(
                &from.position_degrees,
                &to.position_degrees,
                amount,
            ),
        }
    }
}

/// First motor is reversed in the common lift coordinate; second is forward.
pub struct MirroredLift {
    first: Motor,
    second: Motor,
    controller: LiftController,
    last_update: Instant,
}

impl MirroredLift {
    pub fn new(first: Motor, second: Motor, profile: LiftProfile) -> Self {
        Self {
            first,
            second,
            controller: LiftController::new(profile),
            last_update: Instant::now(),
        }
    }

    pub fn position_degrees(&self) -> Result<(f64, f64), PortError> {
        Ok((
            -self.first.position()?.as_degrees(),
            self.second.position()?.as_degrees(),
        ))
    }

    pub fn hold(&mut self) -> Result<(), PortError> {
        let first = self.first.brake(BrakeMode::Hold);
        let second = self.second.brake(BrakeMode::Hold);
        first.and(second)
    }
}

#[async_trait(?Send)]
impl RecordField for MirroredLift {
    type Output = LiftFrame;

    async fn finalize_frame_value(&self, frame: &LiftFrame) -> LiftFrame {
        let mut out = frame.clone();
        if let Ok((first, second)) = self.position_degrees() {
            out.position_degrees = (first + second) / 2.0;
        }
        out
    }

    async fn apply_frame_value(
        &mut self,
        frame: &LiftFrame,
        mode: RecordMode,
    ) -> Result<(), PortError> {
        let now = Instant::now();
        let dt = now
            .saturating_duration_since(self.last_update)
            .min(Duration::from_millis(100));
        self.last_update = now;
        let (first, second) = self.position_degrees()?;
        let target = (mode == RecordMode::Playback).then_some(frame.position_degrees);
        let (first_command, second_command) =
            self.controller
                .update(frame.direction, target, first, second, dt);
        if first_command == 0.0 && second_command == 0.0 {
            return self.hold();
        }
        let first_result = self
            .first
            .set_voltage(-first_command * self.first.max_voltage());
        let second_result = self
            .second
            .set_voltage(second_command * self.second.max_voltage());
        first_result.and(second_result)
    }

    async fn stop_playback(&mut self) -> Result<(), PortError> {
        self.hold()
    }
}
