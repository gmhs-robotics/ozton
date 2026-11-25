use std::{cell::RefCell, rc::Rc};

use glam::DVec3;
use ozton_math::uom::si::f64::{AngularVelocity, Length};
use vexide::prelude::Motor;

use super::{
    DriveConfigError, SharedMotors, WheelFeedback, WheelGearing, Wheels,
    consensus::MAX_SIDE_MOTORS, positive,
};
use crate::{DisplacementObservation, Measurement, MeasurementSink, TrackingSource};

/// Calibration and observation state for drive-wheel encoders. The drive model
/// shares motor handles with this source.
pub struct DifferentialTracking {
    track_width: Length,
    wheels: Wheels,
    motors: [SharedMotors; 2],
    feedback: [Option<WheelFeedback>; 2],
}

impl DifferentialTracking {
    /// Calibrate shared motor arrays with exactly one ratio per motor.
    ///
    /// ```compile_fail
    /// use ozton_math::length;
    /// use ozton_tracking::{odometric::{DifferentialTracking, WheelGearing}, shared_motors};
    /// use vexide::prelude::Motor;
    ///
    /// fn build(left: Motor, right: Motor) {
    ///     let ratio = WheelGearing::new(1, 1);
    ///     DifferentialTracking::from_shared(
    ///         &shared_motors![left], &shared_motors![right],
    ///         ozton_math::qty!(12 in), ozton_math::qty!(3 in), [ratio], [ratio, ratio],
    ///     ).unwrap();
    /// }
    /// ```
    pub fn from_shared<const LEFT: usize, const RIGHT: usize>(
        left: &Rc<RefCell<[Motor; LEFT]>>,
        right: &Rc<RefCell<[Motor; RIGHT]>>,
        track_width: Length,
        diameter: Length,
        left_gearing: [WheelGearing; LEFT],
        right_gearing: [WheelGearing; RIGHT],
    ) -> Result<Self, DriveConfigError> {
        const {
            assert!(
                LEFT > 0 && LEFT <= MAX_SIDE_MOTORS,
                "invalid left motor count"
            );
            assert!(
                RIGHT > 0 && RIGHT <= MAX_SIDE_MOTORS,
                "invalid right motor count"
            );
        }
        Self::new(
            &[left.clone().into(), right.clone().into()],
            track_width,
            diameter,
            &[&left_gearing, &right_gearing],
        )
    }

    /// Calibrate with a gear ratio per motor, in left and right group order.
    pub fn new(
        motors: &[SharedMotors; 2],
        track_width: Length,
        diameter: Length,
        gearing: &[&[WheelGearing]; 2],
    ) -> Result<Self, DriveConfigError> {
        Self::with_wheels(
            motors.clone(),
            track_width,
            Wheels::geared(diameter, motors, gearing)?,
        )
    }

    /// Infer ratios from the common free wheel speed and the motor cartridges.
    pub fn inferred(
        motors: &[SharedMotors; 2],
        track_width: Length,
        diameter: Length,
        free_speed: AngularVelocity,
    ) -> Result<Self, DriveConfigError> {
        Self::with_wheels(
            motors.clone(),
            track_width,
            Wheels::inferred(diameter, motors, free_speed)?,
        )
    }

    fn with_wheels(
        motors: [SharedMotors; 2],
        track_width: Length,
        wheels: Wheels,
    ) -> Result<Self, DriveConfigError> {
        positive(track_width.value, DriveConfigError::TrackWidth)?;
        Ok(Self {
            track_width,
            wheels,
            motors,
            feedback: [None; 2],
        })
    }

    pub fn track_width(&self) -> Length {
        self.track_width
    }

    pub fn wheels(&self) -> &Wheels {
        &self.wheels
    }

    pub fn read_wheels(&mut self) -> [Option<WheelFeedback>; 2] {
        self.feedback = self.wheels.sample(&self.motors);
        self.feedback
    }

    pub fn feedback(&self) -> [Option<WheelFeedback>; 2] {
        self.feedback
    }
}

impl TrackingSource for DifferentialTracking {
    const DIFFERENTIAL_ENCODERS: bool = true;

    fn sample(&mut self, sink: &mut MeasurementSink<'_>) {
        let feedback = self.read_wheels();
        let positions = feedback.map(|sample| sample.map(|wheel| wheel.position));
        sink.push(Measurement::Displacement(
            DisplacementObservation::Differential(positions),
        ));
        if let [Some(left), Some(right)] = feedback {
            let [left, right] = [left.velocity.value, right.velocity.value];
            sink.push(Measurement::Velocity(DVec3::Y * ((left + right) * 0.5)));
            sink.push(Measurement::Rotation(
                DVec3::Z * ((right - left) / self.track_width.value),
            ));
        }
    }
}
