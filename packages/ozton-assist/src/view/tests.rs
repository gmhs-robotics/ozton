use std::convert::Infallible;

use ozton_drivetrain::{
    BrakeMode, Drivetrain, HasBraking, HasLocalization, WheelActuation, WheelCommand,
};
use ozton_math::qty;

use super::*;

#[derive(Default)]
struct Model {
    vector: DVec2,
    turn: f64,
    wheels: [Option<WheelCommand>; 2],
}

impl Drive for Model {
    type Error = Infallible;

    fn drive_vector(&mut self, vector: DVec2, turn: f64) -> Result<(), Self::Error> {
        (self.vector, self.turn) = (vector, turn);
        Ok(())
    }
}

impl LateralDrive for Model {}

impl HasBraking for Model {
    type BrakeError = Infallible;

    fn apply_brake(&mut self, mode: BrakeMode) -> Result<(), Self::BrakeError> {
        self.drive_wheel_commands([WheelCommand::Brake(mode); 2])
    }
}

impl WheelActuation for Model {
    type WheelError = Infallible;

    fn drive_wheel_commands(
        &mut self,
        commands: [WheelCommand; 2],
    ) -> Result<(), Self::WheelError> {
        self.wheels = commands.map(Some);
        Ok(())
    }
}

struct Limit;

impl<D: Drive + HasLocalization<Localization = f64>> DriveAssist<D> for Limit {
    fn apply(&mut self, axes: &mut [f64; 2], context: &AssistContext<'_, D>) {
        *axes = axes.map(|axis| axis * *context.drive.localization());
    }
}

#[derive(Default)]
struct Steering(usize);

impl<D: Drive> DriveAssist<D> for Steering {
    fn apply(&mut self, axes: &mut [f64; 2], _: &AssistContext<'_, D>) {
        self.0 += 1;
        axes[1] += 0.125 * self.0 as f64;
    }

    fn reset(&mut self) {
        self.0 = 0;
    }
}

#[test]
fn borrowed_assists_preserve_order_state_and_capabilities() {
    let mut drive = Drivetrain::new(Model::default(), (), 0.5);
    let mut assists = DriveAssists::new((Limit, Steering::default()));

    assists.apply_to(&mut drive).arcade(0.5, 0.25).unwrap();
    assert_eq!((drive.model.vector.y, drive.model.turn), (0.25, 0.25));
    assert!(drive.take_output());

    assists.apply_to(&mut drive).arcade(0.5, 0.25).unwrap();
    assert_eq!((drive.model.vector.y, drive.model.turn), (0.25, 0.375));

    let mut assisted = assists.apply_to(&mut drive);
    assert_eq!(*assisted.localization(), 0.5);
    assisted.stop_now().unwrap();
    assisted.holonomic(DVec2::new(0.5, 0.5), 0.25).unwrap();
    assert_eq!(
        (drive.model.vector, drive.model.turn),
        (DVec2::new(0.5, 0.25), 0.25)
    );
}

#[test]
fn plain_custom_model_preserves_an_explicit_wheel_brake() {
    let mut drive = Model::default();
    let mut assists = DriveAssists::new(Steering::default());
    let commands = [
        WheelCommand::Brake(BrakeMode::Hold),
        WheelCommand::Velocity(qty!(1 m/s)),
    ];
    assists
        .apply_to(&mut drive)
        .drive_wheel_commands(commands)
        .unwrap();
    assert_eq!(
        drive.wheels,
        [
            Some(commands[0]),
            Some(WheelCommand::Velocity(qty!(0.875 m/s)))
        ]
    );
}
