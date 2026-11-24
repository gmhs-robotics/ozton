use super::*;

#[derive(Default)]
struct Model(DVec2, f64);

impl Drive for Model {
    type Error = std::convert::Infallible;

    fn drive_vector(&mut self, vector: DVec2, turn: f64) -> Result<(), Self::Error> {
        (self.0, self.1) = (vector, turn);
        Ok(())
    }
}

impl LateralDrive for Model {}

impl MotorGroups for Model {
    type WheelSpeeds = [Velocity; 0];
    type WheelVoltages = [ozton_math::uom::si::f64::ElectricPotential; 0];

    fn motor_groups(&self) -> &[ozton_tracking::odometric::SharedMotors] {
        &[]
    }

    fn motor_groups_mut(&mut self) -> &mut [ozton_tracking::odometric::SharedMotors] {
        &mut []
    }
}

struct Estimate(f64);

struct Limit;

impl DriveAssist<Model, (), Estimate> for Limit {
    fn apply(&mut self, axes: &mut [f64; 2], context: &AssistContext<'_, Model, (), Estimate>) {
        *axes = axes.map(|axis| axis * context.localization.0);
    }
}

#[derive(Default)]
struct Steering(usize);

impl DriveAssist<Model, (), Estimate> for Steering {
    fn apply(&mut self, axes: &mut [f64; 2], _: &AssistContext<'_, Model, (), Estimate>) {
        self.0 += 1;
        axes[1] += 0.125 * self.0 as f64;
    }

    fn reset(&mut self) {
        self.0 = 0;
    }
}

#[test]
fn borrowed_assists_preserve_order_state_and_capabilities() {
    let mut drive = Drivetrain::new(Model::default(), (), Estimate(0.5));
    let mut assists = DriveAssists::new((Limit, Steering::default()));

    AssistedDrivetrain::new(&mut drive, &mut assists)
        .drive_vector(DVec2::new(0.0, 0.5), 0.25)
        .unwrap();
    assert_eq!((drive.model.0.y, drive.model.1), (0.25, 0.25));
    assert!(drive.take_output());

    AssistedDrivetrain::new(&mut drive, &mut assists)
        .drive_vector(DVec2::new(0.0, 0.5), 0.25)
        .unwrap();
    assert_eq!((drive.model.0.y, drive.model.1), (0.25, 0.375));

    let mut assisted = AssistedDrivetrain::new(&mut drive, &mut assists);
    assert_holonomic(&assisted);
    assert_eq!(assisted.localization().0, 0.5);
    assisted.stop_now().unwrap();
    assisted.drive_vector(DVec2::new(0.5, 0.5), 0.25).unwrap();
    assert_eq!(
        (drive.model.0, drive.model.1),
        (DVec2::new(0.5, 0.25), 0.25)
    );
}

fn assert_holonomic(_: &impl LateralDrive) {}
