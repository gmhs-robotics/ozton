use super::*;

#[derive(Default)]
struct Model([f64; 2]);

impl DrivetrainModel for Model {
    type Error = std::convert::Infallible;
}

impl Tank for Model {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), Self::Error> {
        self.0 = [left, right];
        Ok(())
    }
}

struct Limit(f64);

impl DriveAssist<Model> for Limit {
    fn apply(&mut self, axes: &mut [f64; 2], _: Option<Orientation>, _: Duration) {
        *axes = axes.map(|axis| axis * self.0);
    }
}

struct Steering(f64);

impl DriveAssist<Model> for Steering {
    fn apply(&mut self, axes: &mut [f64; 2], _: Option<Orientation>, _: Duration) {
        axes[1] += self.0;
    }

    fn reset(&mut self) {
        self.0 = 0.0;
    }
}

#[test]
fn empty_chain_drives_and_installed_chain_runs_in_order_and_resets_on_stop() {
    let mut drive = Drivetrain::new(Model::default(), (), ());
    drive.assisted().drive(ArcadeInput::new(0.5, 0.25));
    assert_eq!(drive.model.0, [0.75, 0.25]);

    let mut drive = drive
        .with_drive_assist(Limit(0.5))
        .with_drive_assist(Steering(0.125));
    drive.assisted().drive(ArcadeInput::new(0.5, 0.25));
    assert_eq!(drive.model.0, [0.5, 0.0]);
    drive.stop_now().unwrap();
    assert_eq!(drive.model.0, [0.0; 2]);
    drive.assisted().drive(ArcadeInput::new(0.5, 0.25));
    assert_eq!(drive.model.0, [0.375, 0.125]);
}
