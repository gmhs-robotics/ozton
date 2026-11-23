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

    let drive = drive
        .with_drive_assist(Limit(0.5))
        .with_drive_assist(Steering(0.125));
    assert_assist_order_and_reset(drive);
}

#[test]
fn dynamic_collection_has_the_same_drive_type_when_empty_or_populated() {
    type Drive = Drivetrain<Model, (), (), DriveAssists<Model>>;

    let mut drive: Drive =
        Drivetrain::new(Model::default(), (), ()).with_drive_assists(DriveAssists::new());
    drive.assisted().drive(ArcadeInput::new(0.5, 0.25));
    assert_eq!(drive.model.0, [0.75, 0.25]);

    let drive: Drive = Drivetrain::new(Model::default(), (), ())
        .with_drive_assists(DriveAssists::new().with(Limit(0.5)).with(Steering(0.125)));
    assert_assist_order_and_reset(drive);
}

fn assert_assist_order_and_reset(mut drive: Drivetrain<Model, (), (), impl DriveAssist<Model>>) {
    drive.assisted().drive(ArcadeInput::new(0.5, 0.25));
    assert_eq!(drive.model.0, [0.5, 0.0]);
    assert_eq!(drive.stop_now(), Ok(()));
    assert_eq!(drive.model.0, [0.0; 2]);
    drive.assisted().drive(ArcadeInput::new(0.5, 0.25));
    assert_eq!(drive.model.0, [0.375, 0.125]);
}
