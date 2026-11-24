use vex_sdk_mock as _;

use super::*;

struct FakeDrive {
    writes: Vec<&'static str>,
}

impl FakeDrive {
    fn stop(&mut self) -> Result<(), &'static str> {
        self.writes.push("stop drive");
        Ok(())
    }
}

struct Application {
    drive: FakeDrive,
    driver_entries: usize,
}

impl OztonCompete for Application {
    type Error = &'static str;
    const CONTROL_PERIOD: Duration = Duration::from_millis(1);

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.drive.stop()
    }

    async fn driver(&mut self) -> Result<(), Self::Error> {
        self.driver_entries += 1;
        self.drive.writes.push("drive");
        Err("driver fault")
    }
}

#[test]
fn application_driver_runs_once_per_activation_with_fault_cleanup() {
    vexide::runtime::block_on(async {
        let mut runtime = Runtime::new(Application {
            drive: FakeDrive {
                writes: Vec::new(),
            },
            driver_entries: 0,
        });

        for activation in 1..=2 {
            session::CompetitionSession(&mut runtime).driver().await;
            let robot = &runtime.robot;
            assert_eq!(robot.driver_entries, activation);
            assert_eq!(
                &robot.drive.writes[(activation - 1) * 2..],
                ["drive", "stop drive"]
            );
        }
    });
}

#[test]
fn route_choice_is_read_once_at_autonomous_entry() {
    use std::{cell::Cell, rc::Rc};

    struct Selector(Rc<Cell<usize>>);

    impl RouteSelector<Option<RobotRoute<Application, &'static str>>> for Selector {
        fn action(&self) -> Option<RobotRoute<Application, &'static str>> {
            self.0.set(self.0.get() + 1);
            None
        }

        fn poll(&mut self, _: bool) -> Option<Option<RobotRoute<Application, &'static str>>> {
            None
        }

        fn status(&mut self, _: &str) {}
    }

    vexide::runtime::block_on(async {
        let reads = Rc::new(Cell::new(0));
        let mut runtime = Runtime::new(Application {
            drive: FakeDrive {
                writes: Vec::new(),
            },
            driver_entries: 0,
        })
        .with_ui(Ui::custom({
            let reads = Rc::clone(&reads);
            move |_| Selector(reads)
        }));

        session::CompetitionSession(&mut runtime).disabled().await;
        assert_eq!(
            runtime.robot.drive.writes.first().copied(),
            Some("stop drive")
        );
        session::CompetitionSession(&mut runtime).driver().await;
        assert_eq!(reads.get(), 0);
        session::CompetitionSession(&mut runtime).autonomous().await;
        assert_eq!(reads.get(), 1);
        session::CompetitionSession(&mut runtime).autonomous().await;
        assert_eq!(reads.get(), 2);
    });
}

#[test]
fn direct_enabled_handoff_does_not_stop_latched_outputs() {
    vexide::runtime::block_on(async {
        let mut runtime = Runtime::new(Application {
            drive: FakeDrive {
                writes: vec!["auton"],
            },
            driver_entries: 0,
        });

        // Simulate an already-commanded motor and an enabled driver entry.
        // The driver itself writes output, then faults; only the fault stops it.
        session::CompetitionSession(&mut runtime).driver().await;
        assert_eq!(runtime.robot.drive.writes, ["auton", "drive", "stop drive"]);
    });
}
