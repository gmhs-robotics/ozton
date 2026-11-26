use vex_sdk_mock as _;

use super::*;
use crate::route;

#[derive(Default)]
struct Application {
    outputs: Vec<&'static str>,
}

impl OztonCompete for Application {
    type Error = &'static str;

    const CONTROL_PERIOD: Duration = Duration::from_millis(1);
    const ROUTES: &'static [RobotRoute<Self>] = &[route!("Practice", Self::practice)];

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.outputs.push("stop drive");
        Ok(())
    }

    async fn driver(&mut self) -> Result<(), Self::Error> {
        self.outputs.push("drive");
        Err("driver fault")
    }
}

impl Application {
    async fn practice(&mut self) -> Result<(), &'static str> {
        self.outputs.push("practice");
        Err("practice fault")
    }
}

#[test]
fn application_driver_runs_once_per_activation_with_fault_cleanup() {
    vexide::runtime::block_on(async {
        // A direct enabled handoff preserves the previous mode's output until
        // the driver writes. Only the fault stops the drive.
        let mut runtime = Runtime::new(Application {
            outputs: vec!["auton"],
        });
        for _ in 0..2 {
            session::CompetitionSession(&mut runtime).driver().await;
        }
        assert_eq!(
            runtime.robot.outputs,
            ["auton", "drive", "stop drive", "drive", "stop drive"]
        );
    });
}

#[test]
fn route_choice_is_read_once_at_autonomous_entry() {
    use std::{cell::Cell, rc::Rc};

    type Selection = Option<RobotRoute<Application>>;
    struct Selector(Rc<Cell<usize>>, Selection);

    impl RouteSelector<Selection> for Selector {
        fn action(&self) -> Selection {
            self.0.set(self.0.get() + 1);
            self.1
        }

        fn poll(&mut self, _: bool) -> Option<Selection> {
            None
        }

        fn status(&mut self, _: &str) {}
    }

    vexide::runtime::block_on(async {
        let reads = Rc::new(Cell::new(0));
        let mut runtime = Runtime::new(Application::default()).with_ui(Ui::<Selection>::custom({
            let reads = Rc::clone(&reads);
            move |choices| {
                assert_eq!(choices.len(), 2);
                assert_eq!(choices[0].0, "Disable");
                assert!(choices[0].1.is_none());
                assert_eq!(choices[1].0, "Practice");
                assert_eq!(choices[1].1.map(|route| route.label), Some("Practice"));
                Selector(reads, choices[1].1)
            }
        }));

        session::CompetitionSession(&mut runtime).disabled().await;
        assert_eq!(runtime.robot.outputs.first().copied(), Some("stop drive"));
        session::CompetitionSession(&mut runtime).driver().await;
        assert_eq!(reads.get(), 0);
        session::CompetitionSession(&mut runtime).autonomous().await;
        assert_eq!(reads.get(), 1);
        session::CompetitionSession(&mut runtime).autonomous().await;
        assert_eq!(reads.get(), 2);
        let selected = runtime.selector.action().unwrap();
        assert_eq!(
            selected.run(&mut runtime.robot).await,
            Err("practice fault")
        );
        assert_eq!(runtime.robot.outputs.last(), Some(&"practice"));
    });
}
