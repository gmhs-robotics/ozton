use vex_sdk_mock as _;

use super::{super::tests::FakeDrive, *};
use crate::diagnostics::Frame;

impl ComponentReport for FakeDrive {
    fn report(&self, _: Frame) {}
}

struct Application {
    drive: FakeDrive,
    mechanism_updates: usize,
    mechanism_stops: usize,
    driver_entries: usize,
}

impl OztonCompete for Application {
    type Error = &'static str;
    type Drivetrain = FakeDrive;
    const CONTROL_PERIOD: Duration = Duration::from_millis(1);

    fn drivetrain(&mut self) -> &mut FakeDrive {
        &mut self.drive
    }

    fn update_mechanisms(&mut self, mode: CompetitionMode, _: Duration) -> Result<(), Self::Error> {
        assert_eq!(mode, CompetitionMode::Driver);
        self.mechanism_updates += 1;
        Ok(())
    }

    fn stop_mechanisms(&mut self) -> Result<(), Self::Error> {
        self.mechanism_stops += 1;
        Ok(())
    }

    async fn driver(&mut self, driver: &mut DriverLoop<'_, Self>) -> Result<(), Self::Error> {
        self.driver_entries += 1;
        self.drive.apply_driver(true, Duration::ZERO)?;
        self.drive.direct_output = true;
        driver.finish_iteration(self).await?;
        driver.finish_iteration(self).await?;
        Err("driver fault")
    }
}

#[test]
fn application_driver_runs_once_per_activation_with_watchdog_and_cleanup() {
    vexide::runtime::block_on(async {
        let mut runtime = Runtime::new(Application {
            drive: FakeDrive {
                writes: Vec::new(),
                fail_stop: false,
                direct_output: false,
            },
            mechanism_updates: 0,
            mechanism_stops: 0,
            driver_entries: 0,
        });

        for activation in 1..=2 {
            Compete::driver(&mut runtime).await;
            let robot = &runtime.robot;
            assert_eq!(robot.driver_entries, activation);
            assert_eq!(robot.mechanism_updates, 2 * activation);
            assert_eq!(robot.mechanism_stops, 2 * activation);
            assert_eq!(
                &robot.drive.writes[(activation - 1) * 4..],
                ["stop drive", "drive", "stop drive", "stop drive"]
            );
        }
    });
}
