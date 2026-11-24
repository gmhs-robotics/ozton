use std::{pin::pin, task::Poll};

use super::{
    dispatch::{dispatch_driver, stop_program, tracking_poll_due},
    session::control_sleep,
    *,
};

pub(super) struct FakeDrive {
    pub(super) writes: Vec<&'static str>,
    pub(super) fail_stop: bool,
    pub(super) direct_output: bool,
}

impl ProgramDrive for FakeDrive {
    type Error = &'static str;
    type DriverCommand = bool;
    type AutonCommand = i32;

    fn apply_driver(&mut self, command: bool, _: Duration) -> Result<(), Self::Error> {
        assert!(command);
        self.writes.push("drive");
        Ok(())
    }

    fn apply_auton(&mut self, command: i32, _: Duration) -> Result<(), Self::Error> {
        assert_eq!(command, 7);
        self.writes.push("auton");
        Ok(())
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.writes.push("stop drive");
        if self.fail_stop { Err("port") } else { Ok(()) }
    }

    fn take_driver_output(&mut self) -> bool {
        std::mem::take(&mut self.direct_output)
    }
}

struct FakeProgram {
    direct: bool,
}
struct NoTelemetry;
type TestRobot = Robot<FakeDrive, (), (), FakeProgram, NoTelemetry>;

impl Telemetry<TestRobot> for NoTelemetry {
    fn record(_: &mut TestRobot, _: Sample) {}
    fn status(_: &mut TestRobot, _: &str) {}
}

impl Program<TestRobot, FakeDrive> for FakeProgram {
    type Error = &'static str;

    fn drive_error(error: &'static str, _: bool) -> Self::Error {
        error
    }

    async fn driver(robot: &mut TestRobot) -> Result<(), Self::Error> {
        robot.drivetrain.writes.push("driver");
        if robot.program.direct {
            robot
                .drivetrain
                .apply_driver(true, robot.driver_elapsed())?;
            robot.drivetrain.direct_output = true;
            Ok(())
        } else {
            robot.drive_driver(true)
        }
    }

    fn update_mechanisms(
        robot: &mut TestRobot,
        mode: CompetitionMode,
        _: Duration,
    ) -> Result<(), Self::Error> {
        assert_eq!(mode, CompetitionMode::Driver);
        robot.drivetrain.writes.push("mechanisms");
        Ok(())
    }

    fn stop_mechanisms(robot: &mut TestRobot) -> Result<(), Self::Error> {
        robot.drivetrain.writes.push("stop mechanisms");
        Ok(())
    }
}

#[test]
fn driver_dispatch_and_stop_use_the_installed_drive() {
    let mut robot = TestRobot {
        drivetrain: FakeDrive {
            writes: Vec::new(),
            fail_stop: false,
            direct_output: false,
        },
        controller: (),
        mechanisms: (),
        program: FakeProgram { direct: false },
        autonomous: NoTelemetry,
        localization: None,
        assists: (),
        driver_elapsed: Duration::ZERO,
        driver_wrote: false,
    };
    let tick = Tick {
        started: Instant::now(),
        elapsed: CONTROL_PERIOD,
    };
    assert_eq!(
        pin!(dispatch_driver(&mut robot, tick)).poll(&mut core::task::Context::from_waker(
            core::task::Waker::noop()
        )),
        Poll::Ready(Ok(()))
    );
    assert_eq!(
        robot.update_mechanisms(CompetitionMode::Driver, tick.elapsed),
        Ok(())
    );
    robot.program.direct = true;
    assert_eq!(
        pin!(dispatch_driver(&mut robot, tick)).poll(&mut core::task::Context::from_waker(
            core::task::Waker::noop()
        )),
        Poll::Ready(Ok(()))
    );
    assert_eq!(
        robot.update_mechanisms(CompetitionMode::Driver, tick.elapsed),
        Ok(())
    );
    assert_eq!(robot.drivetrain.apply_auton(7, CONTROL_PERIOD), Ok(()));
    robot.drivetrain.fail_stop = true;
    assert_eq!(stop_program(&mut robot), Err("port"));
    assert_eq!(
        robot.drivetrain.writes,
        [
            "driver",
            "drive",
            "mechanisms",
            "driver",
            "drive",
            "mechanisms",
            "auton",
            "stop drive",
            "stop mechanisms"
        ]
    );
}

#[test]
fn overrunning_driver_tick_skips_optional_tracking() {
    assert!(tracking_poll_due(
        CompetitionMode::Driver,
        Duration::ZERO,
        CONTROL_PERIOD
    ));
    assert!(!tracking_poll_due(
        CompetitionMode::Driver,
        CONTROL_PERIOD,
        CONTROL_PERIOD
    ));
    assert!(!tracking_poll_due(
        CompetitionMode::Disabled,
        Duration::ZERO,
        CONTROL_PERIOD
    ));
    assert!(tracking_poll_due(
        CompetitionMode::Autonomous,
        CONTROL_PERIOD,
        CONTROL_PERIOD
    ));
}

#[test]
fn control_period_includes_work_and_overruns_still_yield() {
    let period = Duration::from_millis(10);
    assert_eq!(control_sleep(period, Duration::ZERO), period);
    assert_eq!(
        control_sleep(period, Duration::from_millis(3)),
        Duration::from_millis(7)
    );
    assert_eq!(control_sleep(period, period), Duration::from_millis(1));
    assert_eq!(
        control_sleep(period, Duration::from_secs(1)),
        Duration::from_millis(1)
    );
}
