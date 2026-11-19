//! One mixed-gear chassis, one intake, and one embedded PATH.JERRYIO route.

use std::{fmt, fs::File, rc::Rc, time::Duration};

use ozton::{
    autonomous_routes,
    drivetrain::model::{DriverFaultPolicy, GearRatio},
    math::{
        geometry::Pose,
        localization::range::Surface,
        path::{Path, follower::Follower, jerryio},
    },
    motion::auton::{
        Alignment, AutonError, AutonomousControls, Command, ConfiguredChassis, DifferentialConfig,
        Feedback, Runner, TrackingSetup, chassis::ChassisError,
    },
    runtime::{
        Robot,
        clock::CONTROL_PERIOD,
        competition::{Autonomous, CompetitionControls, Tick},
        diagnostics::{Frame, MotorSample, Report, Sample, Sampler},
    },
    tracking::{
        localizer::{PoseTrust, RangeSensorConfig},
        mount::RobotPoint,
    },
    units::{centimeters, inches, meters, meters_per_second},
};
use vexide::{competition::CompetitionMode, math::Angle, prelude::*, smart::motor::BrakeMode};

// Embedded path moves right (+X), so begin facing right (-90 degrees).
const START: Pose = Pose::from_lengths(meters(0.0), meters(0.0), -core::f64::consts::FRAC_PI_2);
const GEARED: GearRatio = GearRatio::from_teeth(36, 48);
const DRIVER_FAULT_POLICY: DriverFaultPolicy = DriverFaultPolicy::Stop;
const CONFIG: DifferentialConfig = DifferentialConfig {
    journal_capacity: 2048,
    ..DifferentialConfig::new(
        inches(2.25).value,
        centimeters(30.0).value,
        meters_per_second(0.50).value,
    )
};
const FRONT_MOUNT: RobotPoint = RobotPoint::new(meters(0.0), centimeters(15.0), meters(0.0));
// `front` is the first (and only) range registered with `TrackingSetup` below.
const FRONT_RANGE_INDEX: usize = 0;

struct Intake {
    motor: Motor,
    on: bool,
}

struct Controls {
    straight_path: Rc<Path>,
}

type CompetitionBot = Robot<
    ConfiguredChassis,
    Controller,
    (),
    Intake,
    (),
    Controls,
    Autonomous<Route, Sampler<IntakeReport>>,
>;

struct IntakeReport;

impl Report<CompetitionBot> for IntakeReport {
    fn collect(robot: &CompetitionBot, frame: Frame, _: Sample) {
        MotorSample::read(&robot.mechanisms.motor).write(frame, "intake");
    }
}

// The competition runtime displays route failures. The runner's errors implement Debug.
#[derive(Debug)]
struct RouteError(AutonError<ChassisError>);

impl fmt::Display for RouteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self.0)
    }
}

impl From<AutonError<ChassisError>> for RouteError {
    fn from(error: AutonError<ChassisError>) -> Self {
        Self(error)
    }
}

impl CompetitionControls<CompetitionBot> for Controls {
    type Error = RouteError;

    fn stop(robot: &mut CompetitionBot) -> Result<(), Self::Error> {
        <Self as AutonomousControls<_>>::stop(robot)
            .map_err(|error| RouteError(AutonError::StopFailed(error)))
    }

    fn driver_tick(robot: &mut CompetitionBot, tick: Tick) {
        let Ok(state) = robot.controller.state() else {
            let _ = <Self as CompetitionControls<_>>::stop(robot);
            return;
        };
        robot.mechanisms.on = state.button_r1.is_pressed();
        let drive = robot
            .drivetrain
            .chassis
            .drivetrain
            .model
            .drive_driver_arcade(
                state.left_stick.y(),
                state.right_stick.x(),
                DRIVER_FAULT_POLICY,
            );
        robot.mark_driver_output(tick);
        if (drive.is_err() && matches!(DRIVER_FAULT_POLICY, DriverFaultPolicy::Stop))
            || Self::update_mechanisms(robot, tick.elapsed).is_err()
        {
            let _ = <Self as CompetitionControls<_>>::stop(robot);
        }
    }

    fn mode_changed(robot: &mut CompetitionBot, mode: CompetitionMode) {
        let journal = &mut robot.drivetrain.chassis.journal;
        if mode == CompetitionMode::Disabled && !journal.is_empty() {
            match File::create("auton.csv").and_then(|file| journal.write_csv(file)) {
                Ok(()) => journal.clear(),
                Err(error) => eprintln!("Telemetry save failed: {error}"),
            }
        }
    }
}

impl AutonomousControls<CompetitionBot> for Controls {
    type Error = ChassisError;

    fn sample(robot: &mut CompetitionBot) -> Result<Option<Feedback>, Self::Error> {
        robot.drivetrain.chassis.sample(0)
    }

    fn update_mechanisms(robot: &mut CompetitionBot, _: Duration) -> Result<(), Self::Error> {
        let intake = &mut robot.mechanisms;
        intake
            .motor
            .set_voltage(if intake.on { 5.0 } else { 0.0 })
            .map_err(ChassisError::Output)
    }

    fn drive(
        robot: &mut CompetitionBot,
        wheels: [f64; 2],
        dt: Duration,
    ) -> Result<(), Self::Error> {
        robot.drivetrain.chassis.drive(wheels, dt)
    }

    fn stop_drive(robot: &mut CompetitionBot) -> Result<(), Self::Error> {
        robot.drivetrain.chassis.stop()
    }

    fn stop(robot: &mut CompetitionBot) -> Result<(), Self::Error> {
        let drive = Self::stop_drive(robot);
        robot.mechanisms.on = false;
        let intake = robot.mechanisms.motor.brake(BrakeMode::Brake);
        drive.and(intake.map_err(ChassisError::Output))
    }

    fn set_pose(robot: &mut CompetitionBot, pose: Pose) -> Result<(), Self::Error> {
        robot.drivetrain.chassis.set_pose(pose)
    }

    fn pose_trusted(robot: &CompetitionBot) -> bool {
        robot.drivetrain.chassis.drivetrain.tracking.pose_trust() == PoseTrust::Trusted
    }

    fn request_distance_reset(robot: &mut CompetitionBot) {
        robot.drivetrain.chassis.request_distance_reset();
    }

    fn cancel_distance_reset(robot: &mut CompetitionBot) {
        robot
            .drivetrain
            .chassis
            .drivetrain
            .tracking
            .cancel_distance_reset();
    }

    fn log_step(
        robot: &mut CompetitionBot,
        elapsed: Duration,
        dt: Duration,
        progress: f64,
        wheels: [f64; 2],
    ) {
        robot
            .drivetrain
            .chassis
            .log_step(elapsed, dt, progress, wheels);
    }
}

// ANCHOR: route
autonomous_routes! {
    enum Route for CompetitionBot => RouteError {
        Straight("Straight") => straight,
    }
}

async fn straight(robot: &mut CompetitionBot) -> Result<(), RouteError> {
    let path = Rc::clone(&robot.controls.straight_path);
    let options = robot.drivetrain.drive_options();
    let mut auton = Runner::new(
        robot,
        options,
        CONTROL_PERIOD,
        CONFIG.max_control_gap,
        CONFIG.route_timeout,
    )?;
    auton.confirm_start(START)?;
    auton.robot().mechanisms.on = true;
    auton
        .run(
            Command::Follow {
                path: &path,
                follower: Follower::PurePursuit { lookahead: 0.16 },
                reverse: false,
                trigger: None,
            },
            Duration::from_secs(5),
        )
        .await?;
    auton.robot().mechanisms.on = false;
    // This alignment is optional. The route can continue after a clean timeout.
    auton
        .try_align(
            Alignment {
                sensor: FRONT_RANGE_INDEX,
                distance: 1.10,
                heading: START.heading,
                maximum_speed: 0.20,
                direction: 1.0,
            },
            Duration::from_secs(2),
        )
        .await?;
    Ok(())
}
// ANCHOR_END: route

#[vexide::main]
async fn main(p: Peripherals) {
    // ANCHOR: prepare_path
    let points = jerryio::parse(include_bytes!("paths/straight.txt")).expect("valid path export");
    let straight_path =
        Rc::new(Path::new(&points, CONFIG.path_limits()).expect("valid profiled path"));
    // ANCHOR_END: prepare_path
    let side = |direct, geared, direction| {
        [
            (Motor::new_exp(direct, direction), GearRatio::DIRECT),
            (Motor::new(geared, Gearset::Blue, direction), GEARED),
        ]
    };
    let left = side(p.port_1, p.port_2, Direction::Reverse);
    let right = side(p.port_3, p.port_4, Direction::Forward);
    let tracking = TrackingSetup::default()
        .with_imu(InertialSensor::new(p.port_7))
        .with_map([Surface::new(1.8, -1.8, 1.8, 1.8)])
        .with_range(
            "front",
            DistanceSensor::new(p.port_8),
            RangeSensorConfig::new(
                FRONT_MOUNT
                    .horizontal_range_mount(Angle::ZERO)
                    .expect("valid range mount"),
            ),
        );
    let drive = CONFIG
        .build(left, right, tracking)
        .await
        .expect("valid chassis");
    let intake = Intake {
        motor: Motor::new(p.port_5, Gearset::Green, Direction::Forward),
        on: false,
    };
    // ANCHOR: assemble
    Robot::new()
        .with_controller(p.primary_controller)
        .with_drivetrain(drive)
        .with_mechanisms(intake)
        .with_controls(Controls { straight_path })
        .with_autonomous::<Route>(p.display)
        .with_diagnostics::<IntakeReport>(Duration::from_millis(250))
        .compete()
        .await;
    // ANCHOR_END: assemble
}
