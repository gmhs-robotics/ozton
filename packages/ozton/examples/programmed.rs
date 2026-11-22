//! One mixed-gear chassis, one intake, and one embedded PATH.JERRYIO route.

#[cfg(feature = "logging")]
use std::fs::File;
use std::{convert::Infallible, rc::Rc, time::Duration};

use ozton::{
    drivetrain::model::ArcadeInput,
    math::{
        geometry::Pose,
        localization::range::Surface,
        path::{Path, jerryio},
    },
    motion::auton::{
        Alignment, ConfiguredChassis, DifferentialConfig, RobotMotion, RoutineError, TrackingSetup,
        TravelDirection, chassis::ChassisError,
    },
    position, route,
    runtime::{
        CompetitionBot as ConfiguredBot, RobotBuilder, RobotLayout,
        competition::{Program, RegisteredRoute},
        diagnostics::{Frame, MotorSample},
    },
    tracking::{localizer::vex::DistanceTracker, mount::Vec3},
    uom::{
        ConstZero,
        si::{
            angle::radian,
            f64::{Angle, AngularVelocity, Length, Velocity},
            length::meter,
            velocity::meter_per_second,
        },
    },
};
use vexide::{competition::CompetitionMode, prelude::*, smart::motor::BrakeMode};

// Embedded path moves right (+X), so begin facing right (-90 degrees).
const START_HEADING: Angle = Angle {
    value: -core::f64::consts::FRAC_PI_2,
    ..Angle::ZERO
};
const START: Pose = Pose::new(Length::ZERO, Length::ZERO, START_HEADING);
const CONFIG: DifferentialConfig = {
    let mut config = DifferentialConfig::new(
        Length {
            value: 0.05715,
            ..Length::ZERO
        },
        Length {
            value: 0.30,
            ..Length::ZERO
        },
        Velocity {
            value: 0.50,
            ..Velocity::ZERO
        },
        AngularVelocity {
            value: 200.0 * core::f64::consts::TAU / 60.0,
            ..AngularVelocity::ZERO
        },
    );
    config.journal_capacity = 2048;
    config
};
const FRONT_MOUNT: Vec3<Length> = position!(0 m, 0.15 m, 0 m);
// `front` is the first (and only) range registered with `TrackingSetup` below.
const FRONT_RANGE_INDEX: usize = 0;

struct Intake {
    motor: Motor,
    on: bool,
}

impl Intake {
    fn update(&mut self) -> Result<(), ChassisError> {
        self.motor
            .set_voltage(if self.on { 5.0 } else { 0.0 })
            .map_err(ChassisError::Output)
    }

    fn stop(&mut self) -> Result<(), ChassisError> {
        self.on = false;
        self.motor
            .brake(BrakeMode::Brake)
            .map_err(ChassisError::Output)
    }
}

struct Mechanisms {
    intake: Intake,
}

struct Controls {
    straight_path: Rc<Path>,
}

type CompetitionBot = ConfiguredBot<Controls>;

impl RobotLayout for Controls {
    type Drivetrain = ConfiguredChassis;
    type Tracking = ();
    type Mechanisms = Mechanisms;
    type Routes = RegisteredRoute<Self, RouteError>;
    type Report = ();
}

impl ozton::runtime::diagnostics::ComponentReport for Mechanisms {
    fn report(&self, frame: Frame) {
        MotorSample::read(&self.intake.motor).write(frame, "intake");
    }
}

impl ozton::runtime::competition::Mechanism<RouteError> for Mechanisms {}

type RouteError = RoutineError<Infallible>;

impl Program<CompetitionBot, ConfiguredChassis> for Controls {
    type Error = RouteError;

    fn drive_error(error: ChassisError, stopping: bool) -> RouteError {
        match error {
            ChassisError::Output(port) if stopping => RouteError::DriveBrake(port),
            ChassisError::Output(port) => RouteError::DriveOutput(port),
            ChassisError::Tracking(fault) => RouteError::Tracking(fault),
            _ => RouteError::MotionFault,
        }
    }

    async fn driver(robot: &mut CompetitionBot) -> Result<(), RouteError> {
        let Ok(input) = robot.controller.state() else {
            return robot.stop_mechanisms();
        };
        robot.record_controller(&input);
        robot.mechanisms.intake.on = input.button_r1.is_pressed();
        robot.drive_driver(ArcadeInput::new(input.left_stick.y(), input.right_stick.x()).into())
    }

    fn update_mechanisms(
        robot: &mut CompetitionBot,
        _: CompetitionMode,
        _: Duration,
    ) -> Result<(), RouteError> {
        robot
            .mechanisms
            .intake
            .update()
            .map_err(|error| Self::drive_error(error, false))
    }

    fn stop_mechanisms(robot: &mut CompetitionBot) -> Result<(), RouteError> {
        robot
            .mechanisms
            .intake
            .stop()
            .map_err(|error| Self::drive_error(error, true))
    }

    #[cfg(feature = "logging")]
    fn mode_changed(robot: &mut CompetitionBot, mode: CompetitionMode) {
        let journal = robot.drivetrain.journal_mut();
        if mode == CompetitionMode::Disabled && !journal.is_empty() {
            match File::create("auton.csv").and_then(|file| journal.write_csv(file)) {
                Ok(()) => journal.clear(),
                Err(error) => {
                    ozton::runtime::diagnostic_event!("telemetry_save_failed,error={error}")
                }
            }
        }
    }
}

// ANCHOR: route
async fn straight(robot: &mut CompetitionBot) -> Result<(), RouteError> {
    let path = Rc::clone(&robot.program.straight_path);
    robot.confirm_start(START)?;
    robot.mechanisms.intake.on = true;
    robot.follow_path(&path, TravelDirection::Forward).await?;
    robot.mechanisms.intake.on = false;
    robot
        .try_align(
            Alignment {
                sensor: FRONT_RANGE_INDEX,
                distance: Length::new::<meter>(1.10),
                heading: START_HEADING,
                maximum_speed: Velocity::new::<meter_per_second>(0.20),
                facing: ozton::motion::auton::RangeFacing::Front,
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
    let side = |first, second, direction| {
        [
            Motor::new(first, Gearset::Green, direction),
            Motor::new(second, Gearset::Green, direction),
        ]
    };
    let left = side(p.port_1, p.port_2, Direction::Reverse);
    let right = side(p.port_3, p.port_4, Direction::Forward);
    let tracking = TrackingSetup {
        imu: Some(InertialSensor::new(p.port_7)),
        map: vec![Surface::new(
            Length::new::<meter>(1.8),
            Length::new::<meter>(-1.8),
            Length::new::<meter>(1.8),
            Length::new::<meter>(1.8),
        )],
        ranges: vec![DistanceTracker::new(
            p.port_8,
            FRONT_MOUNT,
            Angle::new::<radian>(0.0),
        )],
        ..TrackingSetup::default()
    };
    let drive = CONFIG
        .build(left, right, tracking)
        .await
        .expect("valid chassis");
    let intake = Intake {
        motor: Motor::new(p.port_5, Gearset::Green, Direction::Forward),
        on: false,
    };
    // ANCHOR: assemble
    RobotBuilder::new()
        .with_controller(p.primary_controller)
        .with_drivetrain(drive)
        .with_mechanisms(Mechanisms { intake })
        .with_program(Controls { straight_path })
        .with_auton([route!("Straight", straight)])
        .with_ui(ozton::runtime::competition::Ui::Brain(p.display))
        .with_diagnostics::<()>(Duration::from_millis(250))
        .build()
        .compete()
        .await;
    // ANCHOR_END: assemble
}
