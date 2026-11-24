//! One mixed-gear chassis, one intake, and one native waypoint route.

#[cfg(feature = "logging")]
use std::fs::File;
use std::{convert::Infallible, rc::Rc, time::Duration};

use ozton::{
    autonomous::{
        Alignment, ConfiguredChassis, DifferentialConfig, RobotMotion, RoutineError, TrackingSetup,
        TravelDirection, chassis::ChassisError,
    },
    math::{
        geometry::Pose,
        localization::range::Surface,
        path::{Path, Waypoint},
    },
    motion::arcade::ArcadeInput,
    position_xy, position_xyz, route,
    runtime::{
        CompetitionBot as ConfiguredBot, RobotBuilder, RobotLayout,
        competition::{Program, RegisteredRoute},
    },
    tracking::{mount::Vec3, range::DistanceTracker},
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
const START: Pose = Pose::new(
    ozton::math::geometry::point(Length::ZERO, Length::ZERO),
    START_HEADING,
);
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
const FRONT_MOUNT: Vec3<Length> = position_xyz!(0 m, 0.15 m, 0 m);
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
    type Assists = ();
    type Drivetrain = ConfiguredChassis;
    type Mechanisms = Mechanisms;
    type Routes = RegisteredRoute<Self, RouteError>;
    type Report = ();
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
                    ozton::diagnostics::log_event!(
                        Error,
                        "telemetry",
                        "Could not save telemetry ({error})"
                    );
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
    robot
        .follow_path(
            &path,
            TravelDirection::Forward,
            ozton::autonomous::Follower::PurePursuit {
                lookahead: ozton::autonomous::routine_settings::PATH_LOOKAHEAD,
            },
        )
        .await?;
    robot.mechanisms.intake.on = false;
    robot
        .try_align(
            Alignment {
                sensor: FRONT_RANGE_INDEX,
                distance: Length::new::<meter>(1.10),
                heading: START_HEADING,
                maximum_speed: Velocity::new::<meter_per_second>(0.20),
                facing: ozton::autonomous::RangeFacing::Front,
            },
            Duration::from_secs(2),
        )
        .await?;
    Ok(())
}
// ANCHOR_END: route

#[vexide::main]
async fn main(p: Peripherals) -> std::process::ExitCode {
    ozton::run(async move {
        // ANCHOR: prepare_path
        let points = [
            Waypoint::new(position_xy!(0 in, 0 in), 48.0 / 127.0),
            Waypoint::new(position_xy!(4 in, 0 in), 64.0 / 127.0),
            Waypoint::new(position_xy!(8 in, 0 in), 64.0 / 127.0),
            Waypoint::new(position_xy!(12 in, 0 in), 48.0 / 127.0),
            Waypoint::new(position_xy!(16 in, 0 in), 32.0 / 127.0),
            Waypoint::new(position_xy!(18 in, 0 in), 0.0),
        ];
        let straight_path = Rc::new(
            Path::new(&points, CONFIG.path_limits())
                .map_err(|error| format!("Invalid profiled path: {error:?}"))?,
        );
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
                ozton::math::geometry::point(Length::new::<meter>(1.8), Length::new::<meter>(-1.8)),
                ozton::math::geometry::point(Length::new::<meter>(1.8), Length::new::<meter>(1.8)),
            )],
            ranges: vec![DistanceTracker::at_mount(
                DistanceSensor::new(p.port_8),
                (FRONT_MOUNT, Angle::new::<radian>(0.0)),
            )],
            ..TrackingSetup::default()
        };
        let drive = CONFIG
            .build(left, right, tracking)
            .await
            .map_err(|error| format!("Invalid chassis: {error:?}"))?;
        let intake = Intake {
            motor: Motor::new(p.port_5, Gearset::Green, Direction::Forward),
            on: false,
        };
        // ANCHOR: assemble
        Ok::<_, Box<dyn std::error::Error>>(
            RobotBuilder::new()
                .with_controller(p.primary_controller)
                .with_drivetrain(drive)
                .with_mechanisms(Mechanisms { intake })
                .with_program(Controls { straight_path })
                .with_auton([route!("Straight", straight)])
                .with_ui(ozton::runtime::competition::Ui::Brain(p.display))
                .with_diagnostics::<()>(Duration::from_millis(250))
                .build(),
        )
        // ANCHOR_END: assemble
    })
    .await
}
