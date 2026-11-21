//! One mixed-gear chassis, one intake, and one embedded PATH.JERRYIO route.

use std::{fmt, fs::File, rc::Rc, time::Duration};

use ozton::{
    autonomous_routes,
    drivetrain::model::DriverFaultPolicy,
    math::{
        geometry::Pose,
        localization::range::Surface,
        path::{Path, follower::Follower, jerryio},
    },
    motion::auton::{
        Alignment, AutonError, AutonomousControls, Command, ConfiguredChassis, DifferentialConfig,
        Feedback, Runner, TrackingSetup, chassis::ChassisError,
    },
    position,
    runtime::{
        Robot, Subsystem, SubsystemList,
        clock::CONTROL_PERIOD,
        competition::{Autonomous, CompetitionControls, CompetitionRobot, RouteSet, Tick},
        diagnostics::{Frame, MotorSample, Report, Sample, Sampler},
    },
    tracking::{localizer::RangeSensorConfig, mount::Vec3},
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
use vexide::{
    competition::CompetitionMode, controller::ControllerState, prelude::*, smart::motor::BrakeMode,
};

// Embedded path moves right (+X), so begin facing right (-90 degrees).
const START_HEADING: Angle = Angle {
    value: -core::f64::consts::FRAC_PI_2,
    ..Angle::ZERO
};
const START: Pose = Pose::new(Length::ZERO, Length::ZERO, START_HEADING);
const DRIVER_FAULT_POLICY: DriverFaultPolicy = DriverFaultPolicy::Stop;
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

impl Subsystem<(), ChassisError> for Intake {
    fn update(&mut self, _: &mut ()) -> Result<(), ChassisError> {
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

impl Subsystem<ControllerState, RouteError> for Intake {
    fn update(&mut self, state: &mut ControllerState) -> Result<(), RouteError> {
        self.on = state.button_r1.is_pressed();
        self.update(&mut ())
            .map_err(|error| RouteError(AutonError::Robot(error)))
    }

    fn stop(&mut self) -> Result<(), RouteError> {
        <Self as Subsystem<(), ChassisError>>::stop(self)
            .map_err(|error| RouteError(AutonError::StopFailed(error)))
    }
}

struct Controls {
    straight_path: Rc<Path>,
}

type CompetitionBot = Robot<
    ConfiguredChassis,
    Controller,
    (),
    ((), Intake),
    Controls,
    Autonomous<Route, Sampler<IntakeReport>>,
>;

struct IntakeReport;

impl Report<CompetitionBot> for IntakeReport {
    fn collect(robot: &CompetitionBot, frame: Frame, _: Sample) {
        MotorSample::read(&robot.subsystems.1.motor).write(frame, "intake");
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
    type SubsystemInput = ControllerState;

    fn stop(robot: &mut CompetitionBot) -> Result<(), Self::Error> {
        <Self as AutonomousControls<_>>::stop(robot)
            .map_err(|error| RouteError(AutonError::StopFailed(error)))
    }

    fn driver_tick(robot: &mut CompetitionBot, tick: Tick) -> Result<(), Self::Error> {
        let Ok(mut state) = robot.controller.state() else {
            return CompetitionRobot::stop(robot);
        };
        let drive = robot.drivetrain.drive_driver_arcade(
            state.left_stick.y(),
            state.right_stick.x(),
            DRIVER_FAULT_POLICY,
        );
        robot.mark_driver_output(tick);
        if drive.is_err() && matches!(DRIVER_FAULT_POLICY, DriverFaultPolicy::Stop) {
            return CompetitionRobot::stop(robot);
        }
        robot.subsystems.update_all(&mut state)
    }

    fn mode_changed(robot: &mut CompetitionBot, mode: CompetitionMode) {
        let journal = robot.drivetrain.journal_mut();
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
    type SubsystemInput = ();

    fn sample(robot: &mut CompetitionBot) -> Result<Option<Feedback>, Self::Error> {
        robot.drivetrain.sample(0)
    }

    fn subsystem_input(_: &mut CompetitionBot, _: Duration) -> Result<(), Self::Error> {
        Ok(())
    }

    fn drive(
        robot: &mut CompetitionBot,
        wheels: [Velocity; 2],
        dt: Duration,
    ) -> Result<(), Self::Error> {
        robot.drivetrain.drive(wheels, dt)
    }

    fn stop_drive(robot: &mut CompetitionBot) -> Result<(), Self::Error> {
        robot.drivetrain.stop()
    }

    fn stop(robot: &mut CompetitionBot) -> Result<(), Self::Error> {
        Self::stop_drive(robot)
    }

    fn set_pose(robot: &mut CompetitionBot, pose: Pose) -> Result<(), Self::Error> {
        robot.drivetrain.set_pose(pose)
    }

    fn pose_trusted(robot: &CompetitionBot) -> bool {
        robot.drivetrain.chassis().pose_trusted()
    }

    fn request_distance_reset(robot: &mut CompetitionBot) {
        robot.drivetrain.request_distance_reset();
    }

    fn cancel_distance_reset(robot: &mut CompetitionBot) {
        robot.drivetrain.cancel_distance_reset();
    }

    fn log_step(
        robot: &mut CompetitionBot,
        elapsed: Duration,
        dt: Duration,
        progress: Length,
        wheels: [Velocity; 2],
    ) {
        robot.drivetrain.log_step(elapsed, dt, progress, wheels);
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
    auton.robot().subsystems.1.on = true;
    auton
        .run(
            Command::Follow {
                path: &path,
                follower: Follower::PurePursuit {
                    lookahead: Length::new::<meter>(0.16),
                },
                reverse: false,
                trigger: None,
            },
            Duration::from_secs(5),
        )
        .await?;
    auton.robot().subsystems.1.on = false;
    // This alignment is optional. The route can continue after a clean timeout.
    auton
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
        ranges: vec![(
            DistanceSensor::new(p.port_8),
            RangeSensorConfig::at_mount(FRONT_MOUNT, Angle::new::<radian>(0.0)),
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
    Robot::new()
        .with_controller(p.primary_controller)
        .with_drivetrain(drive)
        .add_subsystem(intake)
        .with_controls(Controls { straight_path })
        .with_auton(Route::CHOICES.iter().map(|&(_, route)| route))
        .with_ui(ozton::runtime::competition::Ui::Brain(p.display))
        .with_diagnostics::<IntakeReport>(Duration::from_millis(250))
        .compete()
        .await;
    // ANCHOR_END: assemble
}
