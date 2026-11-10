//! Host-only Ozton route conversion and calibration commands.

mod calibration;
mod csv;

use std::{
    env,
    error::Error,
    fs,
    io::{self, BufReader, Write},
    path::Path,
    process::ExitCode,
};

use ozton_math::path::jerryio;

const USAGE: &str = "ozton-tools import-path <lemlib-v0.5.txt>\n                    ozton-tools calibrate <telemetry.csv> [experiment-name]\n                    Use --help for input column definitions.";

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("ozton-tools: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let arguments: Vec<_> = env::args_os().skip(1).collect();
    let mut stdout = io::stdout().lock();
    match arguments.as_slice() {
        [] => Err(USAGE.into()),
        [help] if help == "--help" || help == "-h" => {
            writeln!(
                stdout,
                "{USAGE}\n\nCalibration rows use the Ozton CSV v1 header.\n{}",
                calibration::COLUMNS
            )?;
            Ok(())
        }
        [command, input] if command == "import-path" => import(Path::new(input), stdout),
        [command, input] if command == "calibrate" => calibrate(Path::new(input), None, stdout),
        [command, input, name] if command == "calibrate" => {
            let name = name.to_str().ok_or("experiment name must be UTF-8")?;
            calibrate(Path::new(input), Some(name), stdout)
        }
        _ => Err(USAGE.into()),
    }
}

fn import(path: &Path, mut writer: impl Write) -> Result<(), Box<dyn Error>> {
    let input = fs::read(path)?;
    let waypoints =
        jerryio::parse(&input).map_err(|error| format!("invalid LemLib v0.5 export: {error:?}"))?;
    writeln!(writer, "const POINTS: &[ozton::math::path::Waypoint] = &[")?;
    for point in waypoints {
        writeln!(
            writer,
            "    ozton::math::path::Waypoint::new({:?}, {:?}, {:?}),",
            point.position.x, point.position.y, point.speed_fraction
        )?;
    }
    writeln!(writer, "];")?;
    Ok(())
}

fn calibrate(path: &Path, name: Option<&str>, writer: impl Write) -> Result<(), Box<dyn Error>> {
    let reader = BufReader::new(fs::File::open(path)?);
    let data = csv::read(reader, name)?;
    calibration::report(data, writer)
}
