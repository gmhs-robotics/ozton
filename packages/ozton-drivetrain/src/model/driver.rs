//! Traced, normalized driver commands.

use glam::DVec2;

use super::Drive;

pub(super) fn drive_command<D: Drive + ?Sized>(
    drive: &mut D,
    translation: DVec2,
    turn: f64,
    style: &str,
) -> Result<(), D::Error> {
    #[cfg(feature = "logging")]
    let started = std::time::Instant::now();
    let result = drive.drive_vector(translation, turn);
    #[cfg(feature = "logging")]
    ozton_diagnostics::log_row!(
        Trace,
        "DRIVER_OUTPUT",
        "style={style},right={:.4},forward={:.4},turn={turn:.4},output_us={},ok={}",
        translation.x,
        translation.y,
        started.elapsed().as_micros(),
        u8::from(result.is_ok()),
    );
    #[cfg(not(feature = "logging"))]
    let _ = style;
    result
}
