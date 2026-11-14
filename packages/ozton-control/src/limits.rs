//! Shared finite-range validation. Physical limits remain owned by applications.
/// Validate an optional inclusive travel range. None means unrestricted.
pub fn valid_range(range: Option<(f64, f64)>) -> bool {
    range.is_none_or(|(min, max)| min.is_finite() && max.is_finite() && min < max)
}
/// Clamp a finite target; invalid raw targets remain invalid for caller validation.
pub fn clamp_target(target: f64, range: Option<(f64, f64)>) -> f64 {
    // Validate raw input before clamping, so infinities cannot become valid targets.
    if !target.is_finite() {
        return target;
    }
    range.map_or(target, |(min, max)| {
        if min.is_finite() && max.is_finite() && min < max {
            target.clamp(min, max)
        } else {
            f64::NAN
        }
    })
}
