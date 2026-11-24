//! Helpers for normalized controller inputs.

/// Zero non-finite or neutral inputs while preserving stick magnitude outside the deadzone.
pub fn deadzone(value: f64, threshold: f64) -> f64 {
    if value.is_finite() && value.abs() > threshold {
        value
    } else {
        0.0
    }
}
