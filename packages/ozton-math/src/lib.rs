//! Math utilities & types.

#![no_std]

pub mod curve;

/// Scales down the values in an array so that none exceed a given maximum magnitude.
///
/// This function checks the element with the largest absolute value in the input array.
/// If that magnitude is greater than `max`, all elements are uniformly scaled down so
/// that the largest magnitude equals `max`. If all elements are already within the limit,
/// the array is unchanged.
///
/// # Examples
///
/// ```
/// use ozton_math::desaturate;
///
/// let values = [3.0, -4.0, 1.0];
/// let result = desaturate(values, 2.0);
/// assert_eq!(result, [1.5, -2.0, 0.5]);
///
/// // Already within bounds, so unchanged:
/// let values = [0.5, -1.2, 0.8];
/// let result = desaturate(values, 2.0);
/// assert_eq!(result, values);
/// ```
pub fn desaturate<const N: usize>(values: [f64; N], max: f64) -> [f64; N] {
    let largest_magnitude = values.iter().map(|v| v.abs()).fold(0.0, f64::max);

    if largest_magnitude > max {
        values.map(|v| v * max / largest_magnitude)
    } else {
        values
    }
}
