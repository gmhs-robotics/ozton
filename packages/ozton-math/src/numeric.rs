//! Scalar math primitives with portable fallbacks.

/// Correctly rounded square root. The V5 target has a hardware VFP square
/// root instruction; the core intrinsic permits LLVM to select it directly.
/// Other targets retain the existing no-std libm implementation.
#[inline]
pub(crate) fn sqrt(value: f64) -> f64 {
    #[cfg(all(target_arch = "arm", target_os = "vexos"))]
    {
        core::f64::math::sqrt(value)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "vexos")))]
    {
        libm::sqrt(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn square_root_preserves_ieee_special_cases() {
        for value in [
            0.0,
            -0.0,
            f64::from_bits(1),
            f64::MIN_POSITIVE,
            0.1,
            1.0,
            2.0,
            f64::MAX,
            f64::INFINITY,
        ] {
            assert_eq!(sqrt(value).to_bits(), libm::sqrt(value).to_bits());
        }
        assert!(sqrt(-1.0).is_nan());
        assert!(sqrt(f64::NAN).is_nan());
    }
}
