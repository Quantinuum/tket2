//! This crate provides utility functions for working with
//! pg-libs

/// Absolute tolerance used for approximate floating-point comparisons.
const EPS: f64 = 2e-9;

/// Compute the Clifford angle in quarter turns for a half-turn input value.
///
/// # Arguments
///
/// - `value` (`f64`) - The half-turn value.
///
/// # Returns
///
/// - `Option<u8>` - The Clifford angle is expressed as a number of quarter turns (0, 1, 2, 3) if the value is a valid Clifford angle, otherwise `None`.
///
pub fn cliff_angle(value: f64) -> Option<u8> {
    let x = value.rem_euclid(2.0);
    let qt = x * 2.0;
    let rounded = qt.round();
    if (qt - rounded).abs() <= EPS {
        Some((rounded as u8) % 4)
    } else {
        None
    }
}

/// Test whether a value is approximately 0 modulo n
///
/// # Arguments
///
/// - `value` (`f64`) - The value to test.
/// - `n` (`f64`) - The modulus value.
///
/// # Returns
///
/// - `bool` - `true` if the value is approximately 0 modulo n, otherwise `false`.
///
/// # Panics
///
/// Panics if `n` is zero or not finite.
///
pub fn equiv_0(value: f64, n: f64) -> bool {
    if !n.is_finite() || n == 0.0 {
        panic!("equiv_0 requires a finite, non-zero modulus");
    }
    let remainder = value.rem_euclid(n);
    remainder <= EPS || n.abs() - remainder <= EPS
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cliff_angle_basic() {
        assert_eq!(cliff_angle(0.0), Some(0));
        assert_eq!(cliff_angle(0.5), Some(1));
        assert_eq!(cliff_angle(1.0), Some(2));
        assert_eq!(cliff_angle(1.5), Some(3));
    }

    #[test]
    fn test_cliff_angle_wrapping() {
        assert_eq!(cliff_angle(2.0), Some(0));
        assert_eq!(cliff_angle(2.5), Some(1));
        assert_eq!(cliff_angle(4.0), Some(0));
    }

    #[test]
    fn test_cliff_angle_negative() {
        assert_eq!(cliff_angle(-0.5), Some(3));
        assert_eq!(cliff_angle(-1.0), Some(2));
        assert_eq!(cliff_angle(-1.5), Some(1));
        assert_eq!(cliff_angle(-2.0), Some(0));
    }

    #[test]
    fn test_cliff_angle_non_clifford() {
        assert_eq!(cliff_angle(0.25), None);
        assert_eq!(cliff_angle(0.1), None);
        assert_eq!(cliff_angle(1.3), None);
        assert_eq!(cliff_angle(f64::NAN), None);
        assert_eq!(cliff_angle(f64::INFINITY), None);
        assert_eq!(cliff_angle(f64::NEG_INFINITY), None);
    }

    #[test]
    fn test_cliff_angle_tolerance() {
        const EPS: f64 = 1e-9;
        assert_eq!(cliff_angle(0.5 + EPS * 0.9), Some(1));
        assert_eq!(cliff_angle(0.5 - EPS * 0.9), Some(1));
        assert_eq!(cliff_angle(0.5 + EPS * 1.1), None);
    }

    #[test]
    fn test_equiv_0_multiples() {
        assert!(equiv_0(0.0, 2.0));
        assert!(equiv_0(4.0, 2.0));
        assert!(equiv_0(-6.0, 3.0));
        assert!(equiv_0(0.25, 0.25));
        assert!(equiv_0(0.5, 0.25));
        assert!(!equiv_0(1.0, 2.0));
        assert!(!equiv_0(-1.0, 2.0));
    }

    #[test]
    fn test_equiv_0_tolerance() {
        assert!(equiv_0(1e-9, 2.0));
        assert!(equiv_0(-1e-9, 2.0));
        assert!(equiv_0(2.0 - 1e-9, 2.0));
        assert!(equiv_0(2.0 + 1e-9, 2.0));
        assert!(!equiv_0(3e-9, 2.0));
        assert!(!equiv_0(-3e-9, 2.0));
    }

    #[test]
    #[should_panic(expected = "equiv_0 requires a finite, non-zero modulus")]
    fn test_equiv_0_zero_modulus() {
        equiv_0(0.0, 0.0);
    }

    #[test]
    fn test_equiv_0_negative_modulus() {
        assert!(equiv_0(4.0, -2.0));
        assert!(equiv_0(2.0 - 1e-9, -2.0));
        assert!(!equiv_0(1.0, -2.0));
    }

    #[test]
    #[should_panic(expected = "equiv_0 requires a finite, non-zero modulus")]
    fn test_equiv_0_nan_modulus() {
        equiv_0(0.0, f64::NAN);
    }

    #[test]
    #[should_panic(expected = "equiv_0 requires a finite, non-zero modulus")]
    fn test_equiv_0_infinite_modulus() {
        equiv_0(0.0, f64::INFINITY);
    }

    #[test]
    fn test_equiv_0_non_finite_values() {
        assert!(!equiv_0(f64::NAN, 2.0));
        assert!(!equiv_0(f64::INFINITY, 2.0));
        assert!(!equiv_0(f64::NEG_INFINITY, 2.0));
    }
}
