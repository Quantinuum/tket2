//! This crate provides utility functions for working with
//! pg-libs

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
    const EPS: f64 = 2e-9;
    let x = value.rem_euclid(2.0);
    let qt = x * 2.0;
    let rounded = qt.round();
    if (qt - rounded).abs() <= EPS {
        Some((rounded as u8) % 4)
    } else {
        None
    }
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
    }

    #[test]
    fn test_cliff_angle_tolerance() {
        const EPS: f64 = 1e-9;
        assert_eq!(cliff_angle(0.5 + EPS * 0.9), Some(1));
        assert_eq!(cliff_angle(0.5 - EPS * 0.9), Some(1));
        assert_eq!(cliff_angle(0.5 + EPS * 1.1), None);
    }
}
