//! Filters for the [`Units`] iterator that unwrap the yielded units when
//! possible.
//!
//! These are meant to be used as a parameter to [`Iterator::filter_map`].
//!
//! [`Units`]: crate::circuit::units::Units

use hugr::CircuitUnit;
use hugr::Wire;
use hugr::extension::prelude;
use hugr::types::Type;

use super::LinearUnit;

/// A unit filter that return only linear units.
pub fn filter_linear<N, P>(item: (CircuitUnit<N>, P, Type)) -> Option<(LinearUnit, P, Type)> {
    match item {
        (CircuitUnit::Linear(unit), port, typ) => Some((LinearUnit::new(unit), port, typ)),
        _ => None,
    }
}

/// A unit filter that return only qubits, a subset of [`filter_linear`].
pub fn filter_qubit<N, P>(item: (CircuitUnit<N>, P, Type)) -> Option<(LinearUnit, P, Type)> {
    match item {
        (CircuitUnit::Linear(unit), port, typ) if typ == prelude::qb_t() => {
            Some((LinearUnit::new(unit), port, typ))
        }
        _ => None,
    }
}

/// A unit filter that return only non-linear units.
pub fn filter_non_linear<N, P>(item: (CircuitUnit<N>, P, Type)) -> Option<(Wire<N>, P, Type)> {
    match item {
        (CircuitUnit::Wire(wire), port, typ) => Some((wire, port, typ)),
        _ => None,
    }
}
