//! A lattice row of values supplied to a node.

use std::cmp::Ordering;

use ascent::Lattice;
use ascent::lattice::BoundedLattice;
use itertools::zip_eq;

use super::{AbstractValue, PartialValue};

/// A dense input row or a sparse single-slot update to one.
///
/// Ascent initializes every node with a dense row of [`PartialValue::Bottom`]
/// before applying input-wire updates. Representing those updates sparsely
/// avoids allocating and joining a full row once for every input port.
#[derive(PartialEq, Clone, Debug, Eq, Hash)]
pub(super) enum ValueRow<V, N> {
    /// A complete row, used for stored lattice values and operation results.
    Dense(Vec<PartialValue<V, N>>),
    /// A single input update. Unspecified slots represent bottom.
    Sparse {
        /// Total number of slots in the row.
        len: usize,
        /// Slot changed by this update.
        index: usize,
        /// New lattice value for the slot.
        value: PartialValue<V, N>,
    },
}

impl<V: AbstractValue, N: Clone> ValueRow<V, N> {
    /// Create a complete bottom row.
    pub fn new(len: usize) -> Self {
        Self::Dense(vec![PartialValue::Bottom; len])
    }

    /// Create a sparse update for one slot in a row.
    pub fn sparse(len: usize, index: usize, value: PartialValue<V, N>) -> Self {
        assert!(index < len);
        Self::Sparse { len, index, value }
    }

    /// Create a complete, single-element row.
    pub fn singleton(value: PartialValue<V, N>) -> Self {
        Self::Dense(vec![value])
    }

    /// Return the complete values stored in this row.
    ///
    /// Sparse rows are only transient lattice updates. Ascent seeds a dense
    /// bottom row in an earlier SCC, so analysis rules only observe dense rows.
    pub fn values(&self) -> &[PartialValue<V, N>] {
        let Self::Dense(values) = self else {
            unreachable!("analysis rules must only observe dense value rows")
        };
        values
    }

    /// The first value in this row must be a sum; unpack the selected variant
    /// and append the remaining values in the row.
    pub fn unpack_first(
        &self,
        variant: usize,
        len: usize,
    ) -> Option<impl Iterator<Item = PartialValue<V, N>> + use<V, N>> {
        let values = self.values();
        let fields = values[0].variant_values(variant, len)?;
        Some(fields.into_iter().chain(values[1..].to_owned()))
    }
}

impl<V, N> FromIterator<PartialValue<V, N>> for ValueRow<V, N> {
    fn from_iter<T: IntoIterator<Item = PartialValue<V, N>>>(iter: T) -> Self {
        Self::Dense(iter.into_iter().collect())
    }
}

impl<V: PartialEq, N: PartialEq + PartialOrd> PartialOrd for ValueRow<V, N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match (self, other) {
            (Self::Dense(lhs), Self::Dense(rhs)) => lhs.partial_cmp(rhs),
            (
                Self::Sparse {
                    len: lhs_len,
                    index: lhs_index,
                    value: lhs_value,
                },
                Self::Sparse {
                    len: rhs_len,
                    index: rhs_index,
                    value: rhs_value,
                },
            ) if lhs_len == rhs_len && lhs_index == rhs_index => lhs_value.partial_cmp(rhs_value),
            _ => None,
        }
    }
}

impl<V: AbstractValue, N: PartialEq + PartialOrd + Clone> Lattice for ValueRow<V, N> {
    fn join_mut(&mut self, other: Self) -> bool {
        match (self, other) {
            (Self::Dense(lhs), Self::Dense(rhs)) => join_dense(lhs, rhs),
            (Self::Dense(values), Self::Sparse { len, index, value }) => {
                assert_eq!(values.len(), len);
                values[index].join_mut(value)
            }
            (row @ Self::Sparse { .. }, Self::Dense(mut values)) => {
                let Self::Sparse { len, index, value } = row else {
                    unreachable!()
                };
                assert_eq!(values.len(), *len);
                let old_value = value.clone();
                values[*index].join_mut(old_value.clone());
                let changed = values.iter().enumerate().any(|(i, value)| {
                    if i == *index {
                        value != &old_value
                    } else {
                        value != &PartialValue::Bottom
                    }
                });
                *row = Self::Dense(values);
                changed
            }
            (
                row @ Self::Sparse { .. },
                Self::Sparse {
                    len: rhs_len,
                    index: rhs_index,
                    value: rhs_value,
                },
            ) => {
                let Self::Sparse { len, index, value } = row else {
                    unreachable!()
                };
                assert_eq!(*len, rhs_len);
                if *index == rhs_index {
                    value.join_mut(rhs_value)
                } else {
                    let changed = rhs_value != PartialValue::Bottom;
                    let mut values = vec![PartialValue::bottom(); *len];
                    values[*index] = value.clone();
                    values[rhs_index] = rhs_value;
                    *row = Self::Dense(values);
                    changed
                }
            }
        }
    }

    fn meet_mut(&mut self, other: Self) -> bool {
        match (self, other) {
            (Self::Dense(lhs), Self::Dense(rhs)) => meet_dense(lhs, rhs),
            (Self::Dense(values), Self::Sparse { len, index, value }) => {
                assert_eq!(values.len(), len);
                let mut changed = values[index].meet_mut(value);
                for (i, value) in values.iter_mut().enumerate() {
                    if i != index && *value != PartialValue::Bottom {
                        *value = PartialValue::Bottom;
                        changed = true;
                    }
                }
                changed
            }
            (Self::Sparse { len, index, value }, Self::Dense(values)) => {
                assert_eq!(*len, values.len());
                value.meet_mut(values[*index].clone())
            }
            (
                row @ Self::Sparse { .. },
                Self::Sparse {
                    len: rhs_len,
                    index: rhs_index,
                    value: rhs_value,
                },
            ) => {
                let Self::Sparse { len, index, value } = row else {
                    unreachable!()
                };
                assert_eq!(*len, rhs_len);
                if *index == rhs_index {
                    value.meet_mut(rhs_value)
                } else {
                    let changed = *value != PartialValue::Bottom;
                    *row = Self::Dense(vec![PartialValue::Bottom; *len]);
                    changed
                }
            }
        }
    }
}

/// Join two complete rows component-wise.
fn join_dense<V: AbstractValue, N: PartialEq + PartialOrd>(
    lhs: &mut [PartialValue<V, N>],
    rhs: Vec<PartialValue<V, N>>,
) -> bool {
    assert_eq!(lhs.len(), rhs.len());
    zip_eq(lhs, rhs).fold(false, |changed, (lhs, rhs)| lhs.join_mut(rhs) | changed)
}

/// Meet two complete rows component-wise.
fn meet_dense<V: AbstractValue, N: PartialEq + PartialOrd>(
    lhs: &mut [PartialValue<V, N>],
    rhs: Vec<PartialValue<V, N>>,
) -> bool {
    assert_eq!(lhs.len(), rhs.len());
    zip_eq(lhs, rhs).fold(false, |changed, (lhs, rhs)| lhs.meet_mut(rhs) | changed)
}

impl<V, N> IntoIterator for ValueRow<V, N> {
    type Item = PartialValue<V, N>;

    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        let Self::Dense(values) = self else {
            unreachable!("sparse value rows are not operation results")
        };
        values.into_iter()
    }
}

#[cfg(test)]
mod tests {
    use ascent::Lattice;

    use super::ValueRow;
    use crate::passes::dataflow::{AbstractValue, PartialValue};

    #[derive(Clone, Debug, PartialEq, Eq, Hash)]
    struct TestValue(u8);

    impl AbstractValue for TestValue {}

    type TestRow = ValueRow<TestValue, usize>;

    #[test]
    fn sparse_updates_join_into_dense_row() {
        let mut row = TestRow::new(4);

        assert!(row.join_mut(TestRow::sparse(4, 2, PartialValue::Value(TestValue(7)),)));
        assert_eq!(
            row.values(),
            &[
                PartialValue::Bottom,
                PartialValue::Bottom,
                PartialValue::Value(TestValue(7)),
                PartialValue::Bottom,
            ]
        );
        assert!(!row.join_mut(TestRow::sparse(4, 2, PartialValue::Value(TestValue(7)),)));
    }

    #[test]
    fn dense_seed_joined_after_sparse_update_is_unchanged() {
        let mut row = TestRow::sparse(3, 1, PartialValue::Value(TestValue(5)));

        assert!(!row.join_mut(TestRow::new(3)));
        assert_eq!(
            row.values(),
            &[
                PartialValue::Bottom,
                PartialValue::Value(TestValue(5)),
                PartialValue::Bottom,
            ]
        );
    }

    #[test]
    fn sparse_updates_for_different_slots_join() {
        let mut row = TestRow::sparse(3, 0, PartialValue::Value(TestValue(1)));

        assert!(row.join_mut(TestRow::sparse(3, 2, PartialValue::Value(TestValue(2)),)));
        assert_eq!(
            row.values(),
            &[
                PartialValue::Value(TestValue(1)),
                PartialValue::Bottom,
                PartialValue::Value(TestValue(2)),
            ]
        );
    }
}
