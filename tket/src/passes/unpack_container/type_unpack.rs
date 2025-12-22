//! Recursively analyse composite types to detect inner types.

use hugr::extension::prelude::qb_t;
use hugr::std_extensions::collections::array::{Array, ArrayKind};
use hugr::std_extensions::collections::borrow_array::BorrowArray;
use hugr::types::{CustomType, SumType, Type, TypeArg, TypeRowRV};
use std::cell::RefCell;
use std::collections::HashMap;

/// If a type is an option of the given element type.
pub fn is_opt_of(ty: &Type, elem_type: &Type) -> bool {
    if let Some(sum) = ty.as_sum()
        && let Some(inner) = sum.as_unary_option()
    {
        return inner == elem_type;
    }
    false
}

/// If a custom type is an array, return size and element type.
pub fn array_args<AT: ArrayKind>(ext: &CustomType) -> Option<(u64, &Type)> {
    AT::type_def()
        .check_custom(ext)
        .ok()
        .and_then(|_| match ext.args() {
            [TypeArg::BoundedNat(n), TypeArg::Runtime(elem_ty)] => Some((*n, elem_ty)),
            _ => None,
        })
}

/// Analyzes types to determine how they should be unpacked
/// such that the `element_type` is extracted out into individual wires.
/// Can recursively handle array and tuple composite types. Elements inside
/// other composite types (e.g. from unknown extensions) are ignored.
///
/// Arrays of `Option[element_type]` are treated as special cases
/// and unpacked as if they were arrays of `element_type`, assuming the array is "full"
/// (i.e. no `None` values). If this assumption is not true there will be a runtime error.
#[derive(Clone)]
pub struct TypeUnpacker {
    /// The target element type to analyze for.
    element_type: Type,
    /// Cache of unpacked types.
    cache: RefCell<HashMap<Type, Option<Vec<Type>>>>,
}

impl TypeUnpacker {
    /// Create a new instance of the [TypeUnpacker] for the given element type.
    pub fn new(element_type: Type) -> Self {
        Self {
            element_type,
            cache: RefCell::new(HashMap::new()),
        }
    }

    /// Create a new instance of the [TypeUnpacker] specifically for qubits.
    pub fn for_qubits() -> Self {
        Self::new(qb_t())
    }

    /// Compute the row produced when a type is unpacked.
    ///
    /// The row contains one entry per element type extracted,
    /// other row elements are the other elements that were not unpacked.
    ///
    /// For example a tuple of (array[bool; 2], array[qubit; 2]) when analyzing for qubits
    /// would produce the row [array[bool; 2], qubit, qubit].
    ///
    /// Uses memoization to avoid recomputing the same type.
    /// `None` if the type does not contain the element type.
    pub fn unpack_type(&self, ty: &Type) -> Option<Vec<Type>> {
        if self.cache.borrow().contains_key(ty) {
            return self.cache.borrow().get(ty).cloned().expect("checked above");
        }

        let unpacked = self._new_unpack_type(ty);
        // SAFETY: types form trees so no cycles, cache will not be corrupted
        self.cache.borrow_mut().insert(ty.clone(), unpacked.clone());
        unpacked
    }

    fn _new_unpack_type(&self, ty: &Type) -> Option<Vec<Type>> {
        if ty == &self.element_type {
            return Some(vec![self.element_type.clone()]);
        }

        if let Some(row) = ty.as_sum().and_then(SumType::as_tuple) {
            self.tuple_row(row)

            // other sums containing the element type are ignored.
        } else if let Some((size, elem_ty)) = ty
            .as_extension()
            .and_then(|ext| array_args::<Array>(ext).or_else(|| array_args::<BorrowArray>(ext)))
        {
            // Special case for Option[ElementType] since it is used in arrays.
            // Fragile - would be better with dedicated array type.
            // TODO remove and only support borrow arrays
            // Not sure how this can be improved without runtime operations being able to
            // take a compile time unknown number of elements.
            if is_opt_of(elem_ty, &self.element_type) {
                Some(vec![self.element_type.clone(); size as usize])
            } else {
                self.unpack_type(elem_ty).map(|inner| {
                    let total_size = size as usize * inner.len();
                    let mut result = Vec::with_capacity(total_size);
                    for _ in 0..size {
                        result.extend_from_slice(&inner);
                    }
                    result
                })
            }
        } else {
            None
        }
    }

    fn tuple_row(&self, row: &TypeRowRV) -> Option<Vec<Type>> {
        let mut any_element = false;
        let unpacked_row = row
            .iter()
            .flat_map(|t| {
                let t = &t.clone().try_into_type().expect("unexpected row variable.");
                match self.unpack_type(t) {
                    Some(inner) => {
                        any_element = true;
                        inner
                    }
                    None => vec![t.clone()],
                }
            })
            .collect::<Vec<_>>();
        any_element.then_some(unpacked_row)
    }

    /// Count the number of wires in a row in an unpacked type.
    pub fn num_unpacked_wires(&self, ty: &Type) -> usize {
        self.unpack_type(ty).as_ref().map_or(1, Vec::len)
    }

    /// Report if a type contains the element type.
    pub fn contains_element_type(&self, ty: &Type) -> bool {
        self.unpack_type(ty).is_some()
    }

    /// Get the element type this analyzer is configured for.
    pub fn element_type(&self) -> &Type {
        &self.element_type
    }
}

/// Check if a type is an array with the given element type
pub fn is_array_of<AT: ArrayKind>(ty: &Type, elem_type: &Type) -> Option<u64> {
    ty.as_extension()
        .and_then(array_args::<AT>)
        .and_then(|(size, e_ty)| (e_ty == elem_type).then_some(size))
}

impl Default for TypeUnpacker {
    fn default() -> Self {
        Self::for_qubits()
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use hugr::extension::prelude::{bool_t, option_type, usize_t};
    use hugr::std_extensions::collections::array::array_type;
    use rstest::rstest;

    #[test]
    fn test_primitive_types() {
        let analyzer = TypeUnpacker::for_qubits();

        let qubit_result = analyzer.unpack_type(&qb_t());
        let types = qubit_result.unwrap();
        assert_eq!(types.len(), 1);
        assert_eq!(types[0], qb_t());

        // Non-quantum types should not be containers
        assert!(!analyzer.contains_element_type(&bool_t()));
    }

    #[rstest]
    #[case::array(Array)]
    #[case::borrow(BorrowArray)]
    fn test_array_types<AK: ArrayKind>(#[case] _kind: AK) {
        let analyzer = TypeUnpacker::for_qubits();

        // Array of qubits should be a container with that many qubits
        let qubit_array = AK::ty(3, qb_t());
        let result = analyzer.unpack_type(&qubit_array);

        let types = result.unwrap();
        assert_eq!(types.len(), 3);
        assert!(types.iter().all(|t| t == &qb_t()));

        // Array of non-quantum types should not be a container
        let bool_array = AK::ty(5, bool_t());
        assert!(!analyzer.contains_element_type(&bool_array));

        // Nested arrays of qubits
        let nested_array = AK::ty(2, array_type(3, qb_t()));
        let result = analyzer.unpack_type(&nested_array);

        let types = result.unwrap();
        assert_eq!(types.len(), 6); // 2 arrays of 3 qubits each
        assert!(types.iter().all(|t| t == &qb_t()));
    }

    #[test]
    fn test_option_types() {
        let analyzer = TypeUnpacker::for_qubits();

        // Option<qubit> by itself should NOT be a container
        let opt_qubit = option_type(qb_t()).into();
        assert!(!analyzer.contains_element_type(&opt_qubit));

        // Array of Option<qubit> should be a special case (a container with that many qubits)
        let opt_qubit_array = array_type(4, option_type(qb_t()).into());
        let result = analyzer.unpack_type(&opt_qubit_array);

        let types = result.unwrap();
        assert_eq!(types.len(), 4);
        assert!(types.iter().all(|t| t == &qb_t()));

        // Option of non-quantum types should not be a container
        let opt_bool = option_type(bool_t()).into();
        assert!(!analyzer.contains_element_type(&opt_bool));
    }

    #[test]
    fn test_tuple_types() {
        let analyzer = TypeUnpacker::for_qubits();

        // Tuple with no qubits
        let no_qubit_tuple = Type::new_tuple(vec![bool_t(), usize_t()]);
        assert!(!analyzer.contains_element_type(&no_qubit_tuple));

        // Tuple with qubits
        let qubit_tuple = Type::new_tuple(vec![bool_t(), qb_t(), usize_t()]);
        let result = analyzer.unpack_type(&qubit_tuple);

        let types = result.unwrap();
        assert_eq!(types.len(), 3); // Only one qubit in the tuple
        assert_eq!(types[1], qb_t());

        // Tuple with multiple qubits
        let multi_qubit_tuple = Type::new_tuple(vec![qb_t(), bool_t(), qb_t()]);
        let result = analyzer.unpack_type(&multi_qubit_tuple);

        let types = result.unwrap();
        assert_eq!(types.len(), 3);
        assert_eq!(types[0], qb_t());
        assert_eq!(types[2], qb_t());

        // Nested tuple with qubits
        let nested_tuple = Type::new_tuple(vec![
            bool_t(),
            Type::new_tuple(vec![usize_t(), qb_t()]),
            usize_t(),
        ]);
        let result = analyzer.unpack_type(&nested_tuple);

        let types = result.unwrap();
        assert_eq!(types.len(), 4);
        assert_eq!(types[2], qb_t());
    }

    #[test]
    fn test_complex_types() {
        let analyzer = TypeUnpacker::for_qubits();

        // Array of tuples containing qubits
        let complex_type = array_type(2, Type::new_tuple(vec![bool_t(), qb_t()]));
        let result = analyzer.unpack_type(&complex_type);

        let types = result.unwrap();
        assert_eq!(types.len(), 4); // 2 arrays, each with one qubit in the tuple

        // Tuple containing array of qubits and standalone qubit
        let complex_type = Type::new_tuple(vec![array_type(3, qb_t()), qb_t()]);
        let result = analyzer.unpack_type(&complex_type);

        let types = result.unwrap();
        assert_eq!(types.len(), 4); // 3 from array + 1 standalone
        assert!(types.iter().all(|t| t == &qb_t()));

        // Super nested complex example
        let complex_type = Type::new_tuple(vec![
            bool_t(),
            array_type(
                2,
                Type::new_tuple(vec![qb_t(), array_type(3, option_type(qb_t()).into())]),
            ),
            qb_t(),
        ]);
        let result = analyzer.unpack_type(&complex_type);

        let types = result.unwrap();
        assert_eq!(types.len(), 10); // 1 + 2*(1 + 3) + 1 = 10 wires total
    }

    #[test]
    fn test_helper_functions() {
        let analyzer = TypeUnpacker::for_qubits();
        // Test unpacked_wires
        assert_eq!(analyzer.num_unpacked_wires(&bool_t()), 1);
        assert_eq!(
            analyzer.num_unpacked_wires(&Type::new_tuple(vec![qb_t(), bool_t(), qb_t()])),
            3
        );

        // Test is_array_of
        assert_eq!(
            is_array_of::<Array>(&array_type(5, qb_t()), &qb_t()),
            Some(5)
        );
        assert_eq!(is_array_of::<Array>(&qb_t(), &qb_t()), None);
        assert_eq!(
            is_array_of::<Array>(&array_type(3, bool_t()), &qb_t()),
            None
        );
    }

    #[test]
    fn test_caching() {
        let analyzer = TypeUnpacker::for_qubits();

        // Create a complex type that will take some processing
        let complex_type = Type::new_tuple(vec![
            array_type(10, qb_t()),
            Type::new_tuple(vec![qb_t(), qb_t(), qb_t()]),
        ]);

        // First call should process everything
        let result1 = analyzer.unpack_type(&complex_type);

        // Second call should use the cache
        let result2 = analyzer.unpack_type(&complex_type);

        assert_eq!(result1, result2);

        // Both should be correct
        let types = result1.unwrap();
        assert_eq!(types.len(), 13); // 10 from array + 3 from tuple
        assert!(types.iter().all(|t| t == &qb_t()));
    }

    #[test]
    fn test_non_qb_element() {
        // Test with bool type as the element type
        let bool_analyzer = TypeUnpacker::new(bool_t());

        // Bool itself should be detected
        let bool_result = bool_analyzer.unpack_type(&bool_t());
        let types = bool_result.unwrap();
        assert_eq!(types.len(), 1);
        assert_eq!(types[0], bool_t());

        // Array of bools should be detected
        let bool_array = array_type(3, bool_t());
        let result = bool_analyzer.unpack_type(&bool_array);
        let types = result.unwrap();
        assert_eq!(types.len(), 3);
        assert!(types.iter().all(|t| t == &bool_t()));

        // Tuple containing bools
        let bool_tuple = Type::new_tuple(vec![qb_t(), bool_t(), usize_t()]);
        let result = bool_analyzer.unpack_type(&bool_tuple);
        let types = result.unwrap();
        assert_eq!(types.len(), 3);
        assert_eq!(types[1], bool_t());

        // Non-bool types should not be containers for this analyzer
        assert!(!bool_analyzer.contains_element_type(&qb_t()));
        assert!(!bool_analyzer.contains_element_type(&usize_t()));

        // Test with usize type as the element type
        let usize_analyzer = TypeUnpacker::new(usize_t());

        // usize itself should be detected
        let usize_result = usize_analyzer.unpack_type(&usize_t());
        let types = usize_result.unwrap();
        assert_eq!(types.len(), 1);
        assert_eq!(types[0], usize_t());

        // Array of usize should be detected
        let usize_array = array_type(2, usize_t());
        let result = usize_analyzer.unpack_type(&usize_array);
        let types = result.unwrap();
        assert_eq!(types.len(), 2);
        assert!(types.iter().all(|t| t == &usize_t()));

        // Check element_type accessor
        assert_eq!(bool_analyzer.element_type(), &bool_t());
        assert_eq!(usize_analyzer.element_type(), &usize_t());
    }
}
