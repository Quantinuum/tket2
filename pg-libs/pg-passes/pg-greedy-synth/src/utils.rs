/// Returns mutable references to the elements at indices `i` and `j`.
///
/// # Panics
///
/// Panics if either index is outside the slice or if the indices are equal.
pub(crate) fn get_two_mut<T>(v: &mut [T], i: usize, j: usize) -> (&mut T, &mut T) {
    assert!(i < v.len() && j < v.len(), "indices must be in bounds");
    assert_ne!(i, j, "indices must be distinct");
    let (a, b) = if i < j {
        let (left, right) = v.split_at_mut(j);
        (&mut left[i], &mut right[0])
    } else {
        let (left, right) = v.split_at_mut(i);
        (&mut right[0], &mut left[j])
    };
    (a, b)
}
