//! A priority queue for states, with support for maximum size and considering
//! every state at most once.

use delegate::delegate;
use fxhash::{FxHashMap, FxHashSet};
use priority_queue::DoublePriorityQueue;

use super::State;

/// A min-priority queue for states in the search space.
///
/// The states are ordered according to their cost. Uses hashes internally to
/// store the states.
///
/// States can be inserted at most once. Any further insertions will fail.
#[derive(Debug, Clone, Default)]
pub(super) struct StatePQueue<S, P: Ord> {
    queue: DoublePriorityQueue<u64, P>,
    hash_lookup: FxHashMap<u64, S>,
    seen_hashes: FxHashSet<u64>,
    max_size: usize,
    /// The all-time best states discovered.
    all_time_best: Option<DoublePriorityQueue<u64, P>>,
}

/// An entry in the priority queue.
pub(super) struct Entry<S, P, H> {
    /// The state.
    pub state: S,
    /// The cost of the state.
    pub cost: P,
    /// The hash of the state.
    pub hash: H,
}

impl<S: Clone, P: Clone + Ord> StatePQueue<S, P> {
    /// Create a new HugrPQ with a cost function and some initial capacity.
    pub fn new(max_size: usize, n_best: Option<usize>) -> Self {
        Self {
            queue: DoublePriorityQueue::with_capacity(max_size),
            hash_lookup: Default::default(),
            seen_hashes: Default::default(),
            max_size,
            all_time_best: n_best.map(|n| DoublePriorityQueue::with_capacity(n)),
        }
    }

    /// Reference to the minimal circuit in the queue.
    #[expect(unused)]
    pub fn peek(&self) -> Option<Entry<&S, &P, u64>> {
        let (&hash, cost) = self.queue.peek_min()?;
        let state = self.hash_lookup.get(&hash)?;
        Some(Entry { state, cost, hash })
    }

    /// Push into the queue.
    ///
    /// If the queue is full, the element with the highest cost will be dropped.
    ///
    /// Return the entry that was just inserted. If one of:
    ///  - hashing or costing failed,
    ///  - the priority queue was full and the cost higher than the highest cost
    ///    in the queue, or
    ///  - the state has already been seen,
    ///
    /// return `None` and the state will be discarded.
    pub fn push<Ctx>(&mut self, state: S, context: &Ctx) -> Option<Entry<&S, &P, u64>>
    where
        S: State<Ctx, Cost = P>,
    {
        let hash = state.hash(context)?;
        let cost = state.cost(context)?;
        self.push_unchecked(state, hash, cost)
    }

    pub fn push_unchecked(&mut self, state: S, hash: u64, cost: P) -> Option<Entry<&S, &P, u64>> {
        if !self.check_accepted(&cost) || !self.seen_hashes.insert(hash) {
            return None;
        }
        if self.len() >= self.max_size {
            self.pop_max();
        }
        self.queue.push(hash, cost.clone());

        self.push_all_time_best(hash, cost);

        let (_, cost_ref) = self.queue.get(&hash).expect("just inserted");
        let state_ref = self.hash_lookup.entry(hash).or_insert(state);

        Some(Entry {
            state: state_ref,
            cost: cost_ref,
            hash,
        })
    }

    #[inline]
    fn push_all_time_best(&mut self, hash: u64, cost: P) {
        let Some(all_time_best) = self.all_time_best.as_ref() else {
            return;
        };
        if all_time_best.len() == all_time_best.capacity() {
            if Some(&cost) < all_time_best.peek_max().map(|(_, cost)| cost) {
                self.pop_max_all_time_best();
            } else {
                return;
            }
        }
        self.all_time_best.as_mut().unwrap().push(hash, cost);
    }

    /// Pop the minimal state from the queue.
    pub fn pop(&mut self) -> Option<Entry<S, P, u64>> {
        let (hash, cost) = self.queue.pop_min()?;
        let state = if self
            .all_time_best
            .as_ref()
            .map(|atb| !atb.contains(&hash))
            .unwrap_or_default()
        {
            self.hash_lookup.remove(&hash)?
        } else {
            self.hash_lookup.get(&hash)?.clone()
        };
        Some(Entry { state, cost, hash })
    }

    /// Pop the maximal state from the queue.
    fn pop_max(&mut self) -> Option<Entry<S, P, u64>>
    where
        S: Clone,
    {
        let (hash, cost) = self.queue.pop_max()?;
        let state = if self
            .all_time_best
            .as_ref()
            .map(|atb| !atb.contains(&hash))
            .unwrap_or_default()
        {
            self.hash_lookup.remove(&hash)?
        } else {
            self.hash_lookup.get(&hash)?.clone()
        };
        Some(Entry { state, cost, hash })
    }

    fn pop_max_all_time_best(&mut self) -> bool {
        let Some((hash, _)) = self.all_time_best.as_mut().and_then(|atb| atb.pop_max()) else {
            return false;
        };
        if !self.queue.contains(&hash) {
            self.hash_lookup.remove(&hash);
        }
        true
    }

    /// Discard the largest elements of the queue.
    ///
    /// Only keep up to `max_size` elements.
    #[expect(unused)]
    pub fn truncate(&mut self, max_size: usize) {
        while self.queue.len() > max_size {
            let (hash, _) = self.queue.pop_max().unwrap();
            self.hash_lookup.remove(&hash);
        }
    }

    /// The largest cost in the queue.
    pub fn max_cost(&self) -> Option<&P> {
        self.queue.peek_max().map(|(_, cost)| cost)
    }

    /// The number of unique hashes in the queue.
    pub fn num_seen_hashes(&self) -> usize {
        self.seen_hashes.len()
    }

    /// Returns `true` if an element with the given cost would be accepted.
    ///
    /// If `false`, the element will be dropped if passed to
    /// [`StatePQueue::push`].
    pub fn check_accepted(&self, cost: &P) -> bool {
        if self.max_size == 0 {
            return false;
        }
        if self.len() < self.max_size {
            return true;
        }
        cost < self.max_cost().unwrap()
    }

    /// Returns `true` is the queue is at capacity.
    pub fn is_full(&self) -> bool {
        self.queue.len() >= self.max_size
    }

    /// Consume the priority queue into a vector with the all-time best states.
    pub fn into_all_time_best(mut self) -> Option<Vec<S>> {
        let all_time_best = self.all_time_best?;
        Some(
            all_time_best
                .into_sorted_iter()
                .map(|(hash, _)| self.hash_lookup.remove(&hash).unwrap())
                .collect(),
        )
    }

    delegate! {
        to self.queue {
            pub fn len(&self) -> usize;
            pub fn is_empty(&self) -> bool;
        }
    }
}

#[cfg(test)]
mod tests {
    use itertools::Itertools;

    use super::*;

    #[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
    struct DummyState(usize);

    impl DummyState {
        fn hash(&self) -> u64 {
            self.0 as u64
        }
        fn cost(&self) -> usize {
            self.0
        }
    }

    impl State<()> for DummyState {
        type Cost = usize;

        fn hash(&self, (): &()) -> Option<u64> {
            Some(DummyState::hash(self))
        }

        fn cost(&self, (): &()) -> Option<Self::Cost> {
            Some(DummyState::cost(self))
        }

        fn next_states(&self, (): &mut ()) -> Vec<Self> {
            unimplemented!()
        }
    }

    impl StatePQueue<DummyState, usize> {
        fn get_enqueued(&self) -> Vec<DummyState> {
            let mut enqueued = self
                .queue
                .iter()
                .map(|(hash, _)| *self.hash_lookup.get(hash).unwrap())
                .collect_vec();
            enqueued.sort_unstable();
            enqueued
        }

        fn get_all_time_best(&self) -> Vec<DummyState> {
            let mut all_time_best = self
                .all_time_best
                .as_ref()
                .unwrap()
                .iter()
                .map(|(hash, _)| *self.hash_lookup.get(hash).unwrap())
                .collect_vec();
            all_time_best.sort_unstable();
            all_time_best
        }
    }

    #[test]
    fn test_queue_truncation() {
        let max_size = 5;
        let n_best = 3;
        let mut pq = StatePQueue::new(max_size, Some(n_best));

        // Insert 10 states with decreasing cost
        for i in (0..10).rev() {
            pq.push(DummyState(i), &());
        }

        // Only max_size states should be in the queue
        assert_eq!(pq.len(), max_size);

        // The states with the lowest costs should be present (0..5)
        assert_eq!(
            pq.get_enqueued(),
            (0..max_size).map(DummyState).collect_vec()
        );

        // The states with the lowest costs should be in all_time_best (0..3)
        assert_eq!(
            pq.get_all_time_best(),
            (0..n_best).map(DummyState).collect_vec()
        );

        // Remove 4, 3, 2
        for i in [4, 3, 2] {
            assert_eq!(pq.pop_max().map(|e| e.state), Some(DummyState(i)));
        }

        // The states with the lowest costs should be present: [0, 1]
        assert_eq!(pq.get_enqueued(), (0..2).map(DummyState).collect_vec());
        // but all time best should be unchanged
        assert_eq!(
            pq.get_all_time_best(),
            (0..n_best).map(DummyState).collect_vec()
        );

        // Remove min (0)
        assert_eq!(pq.pop().map(|e| e.state), Some(DummyState(0)));

        // Should be present: [1]
        assert_eq!(pq.get_enqueued(), [DummyState(1)]);
        // but all time best should be unchanged
        assert_eq!(
            pq.get_all_time_best(),
            (0..n_best).map(DummyState).collect_vec()
        );

        // Finally, remove all all_time_best
        for _ in 0..n_best {
            assert!(pq.pop_max_all_time_best());
        }

        // Should be present: [1]
        assert_eq!(pq.get_enqueued(), [DummyState(1)]);
        assert_eq!(pq.hash_lookup.len(), 1);
        // Should be empty
        assert!(pq.all_time_best.as_ref().unwrap().is_empty());

        // Finally, remove the last state
        assert!(pq.pop().is_some());
        assert!(pq.hash_lookup.is_empty());
    }
}
