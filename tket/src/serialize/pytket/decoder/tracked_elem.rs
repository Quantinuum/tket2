//! Pytket qubit and bit elements that we track during decoding.
use std::hash::Hasher;
use std::sync::{Arc, LazyLock};

use hugr::extension::prelude::{bool_t, qb_t};
use hugr::types::Type;
use tket_json_rs::register::ElementId as PytketRegister;

use crate::serialize::pytket::RegisterHash;

/// An internal lightweight identifier for a [`TrackedQubit`] in the decoder.
#[derive(Clone, Copy, Debug, derive_more::Display, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub(super) struct TrackedQubitId(#[display(transparent)] pub usize);

/// An internal lightweight identifier for a [`TrackedBit`] in the decoder.
#[derive(Clone, Copy, Debug, derive_more::Display, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub(super) struct TrackedBitId(#[display(transparent)] pub usize);

/// An identifier for a pytket qubit register in the data carried by a wire.
///
/// After a pytket circuit assigns a new value to the register, older
/// [`TrackedQubit`]s referring to it become _outdated_.
///
/// Outdated values no longer correspond to a pytket circuit register, but they
/// can still be found in the wires of the hugr being extracted.
#[derive(Debug, Clone, Eq, derive_more::Display)]
#[display("{reg}")]
pub struct TrackedQubit {
    /// The id of this tracked qubit in the [`WireTracker`].
    id: TrackedQubitId,
    /// Whether this tracked qubit is outdated, meaning that we have seen the
    /// register in a newer wire.
    outdated: bool,
    /// The pytket register for this tracked element.
    reg: Arc<PytketRegister>,
    /// The hash of the pytket register for this tracked element, used to
    /// speed up hashing and equality checks.
    reg_hash: RegisterHash,
}

/// An identifier for a pytket bit register in the data carried by a wire.
///
/// After a pytket circuit assigns a new value to the register, older
/// [`TrackedBit`]s referring to it become _outdated_.
///
/// Outdated values no longer correspond to a pytket circuit register, but they
/// can still be found in the wires of the hugr being extracted.
#[derive(Debug, Clone, Eq, derive_more::Display)]
#[display("{reg}")]
pub struct TrackedBit {
    /// The id of this tracked bit in the [`WireTracker`].
    id: TrackedBitId,
    /// Whether this tracked bit is outdated, meaning that we have seen the
    /// register in a newer wire.
    outdated: bool,
    /// The pytket register for this tracked element.
    reg: Arc<PytketRegister>,
    /// The hash of the pytket register for this tracked element, used to
    /// speed up hashing and equality checks.
    //
    // TODO: We could put this along with `reg` in a `PytketResource` struct
    // that gets used around the crate.
    reg_hash: RegisterHash,
}

impl TrackedQubit {
    /// Returns a new tracked qubit.
    pub(super) fn new(id: TrackedQubitId, reg: Arc<PytketRegister>) -> Self {
        let reg_hash = RegisterHash::from(reg.as_ref());
        Self::new_with_hash(id, reg, reg_hash)
    }

    /// Returns a new tracked qubit.
    pub(super) fn new_with_hash(
        id: TrackedQubitId,
        reg: Arc<PytketRegister>,
        reg_hash: RegisterHash,
    ) -> Self {
        Self {
            id,
            outdated: false,
            reg,
            reg_hash,
        }
    }

    /// Returns the pytket register for this tracked element.
    pub fn pytket_register(&self) -> &PytketRegister {
        &self.reg
    }

    /// Returns the pytket register for this tracked element.
    pub fn pytket_register_arc(&self) -> Arc<PytketRegister> {
        self.reg.clone()
    }

    /// Returns the type of the element.
    pub fn ty(&self) -> Arc<Type> {
        static QUBIT_TYPE: LazyLock<Arc<Type>> = LazyLock::new(|| qb_t().into());
        QUBIT_TYPE.clone()
    }

    /// Returns the id of this tracked qubit.
    pub(super) fn id(&self) -> TrackedQubitId {
        self.id
    }

    /// Returns the hash of the pytket register for this tracked element.
    pub(super) fn reg_hash(&self) -> RegisterHash {
        self.reg_hash
    }

    /// Returns `true` if the element has been overwritten by a new value.
    pub fn is_outdated(&self) -> bool {
        self.outdated
    }

    /// Mark the element as outdated.
    pub(super) fn mark_outdated(&mut self) {
        self.outdated = true;
    }
}

impl TrackedBit {
    /// Returns a new tracked bit.
    pub(super) fn new(id: TrackedBitId, reg: Arc<PytketRegister>) -> Self {
        let reg_hash = RegisterHash::from(reg.as_ref());
        Self::new_with_hash(id, reg, reg_hash)
    }

    /// Returns a new tracked bit.
    pub(super) fn new_with_hash(
        id: TrackedBitId,
        reg: Arc<PytketRegister>,
        reg_hash: RegisterHash,
    ) -> Self {
        Self {
            id,
            outdated: false,
            reg,
            reg_hash,
        }
    }

    /// Returns the pytket register for this tracked element.
    pub fn pytket_register(&self) -> &PytketRegister {
        &self.reg
    }

    /// Returns the pytket register for this tracked element.
    pub fn pytket_register_arc(&self) -> Arc<PytketRegister> {
        self.reg.clone()
    }

    /// Returns the type of the element.
    pub fn ty(&self) -> Arc<Type> {
        static BOOL_TYPE: LazyLock<Arc<Type>> = LazyLock::new(|| bool_t().into());
        BOOL_TYPE.clone()
    }

    /// Returns the id of this tracked bit.
    pub(super) fn id(&self) -> TrackedBitId {
        self.id
    }

    /// Returns the hash of the pytket register for this tracked element.
    pub(super) fn reg_hash(&self) -> RegisterHash {
        self.reg_hash
    }

    /// Returns `true` if the element has been overwritten by a new value.
    pub fn is_outdated(&self) -> bool {
        self.outdated
    }

    /// Mark the element as outdated.
    pub(super) fn mark_outdated(&mut self) {
        self.outdated = true;
    }
}

impl PartialEq for TrackedQubit {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id && self.outdated == other.outdated && self.reg_hash == other.reg_hash
    }
}

impl PartialEq for TrackedBit {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id && self.outdated == other.outdated && self.reg_hash == other.reg_hash
    }
}

impl std::hash::Hash for TrackedQubit {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
        self.outdated.hash(state);
        self.reg_hash.hash(state);
    }
}

impl std::hash::Hash for TrackedBit {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
        self.outdated.hash(state);
        self.reg_hash.hash(state);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hugr::extension::prelude::{bool_t, qb_t};
    use rstest::rstest;
    use std::sync::Arc;
    use tket_json_rs::register::ElementId;

    #[rstest]
    fn tracked_qubit_basic_behaviour() {
        let reg = Arc::new(ElementId("q".to_string(), vec![0]));
        let mut tq = TrackedQubit::new(TrackedQubitId(0), reg.clone());

        assert!(!tq.is_outdated());
        assert_eq!(tq.pytket_register(), &*reg);
        assert_eq!(tq.pytket_register_arc(), reg);
        assert_eq!(&*tq.ty(), &qb_t());

        tq.mark_outdated();
        assert!(tq.is_outdated());
    }

    #[rstest]
    fn tracked_bit_basic_behaviour() {
        let reg = Arc::new(ElementId("c".to_string(), vec![1]));
        let mut tb = TrackedBit::new(TrackedBitId(0), reg.clone());

        assert!(!tb.is_outdated());
        assert_eq!(tb.pytket_register(), &*reg);
        assert_eq!(tb.pytket_register_arc(), reg);
        assert_eq!(&*tb.ty(), &bool_t());

        tb.mark_outdated();
        assert!(tb.is_outdated());
    }
}
