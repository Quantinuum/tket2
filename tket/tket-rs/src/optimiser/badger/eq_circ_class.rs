use std::io;
use std::path::Path;

use hugr::Hugr;
use hugr::envelope::serde_with::AsStringEnvelope;
use itertools::Itertools;
use serde_with::serde_as;

use crate::circuit::Circuit;

use super::qtz_circuit::load_ecc_set;

#[derive(Debug, Clone)]
pub enum EqCircClassError {
    NoRepresentative,
}

/// A set of circuits forming an Equivalence Circuit Class (ECC).
///
/// The set contains a distinguished circuit called the representative circuit,
/// typically chosen to be the smallest circuit in the set.
#[serde_as]
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct EqCircClass {
    // TODO: Encode envelopes in binary format.
    // This requires adding a new `AsBinaryEnvelope` adaptor in `hugr`.
    #[serde_as(as = "AsStringEnvelope")]
    rep_circ: Hugr,
    /// Other equivalent circuits to the representative.
    #[serde_as(as = "Vec<AsStringEnvelope>")]
    other_circs: Vec<Hugr>,
}

impl EqCircClass {
    /// Create a new equivalence class with a representative circuit.
    pub fn new(rep_circ: Circuit, other_circs: impl IntoIterator<Item = Circuit>) -> Self {
        Self {
            rep_circ: rep_circ.into_hugr(),
            other_circs: other_circs.into_iter().map(|c| c.into_hugr()).collect(),
        }
    }

    /// The representative circuit of the equivalence class.
    pub fn rep_circ(&self) -> &Hugr {
        &self.rep_circ
    }

    /// The other circuits in the equivalence class.
    pub fn others(&self) -> &[Hugr] {
        &self.other_circs
    }

    /// All circuits in the equivalence class.
    pub fn circuits(&self) -> impl Iterator<Item = &Hugr> {
        std::iter::once(&self.rep_circ).chain(self.other_circs.iter())
    }

    /// Consume into circuits of the equivalence class.
    pub fn into_circuits(self) -> impl Iterator<Item = Hugr> {
        std::iter::once(self.rep_circ).chain(self.other_circs)
    }

    /// The number of circuits in the equivalence class.
    ///
    /// An ECC always has a representative circuit, so this method will always
    /// return an integer strictly greater than 0.
    pub fn n_circuits(&self) -> usize {
        self.other_circs.len() + 1
    }

    /// Create an equivalence class from a set of circuits.
    ///
    /// The smallest circuit is chosen as the representative.
    pub fn from_circuits(
        circs: impl IntoIterator<Item = Circuit>,
    ) -> Result<Self, EqCircClassError> {
        let mut circs: Vec<Circuit> = circs.into_iter().collect();

        if circs.is_empty() {
            return Err(EqCircClassError::NoRepresentative);
        };

        // Find the index for the smallest circuit
        let min_index = circs
            .iter()
            .position_min_by_key(|c| c.num_operations())
            .unwrap();
        let representative = circs.swap_remove(min_index);
        Ok(Self::new(representative, circs))
    }
}

/// Load a set of equivalence classes from a JSON file.
pub fn load_eccs_json_file(path: impl AsRef<Path>) -> io::Result<Vec<EqCircClass>> {
    let all_circs = load_ecc_set(path)?;

    Ok(all_circs
        .into_values()
        .map(EqCircClass::from_circuits)
        .collect::<Result<Vec<_>, _>>()
        .unwrap())
}
