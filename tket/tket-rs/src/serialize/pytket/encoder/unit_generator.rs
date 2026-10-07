//! This module contains the [`RegisterUnitGenerator`] struct, which is used to
//! generate fresh pytket register names for qubits and bits in a circuit.

use tket_json_rs::register::ElementId as RegisterUnit;

/// A utility class for finding new unused qubit/bit names.
#[derive(Debug, Clone, Default)]
pub struct RegisterUnitGenerator {
    /// The next index to use for a new register.
    next_unit: u16,
    /// The register name to use.
    register: String,
}

impl RegisterUnitGenerator {
    /// Create a new [`RegisterUnitGenerator`]
    ///
    /// Scans the set of existing registers to find the last used index, and
    /// starts generating new unit names from there.
    pub fn new<'a>(
        register: impl ToString,
        existing: impl IntoIterator<Item = &'a RegisterUnit>,
    ) -> Self {
        let register = register.to_string();
        let mut last_unit: Option<u16> = None;
        for reg in existing {
            if reg.0 != register {
                continue;
            }
            last_unit = Some(last_unit.unwrap_or_default().max(reg.1[0] as u16));
        }
        RegisterUnitGenerator {
            register,
            next_unit: last_unit.map_or(0, |i| i + 1),
        }
    }

    /// Returns a fresh register unit.
    pub fn next(&mut self) -> RegisterUnit {
        let unit = self.next_unit;
        self.next_unit += 1;
        RegisterUnit(self.register.clone(), vec![unit as i64])
    }
}
