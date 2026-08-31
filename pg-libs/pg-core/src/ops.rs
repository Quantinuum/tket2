use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::errors::PauliGraphError;
use crate::{GateType, Pauli};

pub(crate) fn string_to_paulis(pstr: &str) -> Result<Vec<Pauli>, PauliGraphError> {
    pstr.chars()
        .map(|pauli| match pauli {
            'I' => Ok(Pauli::I),
            'X' => Ok(Pauli::X),
            'Y' => Ok(Pauli::Y),
            'Z' => Ok(Pauli::Z),
            _ => Err(PauliGraphError::InvalidInputJson(format!(
                "Invalid Pauli string: contains invalid character: {}",
                pauli
            ))),
        })
        .collect()
}

fn paulis_to_string(paulis: &[Pauli]) -> String {
    paulis
        .iter()
        .map(|p| match p {
            Pauli::I => 'I',
            Pauli::X => 'X',
            Pauli::Y => 'Y',
            Pauli::Z => 'Z',
        })
        .collect()
}

fn serialize_pauli_string<S>(v: &[Pauli], s: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    s.serialize_str(&paulis_to_string(v))
}

fn serialize_pauli_string_bool_vec<S>(v: &Vec<(Vec<Pauli>, bool)>, s: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    use serde::ser::SerializeSeq;
    let mut seq = s.serialize_seq(Some(v.len()))?;
    for (paulis, sign) in v {
        seq.serialize_element(&(paulis_to_string(paulis), sign))?;
    }
    seq.end()
}

fn deserialize_pauli_string<'de, D>(deserializer: D) -> Result<Vec<Pauli>, D::Error>
where
    D: Deserializer<'de>,
{
    let s = String::deserialize(deserializer)?;
    string_to_paulis(&s).map_err(serde::de::Error::custom)
}

fn deserialize_pauli_string_bool_vec<'de, D>(
    deserializer: D,
) -> Result<Vec<(Vec<Pauli>, bool)>, D::Error>
where
    D: Deserializer<'de>,
{
    let items = Vec::<(String, bool)>::deserialize(deserializer)?;
    items
        .into_iter()
        .map(|(s, sign)| {
            Ok((
                string_to_paulis(&s).map_err(serde::de::Error::custom)?,
                sign,
            ))
        })
        .collect()
}

/// An operation node in a Pauli graph.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(tag = "type")]
pub enum Op {
    /// A Pauli rotation.
    Rotation {
        /// rotation data
        data: RotationData,
    },
    /// A measurement operation.
    Measure {
        /// measurement data
        data: MeasureData,
    },
    /// A reset operation.
    Reset {
        /// reset data
        data: ResetData,
    },
    /// An opaque operation.
    BlackBox {
        /// blackbox data
        data: BlackBoxData,
    },
    /// A box of operations with classical conditions.
    ConditionalBox {
        /// conditional box data
        data: ConditionalBoxData,
    },
    /// A forward facing Clifford tableau operation.
    Tableau {
        /// tableau data
        data: TableauData,
    },
    /// A gate operation.
    Gate {
        /// gate data
        data: GateData,
    },
    /// Special op to indicate commuting boundaries, which can be treated as identities in many passes.
    SetBoundary,
}

/// Data associated with a Pauli rotation.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct RotationData {
    #[serde(
        serialize_with = "serialize_pauli_string",
        deserialize_with = "deserialize_pauli_string"
    )]
    pub(crate) string: Vec<Pauli>,
    angle: f64,
}

impl RotationData {
    /// Creates rotation data from a Pauli string and rotation angle.
    pub fn new(string: Vec<Pauli>, angle: f64) -> Self {
        Self { string, angle }
    }

    /// Returns the Pauli string acted on by the rotation.
    pub fn get_string(&self) -> &Vec<Pauli> {
        &self.string
    }

    /// Returns the rotation angle.
    pub fn get_angle(&self) -> f64 {
        self.angle
    }
}

/// Data associated with a Pauli measurement.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct MeasureData {
    #[serde(
        serialize_with = "serialize_pauli_string",
        deserialize_with = "deserialize_pauli_string"
    )]
    pub(crate) string: Vec<Pauli>,
    sign: bool,
    cbit: usize,
}

impl MeasureData {
    /// Creates measurement data from a Pauli string, sign, and classical bit target.
    pub fn new(string: Vec<Pauli>, sign: bool, cbit: usize) -> Self {
        Self { string, sign, cbit }
    }

    /// Returns the measured Pauli string.
    pub fn get_string(&self) -> &Vec<Pauli> {
        &self.string
    }

    /// Returns the sign of the Pauli string.
    pub fn get_sign(&self) -> bool {
        self.sign
    }

    /// Returns the destination classical bit.
    pub fn get_cbit(&self) -> usize {
        self.cbit
    }
}

/// Data associated with a reset operation.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ResetData {
    #[serde(
        serialize_with = "serialize_pauli_string",
        deserialize_with = "deserialize_pauli_string"
    )]
    pub(crate) first_string: Vec<Pauli>,
    #[serde(
        serialize_with = "serialize_pauli_string",
        deserialize_with = "deserialize_pauli_string"
    )]
    pub(crate) second_string: Vec<Pauli>,
    first_sign: bool,
    second_sign: bool,
}

impl ResetData {
    /// Creates reset data from two anti-commuting Pauli strings and their corresponding signs.
    pub fn new(
        first_string: Vec<Pauli>,
        second_string: Vec<Pauli>,
        first_sign: bool,
        second_sign: bool,
    ) -> Self {
        Self {
            first_string,
            second_string,
            first_sign,
            second_sign,
        }
    }

    /// Returns the first Pauli string involved in the reset.
    pub fn get_first_string(&self) -> &Vec<Pauli> {
        &self.first_string
    }

    /// Returns the second Pauli string involved in the reset.
    pub fn get_second_string(&self) -> &Vec<Pauli> {
        &self.second_string
    }

    /// Returns the sign associated with the first string.
    pub fn get_first_sign(&self) -> bool {
        self.first_sign
    }

    /// Returns the sign associated with the second string.
    pub fn get_second_sign(&self) -> bool {
        self.second_sign
    }
}

/// Data associated with an opaque black-box operation.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct BlackBoxData {
    qubits: Vec<usize>,
    content: String,
}

impl BlackBoxData {
    /// Creates black-box data for the provided qubits and payload string.
    pub fn new(qubits: Vec<usize>, content: String) -> Self {
        Self { qubits, content }
    }

    /// Returns the qubits touched by the black-box operation.
    pub fn get_qubits(&self) -> &Vec<usize> {
        &self.qubits
    }

    /// Returns the opaque payload content.
    pub fn get_content(&self) -> &String {
        &self.content
    }
}

/// Data for a conditional box of operations.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ConditionalBoxData {
    conditional_bits: Vec<usize>,
    conditional_values: Vec<bool>,
    pub(crate) ops: Vec<Op>,
}

impl ConditionalBoxData {
    /// Creates a conditional box from its operations and classical control values.
    pub fn new(ops: Vec<Op>, conditional_bits: Vec<usize>, conditional_values: Vec<bool>) -> Self {
        Self {
            conditional_bits,
            conditional_values,
            ops,
        }
    }

    /// Returns the classical bits used as conditions.
    pub fn get_conditional_bits(&self) -> &Vec<usize> {
        &self.conditional_bits
    }

    /// Returns the boolean values required on the conditional bits.
    pub fn get_conditional_values(&self) -> &Vec<bool> {
        &self.conditional_values
    }

    /// Returns the operations contained in the box.
    pub fn get_ops(&self) -> &Vec<Op> {
        &self.ops
    }
}

/// Tableau output data for a Clifford operation.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct TableauData {
    #[serde(
        serialize_with = "serialize_pauli_string_bool_vec",
        deserialize_with = "deserialize_pauli_string_bool_vec"
    )]
    pub(crate) z_outputs: Vec<(Vec<Pauli>, bool)>,
    #[serde(
        serialize_with = "serialize_pauli_string_bool_vec",
        deserialize_with = "deserialize_pauli_string_bool_vec"
    )]
    pub(crate) x_outputs: Vec<(Vec<Pauli>, bool)>,
}

impl TableauData {
    /// Creates tableau data from $Z$ and $X$ output images.
    pub fn new(z_outputs: Vec<(Vec<Pauli>, bool)>, x_outputs: Vec<(Vec<Pauli>, bool)>) -> Self {
        Self {
            z_outputs,
            x_outputs,
        }
    }

    /// Returns the images of the input $Z$ operators.
    pub fn get_z_outputs(&self) -> &Vec<(Vec<Pauli>, bool)> {
        &self.z_outputs
    }

    /// Returns the images of the input $X$ operators.
    pub fn get_x_outputs(&self) -> &Vec<(Vec<Pauli>, bool)> {
        &self.x_outputs
    }
}

/// Data associated with a gate operation.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct GateData {
    gate_type: GateType,
    params: Vec<f64>,
    args: Vec<usize>,
    data: Option<String>,
    conditional_bits: Vec<usize>,
    conditional_values: Vec<bool>,
}

impl GateData {
    /// Creates gate data with a gate type and positional arguments.
    pub fn new(gate_type: GateType, args: Vec<usize>) -> Self {
        Self {
            gate_type,
            params: vec![],
            args,
            data: None,
            conditional_bits: vec![],
            conditional_values: vec![],
        }
    }

    /// Sets the floating-point parameters associated with the gate.
    pub fn with_params(mut self, params: Vec<f64>) -> Self {
        self.params = params;
        self
    }

    /// Sets the optional opaque payload associated with the gate.
    pub fn with_data(mut self, data: String) -> Self {
        self.data = Some(data);
        self
    }

    /// Sets the classical conditions required for the gate to execute.
    pub fn with_conditional(
        mut self,
        conditional_bits: Vec<usize>,
        conditional_values: Vec<bool>,
    ) -> Self {
        self.conditional_bits = conditional_bits;
        self.conditional_values = conditional_values;
        self
    }

    /// Returns the gate type.
    pub fn get_gate_type(&self) -> &GateType {
        &self.gate_type
    }

    /// Returns the gate parameters.
    pub fn get_params(&self) -> &Vec<f64> {
        &self.params
    }

    /// Returns the gate argument indices.
    pub fn get_args(&self) -> &Vec<usize> {
        &self.args
    }

    /// Returns the optional opaque gate payload.
    pub fn get_data(&self) -> &Option<String> {
        &self.data
    }

    /// Returns the classical bits used as conditions.
    pub fn get_conditional_bits(&self) -> &Vec<usize> {
        &self.conditional_bits
    }

    /// Returns the required values for the classical conditions.
    pub fn get_conditional_values(&self) -> &Vec<bool> {
        &self.conditional_values
    }
}
