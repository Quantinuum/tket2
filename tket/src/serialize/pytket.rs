//! Serialization and deserialization of circuits using the `pytket` JSON format.

mod circuit;
mod config;
pub mod decoder;
pub mod encoder;
mod error;
pub mod extension;
pub mod opaque;
mod options;

pub use circuit::EncodedCircuit;
pub use config::{
    PytketDecoderConfig, PytketEncoderConfig, TypeTranslatorSet, default_decoder_config,
    default_encoder_config,
};
pub use encoder::PytketEncoderContext;
pub use error::{
    PytketDecodeError, PytketDecodeErrorInner, PytketEncodeError, PytketEncodeOpError,
};
pub use extension::PytketEmitter;
use hugr::core::HugrNode;
use hugr::ops::OpTag;
use hugr::std_extensions::arithmetic::float_types::float64_type;
use hugr::types::Type;
pub use options::{DecodeInsertionTarget, DecodeOptions, EncodeOptions};

use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::handle::NodeHandle;
use hugr::{Hugr, HugrView, Node};
#[cfg(test)]
mod tests;

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::path::Path;
use std::sync::LazyLock;
use std::{fs, io};

use tket_json_rs::circuit_json::SerialCircuit;
use tket_json_rs::register::{Bit, ElementId, Qubit};

use self::decoder::PytketDecoderContext;

use crate::extension::rotation::rotation_type;
pub use crate::passes::pytket::lower_to_pytket;

/// Encode and decode dataflow regions in HUGRs into pytket-like flat quantum circuits.
///
/// Implemented by [`SerialCircuit`], the JSON format used by tket1's `pytket` library.
pub trait TKETDecode: Sized {
    /// Error type of decoding errors.
    type DecodeError;
    /// Error type of encoding errors.
    type EncodeError<N: HugrNode>;
    /// Convert a serialized pytket circuit to a HUGR.
    ///
    /// The HUGR will contain a single function as entrypoint containing the
    /// decoded circuit.
    ///
    /// The function name will be determined by the `name` of the serialized
    /// circuit, if present, or will be empty otherwise.
    ///
    /// See [DecodeOptions] to define the options used by the decoder.
    ///
    /// # Arguments
    ///
    /// - `options`: The options for the decoder.
    ///
    /// # Returns
    ///
    /// The encoded circuit.
    fn decode(&self, options: DecodeOptions) -> Result<Hugr, Self::DecodeError>;

    /// Convert the serialized circuit into a function definition in an existing HUGR.
    ///
    /// Does **not** modify the HUGR's entrypoint.
    ///
    /// # Arguments
    ///
    /// - `hugr`: The HUGR to define the function in.
    /// - `target`: Where to insert the decoded circuit.
    /// - `options`: The options for the decoder.
    ///
    /// # Returns
    ///
    /// The node id of the defined function.
    fn decode_into(
        &self,
        // This cannot be a generic HugrMut since it is stored inside the `PytketDecoderContext` that we to be Send+Sync
        // (so that the extension decoder traits are dyn-compatible).
        hugr: &mut Hugr,
        target: DecodeInsertionTarget,
        options: DecodeOptions,
    ) -> Result<Node, Self::DecodeError>;

    /// Convert the circuit-like entrypoint region of a Hugr to a serialized
    /// pytket circuit.
    ///
    /// See [EncodeOptions] for the options used by the encoder.
    ///
    /// If the entrypoint region is not a dataflow region, an error will be returned.
    ///
    /// # Arguments
    ///
    /// - `hugr`: The Hugr to encode.
    /// - `options`: The options for the encoder.
    ///
    /// # Returns
    ///
    /// A serialized pytket circuit.
    fn encode<H: HugrView>(
        hugr: &H,
        options: EncodeOptions<H>,
    ) -> Result<Self, Self::EncodeError<H::Node>>;
}

impl TKETDecode for SerialCircuit {
    type DecodeError = PytketDecodeError;
    type EncodeError<N: HugrNode> = PytketEncodeError<N>;

    fn decode(&self, options: DecodeOptions) -> Result<Hugr, Self::DecodeError> {
        let mut hugr = Hugr::new();
        let main_func = self.decode_into(
            &mut hugr,
            DecodeInsertionTarget::Function { fn_name: None },
            options,
        )?;
        hugr.set_entrypoint(main_func);
        Ok(hugr)
    }

    fn decode_into(
        &self,
        hugr: &mut Hugr,
        target: DecodeInsertionTarget,
        options: DecodeOptions,
    ) -> Result<Node, Self::DecodeError> {
        let mut decoder = PytketDecoderContext::new(self, hugr, target, options, None)?;
        decoder.run_decoder(&self.commands, None)?;
        Ok(decoder.finish(None)?.node())
    }

    fn encode<H: HugrView>(
        hugr: &H,
        options: EncodeOptions<H>,
    ) -> Result<Self, Self::EncodeError<H::Node>> {
        if !OpTag::DataflowParent.is_superset(hugr.entrypoint_tag()) {
            return Err(PytketEncodeError::NonDataflowRegion {
                region: hugr.entrypoint(),
                optype: hugr.entrypoint_optype().to_string(),
            });
        }

        let mut encoded = EncodedCircuit::new_standalone(hugr, options)?;

        let serial_circ = encoded
            .get_circuit_mut(hugr.entrypoint())
            .expect("Hugr entrypoint must be a dataflow region");
        Ok(std::mem::take(serial_circ))
    }
}

/// Load a TKET1 circuit from a JSON file.
///
/// See [DecodeOptions] for the options used by the decoder.
pub fn load_tk1_json_file(
    path: impl AsRef<Path>,
    options: DecodeOptions,
) -> Result<Hugr, PytketDecodeError> {
    let file = fs::File::open(path).map_err(PytketDecodeError::custom)?;
    let reader = io::BufReader::new(file);
    load_tk1_json_reader(reader, options)
}

/// Load a TKET1 circuit from a JSON reader.
///
/// See [DecodeOptions] for the options used by the decoder.
pub fn load_tk1_json_reader(
    json: impl io::Read,
    options: DecodeOptions,
) -> Result<Hugr, PytketDecodeError> {
    let ser: SerialCircuit = serde_json::from_reader(json).map_err(PytketDecodeError::custom)?;
    let circ: Hugr = ser.decode(options)?;
    Ok(circ)
}

/// Load a TKET1 circuit from a JSON string.
///
/// See [DecodeOptions] for the options used by the decoder.
pub fn load_tk1_json_str(json: &str, options: DecodeOptions) -> Result<Hugr, PytketDecodeError> {
    let reader = json.as_bytes();
    load_tk1_json_reader(reader, options)
}

/// Save a circuit to file in TK1 JSON format.
///
/// You may need to normalize the circuit using [`lower_to_pytket`] before saving.
///
/// See [EncodeOptions] for the options used by the encoder.
///
/// # Errors
///
/// Returns an error if the circuit is not flat or if it contains operations not
/// supported by pytket.
pub fn save_tk1_json_file<H: HugrView>(
    circ: &H,
    path: impl AsRef<Path>,
    options: EncodeOptions<H>,
) -> Result<(), PytketEncodeError<H::Node>> {
    let file = fs::File::create(path).map_err(PytketEncodeError::custom)?;
    let writer = io::BufWriter::new(file);
    save_tk1_json_writer(circ, writer, options)
}

/// Save a circuit in TK1 JSON format to a writer.
///
/// You may need to normalize the circuit using [`lower_to_pytket`] before saving.
///
/// See [EncodeOptions] for the options used by the encoder.
///
/// # Errors
///
/// Returns an error if the circuit is not flat or if it contains operations not
/// supported by pytket.
pub fn save_tk1_json_writer<H: HugrView>(
    circ: &H,
    w: impl io::Write,
    options: EncodeOptions<H>,
) -> Result<(), PytketEncodeError<H::Node>> {
    let serial_circ = SerialCircuit::encode(circ, options)?;
    serde_json::to_writer(w, &serial_circ).map_err(PytketEncodeError::custom)?;
    Ok(())
}

/// Save a circuit in TK1 JSON format to a String.
///
/// You may need to normalize the circuit using [`lower_to_pytket`] before saving.
///
/// See [EncodeOptions] for the options used by the encoder.
///
/// # Errors
///
/// Returns an error if the circuit is not flat or if it contains operations not
/// supported by pytket.
pub fn save_tk1_json_str<H: HugrView>(
    circ: &H,
    options: EncodeOptions<H>,
) -> Result<String, PytketEncodeError<H::Node>> {
    let mut buf = io::BufWriter::new(Vec::new());
    save_tk1_json_writer(circ, &mut buf, options)?;
    let bytes = buf.into_inner().unwrap();
    String::from_utf8(bytes).map_err(PytketEncodeError::custom)
}

/// A hashed register, used to identify registers in the [`Tk1Decoder::register_wire`] map,
/// avoiding string and vector clones on lookup.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
struct RegisterHash {
    hash: u64,
}

impl From<&ElementId> for RegisterHash {
    fn from(reg: &ElementId) -> Self {
        let mut hasher = DefaultHasher::new();
        reg.hash(&mut hasher);
        Self {
            hash: hasher.finish(),
        }
    }
}

impl From<&Qubit> for RegisterHash {
    fn from(reg: &Qubit) -> Self {
        let mut hasher = DefaultHasher::new();
        reg.hash(&mut hasher);
        Self {
            hash: hasher.finish(),
        }
    }
}

impl From<&Bit> for RegisterHash {
    fn from(reg: &Bit) -> Self {
        let mut hasher = DefaultHasher::new();
        reg.hash(&mut hasher);
        Self {
            hash: hasher.finish(),
        }
    }
}

/// A list of types we translate as pytket parameters.
static PARAMETER_TYPES: LazyLock<[Type; 2]> = LazyLock::new(|| [float64_type(), rotation_type()]);
