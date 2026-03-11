//! Extension providing logical operations on Iceberg codeblocks.

use std::sync::{Arc, LazyLock, Weak};

use hugr::{
    Extension,
    extension::{
        CustomValidator, ExtensionId, OpDef, SignatureError, SignatureFunc, ValidateJustArgs,
        prelude::bool_t,
        simple_op::{
            HasConcrete, HasDef, MakeExtensionOp, MakeOpDef, MakeRegisteredOp, OpLoadError,
            try_from_name,
        },
    },
    ops::{ExtensionOp, OpName},
    std_extensions::{
        arithmetic::float_types::float64_type,
        collections::array::{Array, ArrayKind},
    },
    types::{FuncValueType, PolyFuncTypeRV, Type, TypeArg, type_param::TypeParam},
};
use strum::{EnumIter, EnumString, IntoStaticStr};

use super::types::{block_tv, get_usize};

/// The extension identifier.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.qec.iceberg.ops");
/// Extension version.
pub const VERSION: semver::Version = semver::Version::new(0, 1, 0);

/// Logical Iceberg operations.
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq, EnumIter, IntoStaticStr, EnumString)]
#[expect(non_camel_case_types)]
#[non_exhaustive]
pub enum IcebergOpDef {
    /// X gate.
    x,
    /// Z gate.
    z,
    /// X gate on two qubits.
    xx,
    /// Y gate on two qubits.
    yy,
    /// Z gate on two qubits.
    zz,
    /// X gate on all but one qubit.
    all_but_one_x,
    /// Z gate on all but one qubit.
    all_but_one_z,
    /// X gate on all qubits.
    all_x,
    /// Y gate on all qubits.
    all_y,
    /// Z gate on all qubits.
    all_z,
    /// X gate on one qubit with Z on all others.
    x_with_all_but_one_z,
    /// Z gate on one qubit with X on all others.
    z_with_all_but_one_x,
    /// Fan-out from one qubit to all others.
    fan_out,
    /// Fan-in to one qubut from all others.
    fan_in,
    /// Rx gate.
    rx,
    /// Rz gate.
    rz,
    /// Rx gate on all qubits.
    all_rx,
    /// Ry gate on all qubits.
    all_ry,
    /// Rz gate on all qubits.
    all_rz,
    /// Rx gate on all but one qubit.
    all_but_one_rx,
    /// Ry gate on all but one qubit.
    all_but_one_ry,
    /// Rz gate on all but one qubit.
    all_but_one_rz,
    /// H gate on all qubits.
    all_h,
    /// XXPhase gate.
    xx_phase,
    /// YYPhase gate.
    yy_phase,
    /// ZZPhase gate.
    zz_phase,
    /// CX gate.
    cx,
    /// Swap of two qubits within a block.
    swap,
    /// ZZPhase gate involving two blocks.
    zz_phase_between_blocks,
    /// CX gate applied transversally over two blocks.
    cx_transverse,
    /// Prepare the all-zero state on a block.
    alloc_zero,
    /// Syndrome measurement.
    measure_syndrome,
    /// Destructive measurement of all qubits.
    measure_all,
    /// Non-destructive measurement of one qubit in the X basis.
    measure_one_x,
    /// Non-destructive measurement of one qubit in the Z basis.
    measure_one_z,
}

/// Concrete Iceberg logical operation with block size and indices set.
pub struct ConcreteIcebergOp {
    /// The kind of operation.
    pub def: IcebergOpDef,

    /// The block size.
    pub k: usize,

    /// Qubit index parameters.
    pub indices: Vec<usize>,
}

impl HasConcrete for IcebergOpDef {
    type Concrete = ConcreteIcebergOp;

    fn instantiate(&self, type_args: &[TypeArg]) -> Result<Self::Concrete, OpLoadError> {
        let args: Vec<usize> = type_args
            .iter()
            .map(|a| get_usize(a).map_err(|_| SignatureError::InvalidTypeArgs))
            .collect::<Result<_, _>>()?;
        Ok(ConcreteIcebergOp {
            def: *self,
            k: args[0],
            indices: args[1..].to_vec(),
        })
    }
}

impl HasDef for ConcreteIcebergOp {
    type Def = IcebergOpDef;
}

impl MakeExtensionOp for ConcreteIcebergOp {
    fn op_id(&self) -> OpName {
        self.def.opdef_id()
    }

    fn from_extension_op(ext_op: &ExtensionOp) -> Result<Self, OpLoadError> {
        let def = IcebergOpDef::from_def(ext_op.def())?;
        def.instantiate(ext_op.args())
    }

    fn type_args(&self) -> Vec<TypeArg> {
        let mut args: Vec<TypeArg> = vec![(self.k as u64).into()];
        args.extend(self.indices.iter().map(|&i| (i as u64).into()));
        args
    }
}

impl MakeRegisteredOp for ConcreteIcebergOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID.clone()
    }

    fn extension_ref(&self) -> Arc<Extension> {
        EXTENSION.clone()
    }
}

impl IcebergOpDef {
    /// Initialize a [`ConcreteIcebergOp`] from an [`IcebergOpDef`] that
    /// requires a single qubit index.
    #[must_use]
    pub fn with_size_and_index(self, k: usize, i: usize) -> ConcreteIcebergOp {
        ConcreteIcebergOp {
            def: self,
            k,
            indices: vec![i],
        }
    }
}

/// Validator to check that the list of type arguments consists of a sequence
/// of natural numbers, the first of which (representing the block size) is
/// at least 2 and greater than all subsequent (representing qubit indices).
struct ArgsValidator {
    /// Expected number of index arguments following the initial block size.
    n_idx: usize,
}

impl ValidateJustArgs for ArgsValidator {
    fn validate(&self, arg_values: &[TypeArg]) -> Result<(), SignatureError> {
        let n = arg_values.len();
        if n != 1 + self.n_idx {
            return Err(SignatureError::InvalidTypeArgs);
        }
        let k = get_usize(&arg_values[0])?;
        if k == 0 || k % 2 == 1 {
            return Err(SignatureError::InvalidTypeArgs);
        }
        for arg in arg_values.iter().skip(1) {
            let i = get_usize(arg)?;
            if i >= k {
                return Err(SignatureError::InvalidTypeArgs);
            }
        }
        Ok(())
    }
}

/// Get an array-of-bool type with size corresponding to a type variable with a
/// given ID.
fn bool_array_tv(var_id: usize) -> Type {
    Array::ty_parametric(
        TypeArg::new_var_use(var_id, TypeParam::max_nat_type()),
        bool_t(),
    )
    .unwrap()
}

fn vec_of_blocks_and_angles(n_blocks: usize, n_angles: usize) -> Vec<Type> {
    let mut types: Vec<Type> = vec![block_tv(0); n_blocks];
    types.extend(vec![float64_type(); n_angles]);
    types
}

fn vec_of_blocks_and_bools(n_blocks: usize, n_bools: usize) -> Vec<Type> {
    let mut types: Vec<Type> = vec![block_tv(0); n_blocks];
    types.extend(vec![bool_t(); n_bools]);
    types
}

fn block_with_angles_sig(n_angles: usize) -> FuncValueType {
    FuncValueType::new(
        vec_of_blocks_and_angles(1, n_angles),
        vec_of_blocks_and_angles(1, 0),
    )
}

/// Signature of an operation that acts on a single block, with a number of
/// additional angle inputs and a number of index parameters.
fn sig_1_block(n_angles: usize, n_indices: usize) -> SignatureFunc {
    CustomValidator::new(
        PolyFuncTypeRV::new(
            vec![TypeParam::max_nat_type(); 1 + n_indices],
            block_with_angles_sig(n_angles),
        ),
        ArgsValidator { n_idx: n_indices },
    )
    .into()
}

impl MakeOpDef for IcebergOpDef {
    fn opdef_id(&self) -> OpName {
        <&Self as Into<&'static str>>::into(self).into()
    }

    fn from_def(op_def: &OpDef) -> Result<Self, OpLoadError> {
        try_from_name(op_def.name(), op_def.extension_id())
    }

    fn extension(&self) -> ExtensionId {
        EXTENSION_ID.clone()
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }

    fn init_signature(&self, _extension_ref: &Weak<Extension>) -> SignatureFunc {
        use IcebergOpDef::*;
        match self {
            x => sig_1_block(0, 1),
            z => sig_1_block(0, 1),
            xx => sig_1_block(0, 2),
            yy => sig_1_block(0, 2),
            zz => sig_1_block(0, 2),
            all_but_one_x => sig_1_block(0, 1),
            all_but_one_z => sig_1_block(0, 1),
            all_x => sig_1_block(0, 0),
            all_y => sig_1_block(0, 0),
            all_z => sig_1_block(0, 0),
            x_with_all_but_one_z => sig_1_block(0, 1),
            z_with_all_but_one_x => sig_1_block(0, 1),
            fan_out => sig_1_block(0, 1),
            fan_in => sig_1_block(0, 1),
            rx => sig_1_block(1, 1),
            rz => sig_1_block(1, 1),
            all_rx => sig_1_block(1, 0),
            all_ry => sig_1_block(1, 0),
            all_rz => sig_1_block(1, 0),
            all_but_one_rx => sig_1_block(1, 1),
            all_but_one_ry => sig_1_block(1, 1),
            all_but_one_rz => sig_1_block(1, 1),
            all_h => sig_1_block(0, 0),
            xx_phase => sig_1_block(1, 2),
            yy_phase => sig_1_block(1, 2),
            zz_phase => sig_1_block(1, 2),
            cx => sig_1_block(0, 2),
            swap => sig_1_block(0, 2),
            zz_phase_between_blocks => CustomValidator::new(
                PolyFuncTypeRV::new(
                    vec![TypeParam::max_nat_type(); 3],
                    FuncValueType::new(
                        vec_of_blocks_and_angles(2, 1),
                        vec_of_blocks_and_angles(2, 0),
                    ),
                ),
                ArgsValidator { n_idx: 2 },
            )
            .into(),
            cx_transverse => CustomValidator::new(
                PolyFuncTypeRV::new(
                    vec![TypeParam::max_nat_type()],
                    FuncValueType::new_endo(vec_of_blocks_and_angles(2, 0)),
                ),
                ArgsValidator { n_idx: 0 },
            )
            .into(),
            alloc_zero => CustomValidator::new(
                PolyFuncTypeRV::new(
                    vec![TypeParam::max_nat_type()],
                    FuncValueType::new(
                        vec_of_blocks_and_angles(0, 0),
                        vec_of_blocks_and_angles(1, 0),
                    ),
                ),
                ArgsValidator { n_idx: 0 },
            )
            .into(),
            measure_syndrome => CustomValidator::new(
                PolyFuncTypeRV::new(
                    vec![TypeParam::max_nat_type()],
                    FuncValueType::new(
                        vec_of_blocks_and_angles(1, 0),
                        vec_of_blocks_and_bools(1, 2),
                    ),
                ),
                ArgsValidator { n_idx: 0 },
            )
            .into(),
            measure_all => CustomValidator::new(
                PolyFuncTypeRV::new(
                    vec![TypeParam::max_nat_type()],
                    FuncValueType::new(vec_of_blocks_and_angles(1, 0), vec![bool_array_tv(0)]),
                ),
                ArgsValidator { n_idx: 0 },
            )
            .into(),
            measure_one_x => CustomValidator::new(
                PolyFuncTypeRV::new(
                    vec![TypeParam::max_nat_type(); 2],
                    FuncValueType::new(
                        vec_of_blocks_and_angles(1, 0),
                        vec_of_blocks_and_bools(1, 1),
                    ),
                ),
                ArgsValidator { n_idx: 1 },
            )
            .into(),
            measure_one_z => CustomValidator::new(
                PolyFuncTypeRV::new(
                    vec![TypeParam::max_nat_type(); 2],
                    FuncValueType::new(
                        vec_of_blocks_and_angles(1, 0),
                        vec_of_blocks_and_bools(1, 1),
                    ),
                ),
                ArgsValidator { n_idx: 1 },
            )
            .into(),
        }
    }

    fn description(&self) -> String {
        use IcebergOpDef::*;
        match self {
            x => "apply an X gate to one qubit",
            z => "apply a Z gate to one qubit",
            xx => "apply an X gate to two qubits",
            yy => "apply a Y gate to two qubits",
            zz => "apply a Z gate to two qubits",
            all_but_one_x => "apply an X gate to all but one qubit",
            all_but_one_z => "apply a Z gate to all but one qubit",
            all_x => "apply an X gate to all qubits",
            all_y => "apply a Y gate to all qubits",
            all_z => "apply a Z gate to all qubits",
            x_with_all_but_one_z => "apply an X gate to one qubit and a Z to the rest",
            z_with_all_but_one_x => "apply a Z gate to one qubit and an X to the rest",
            fan_out => "fan-out from one qubit to the rest",
            fan_in => "fan-in to one qubit from the rest",
            rx => "apply an Rx gate to one qubit",
            rz => "apply an Rz gate to one qubit",
            all_rx => "apply an Rx gate to all qubits",
            all_ry => "apply an Ry gate to all qubits",
            all_rz => "apply an Rz gate to all qubits",
            all_but_one_rx => "apply an Rx gate to all but one qubit",
            all_but_one_ry => "apply an Ry gate to all but one qubit",
            all_but_one_rz => "apply an Rz gate to all but one qubit",
            all_h => "apply an H gate to all qubits",
            xx_phase => "apply an XXPhase gate to two qubits within a block",
            yy_phase => "apply a YYPhase gate to two qubits within a block",
            zz_phase => "apply a ZZPhase gate to two qubits within a block",
            cx => "apply a CX gate to two qubits within a block",
            swap => "swap two qubits within a block",
            zz_phase_between_blocks => {
                "apply a ZZPhase gate to two qubits on different blocks of the same size"
            }
            cx_transverse => "apply a CX gate transversally over two blocks of the same size",
            alloc_zero => "allocate a block in the all-zero state",
            measure_syndrome => "perform a syndrome measurement, producing (X,Z) error indicators",
            measure_all => "destructively measure all qubits in the Z basis",
            measure_one_x => "non-destructively measure one qubit in the X basis",
            measure_one_z => "non-destructively measure one qubit in the Z basis",
        }
        .into()
    }
}

/// Extension for logical Iceberg operations.
pub static EXTENSION: LazyLock<Arc<Extension>> = LazyLock::new(|| {
    Extension::new_arc(EXTENSION_ID, VERSION, |extension, extension_ref| {
        IcebergOpDef::load_all_ops(extension, extension_ref).unwrap();
    })
});

#[cfg(test)]
mod tests {
    use hugr::{
        CircuitUnit, HugrView,
        builder::{
            DFGBuilder, Dataflow, DataflowHugr, DataflowSubContainer, HugrBuilder, ModuleBuilder,
        },
        envelope::{EnvelopeConfig, EnvelopeFormat, read_envelope, write_envelope},
        extension::ExtensionRegistry,
        ops::DataflowOpTrait,
        package::Package,
        std_extensions::{
            arithmetic::float_types::ConstF64, collections::array::array_type, std_reg,
        },
        types::Signature,
    };

    use crate::iceberg::types::EXTENSION as types_extension;
    use crate::iceberg::types::block_type;

    use super::*;

    #[test]
    fn test_iceberg_ops_extension() {
        assert_eq!(EXTENSION.name() as &str, "tket.qec.iceberg.ops");
        assert_eq!(EXTENSION.types().count(), 0);
        assert_eq!(EXTENSION.operations().count(), 35);
    }

    #[test]
    fn test_signatures() {
        assert_eq!(
            IcebergOpDef::x
                .with_size_and_index(6, 3)
                .to_extension_op()
                .unwrap()
                .signature()
                .as_ref(),
            &Signature::new(block_type(6), block_type(6))
        );
    }

    #[test]
    fn test_hugr_ops() {
        let block = block_type(6);
        let x3 = EXTENSION
            .instantiate_extension_op("x", [6.into(), 3.into()])
            .unwrap();
        let z4 = EXTENSION
            .instantiate_extension_op("z", [6.into(), 4.into()])
            .unwrap();
        let yy14 = EXTENSION
            .instantiate_extension_op("yy", [6.into(), 1.into(), 4.into()])
            .unwrap();
        let allx = EXTENSION
            .instantiate_extension_op("all_x", [6.into()])
            .unwrap();
        let allrz = EXTENSION
            .instantiate_extension_op("all_rz", [6.into()])
            .unwrap();
        let rz3 = EXTENSION
            .instantiate_extension_op("rz", [6.into(), 3.into()])
            .unwrap();
        let allbutonery1 = EXTENSION
            .instantiate_extension_op("all_but_one_ry", [6.into(), 1.into()])
            .unwrap();
        let allh = EXTENSION
            .instantiate_extension_op("all_h", [6.into()])
            .unwrap();
        let yyphase05 = EXTENSION
            .instantiate_extension_op("yy_phase", [6.into(), 0.into(), 5.into()])
            .unwrap();
        let zzphasebetweenblocks34 = EXTENSION
            .instantiate_extension_op("zz_phase_between_blocks", [6.into(), 3.into(), 4.into()])
            .unwrap();
        let cxtransverse = EXTENSION
            .instantiate_extension_op("cx_transverse", [6.into()])
            .unwrap();
        let cx23 = EXTENSION
            .instantiate_extension_op("cx", [6.into(), 2.into(), 3.into()])
            .unwrap();
        let swap51 = EXTENSION
            .instantiate_extension_op("swap", [6.into(), 5.into(), 1.into()])
            .unwrap();
        let mut module_builder = ModuleBuilder::new();
        let signature = Signature::new_endo(vec![block; 2]);
        let mut f_build = module_builder.define_function("main", signature).unwrap();
        let wires: Vec<_> = f_build.input_wires().collect();
        let mut linear = f_build.as_circuit(wires);
        linear.append(x3, [0]).unwrap();
        linear.append(z4, [0]).unwrap();
        linear.append(yy14, [0]).unwrap();
        linear.append(allx, [0]).unwrap();
        let angle = linear.add_constant(ConstF64::new(0.25));
        linear
            .append_and_consume(allrz, [CircuitUnit::Linear(0), CircuitUnit::Wire(angle)])
            .unwrap();
        linear
            .append_and_consume(rz3, [CircuitUnit::Linear(0), CircuitUnit::Wire(angle)])
            .unwrap();
        linear
            .append_and_consume(
                allbutonery1,
                [CircuitUnit::Linear(0), CircuitUnit::Wire(angle)],
            )
            .unwrap();
        linear.append(allh, [0]).unwrap();
        linear
            .append_and_consume(
                yyphase05,
                [CircuitUnit::Linear(0), CircuitUnit::Wire(angle)],
            )
            .unwrap();
        linear
            .append_and_consume(
                zzphasebetweenblocks34,
                [
                    CircuitUnit::Linear(0),
                    CircuitUnit::Linear(1),
                    CircuitUnit::Wire(angle),
                ],
            )
            .unwrap();
        linear.append(cxtransverse, [0, 1]).unwrap();
        linear.append(cx23, [1]).unwrap();
        linear.append(swap51, [0]).unwrap();
        let outs = linear.finish();
        f_build.finish_with_outputs(outs).unwrap();
        let h = module_builder.finish_hugr().unwrap();
        h.validate().unwrap();
    }

    #[test]
    fn test_alloc_measure() {
        let alloczero = EXTENSION
            .instantiate_extension_op("alloc_zero", [8.into()])
            .unwrap();
        let x3 = EXTENSION
            .instantiate_extension_op("x", [8.into(), 3.into()])
            .unwrap();
        let measuresyndrome = EXTENSION
            .instantiate_extension_op("measure_syndrome", [8.into()])
            .unwrap();
        let mut outputs: Vec<Type> = vec![block_type(8)];
        outputs.extend(vec![bool_t(); 2]);
        let mut dfg_builder = DFGBuilder::new(Signature::new(vec![], outputs)).unwrap();
        let handle = dfg_builder.add_dataflow_op(alloczero, vec![]).unwrap();
        let handle = dfg_builder.add_dataflow_op(x3, handle.outputs()).unwrap();
        let handle = dfg_builder
            .add_dataflow_op(measuresyndrome, handle.outputs())
            .unwrap();
        let h = dfg_builder
            .finish_hugr_with_outputs(handle.outputs())
            .unwrap();
        h.validate().unwrap();
    }

    #[test]
    fn test_measure_all() {
        let measureall = EXTENSION
            .instantiate_extension_op("measure_all", [4.into()])
            .unwrap();
        let mut dfg_builder =
            DFGBuilder::new(Signature::new(vec![block_type(4)], array_type(4, bool_t()))).unwrap();
        let handle = dfg_builder
            .add_dataflow_op(measureall, dfg_builder.input_wires())
            .unwrap();
        let h = dfg_builder
            .finish_hugr_with_outputs(handle.outputs())
            .unwrap();
        h.validate().unwrap();
    }

    #[test]
    fn test_measure_one() {
        let measureonez0 = EXTENSION
            .instantiate_extension_op("measure_one_z", [2.into(), 0.into()])
            .unwrap();
        let measureonez1 = EXTENSION
            .instantiate_extension_op("measure_one_z", [2.into(), 1.into()])
            .unwrap();
        let allh = EXTENSION
            .instantiate_extension_op("all_h", [2.into()])
            .unwrap();
        let mut dfg_builder = DFGBuilder::new(Signature::new(
            vec![block_type(2)],
            vec![block_type(2), bool_t(), bool_t()],
        ))
        .unwrap();
        let handle = dfg_builder
            .add_dataflow_op(allh, dfg_builder.input_wires())
            .unwrap();
        let handle = dfg_builder
            .add_dataflow_op(measureonez0, handle.outputs())
            .unwrap();
        let [block, c0] = handle.outputs_arr();
        let handle = dfg_builder
            .add_dataflow_op(measureonez1, vec![block])
            .unwrap();
        let [block, c1] = handle.outputs_arr();
        let h = dfg_builder
            .finish_hugr_with_outputs(vec![block, c0, c1])
            .unwrap();
        h.validate().unwrap();
    }

    #[test]
    fn test_serialization() {
        let block = block_type(6);
        let x3 = EXTENSION
            .instantiate_extension_op("x", [6.into(), 3.into()])
            .unwrap();
        let mut module_builder = ModuleBuilder::new();
        let signature = Signature::new_endo(vec![block]);
        let mut f_build = module_builder.define_function("main", signature).unwrap();
        let wires: Vec<_> = f_build.input_wires().collect();
        let mut linear = f_build.as_circuit(wires);
        linear.append(x3, [0]).unwrap();
        let outs = linear.finish();
        f_build.finish_with_outputs(outs).unwrap();
        let h = module_builder.finish_hugr().unwrap();
        let package = Package::new([h]);
        let mut bytes: Vec<u8> = Vec::new();
        write_envelope(
            &mut bytes,
            &package,
            EnvelopeConfig::new(EnvelopeFormat::ModelWithExtensions),
        )
        .unwrap();
        let buff = std::io::BufReader::new(bytes.as_slice());
        let mut reg: ExtensionRegistry = std_reg();
        reg.extend([types_extension.clone(), EXTENSION.clone()]);
        let (_, package1) = read_envelope(buff, &reg).unwrap();
        let h1 = &package1.modules[0];
        h1.validate().unwrap();
    }

    #[test]
    fn test_mismatched_k() {
        let block = block_type(6);
        let x3 = EXTENSION
            .instantiate_extension_op("x", [6.into(), 3.into()])
            .unwrap();
        let x3_bad_k = EXTENSION
            .instantiate_extension_op("x", [4.into(), 3.into()])
            .unwrap();
        let mut module_builder = ModuleBuilder::new();
        let signature = Signature::new_endo(vec![block]);
        let mut f_build = module_builder.define_function("main", signature).unwrap();
        let wires: Vec<_> = f_build.input_wires().collect();
        let mut linear = f_build.as_circuit(wires);
        linear.append(x3, [0]).unwrap();
        linear.append(x3_bad_k, [0]).unwrap();
        let outs = linear.finish();
        f_build.finish_with_outputs(outs).unwrap();
        assert!(module_builder.finish_hugr().is_err());
    }

    #[test]
    fn test_invalid_ops() {
        assert!(
            EXTENSION
                .instantiate_extension_op("x", [1.into(), 0.into()])
                .is_err()
        );
        assert!(
            EXTENSION
                .instantiate_extension_op("x", [6.into(), 6.into()])
                .is_err()
        );
        assert!(
            EXTENSION
                .instantiate_extension_op("xx", [6.into(), 0.into()])
                .is_err()
        );
    }
}
