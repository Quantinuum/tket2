//! Shared mutable storage with explicitly managed linear cell handles.
//!
//! A cell can hold a linear value. Use [`CellOpDef::Dup`] to share a cell and
//! [`CellOpDef::Free`] to release each handle, recovering the value when the last
//! handle is released. [`CellOpDef::Swap`] and [`CellOpDef::Map`] update the cell
//! without copying or discarding its contents. [`CellOpDef::Read`] and
//! [`CellOpDef::Write`] require a copyable payload.
//!
//! `Map` gives its callback exclusive access to the payload. The callback returns
//! the replacement payload followed by its results. The operation installs the
//! replacement and returns the handle followed by those results. Backends that
//! execute aliases concurrently must synchronize access to the shared storage.
//!
//! Passing the returned handle to the next operation orders their execution.
//! After [`CellOpDef::Dup`], operations on separate handles have unspecified
//! order unless another dependency orders them.

use std::sync::{Arc, LazyLock, Weak};

use strum::{EnumIter, EnumString, IntoStaticStr};

use hugr_core::Wire;
use hugr_core::builder::{BuildError, Dataflow};
use hugr_core::extension::TypeDefBound;
use hugr_core::extension::prelude::{bool_t, option_type};
use hugr_core::ops::OpName;
use hugr_core::types::{
    CustomType, FuncValueType, PolyFuncType, PolyFuncTypeRV, Signature, Type, TypeBound, TypeName,
    TypeRow, TypeRowRV,
};
use hugr_core::{
    Extension,
    extension::{
        ExtensionId, OpDef, SignatureError, SignatureFunc,
        simple_op::{
            HasConcrete, HasDef, MakeExtensionOp, MakeOpDef, MakeRegisteredOp, OpLoadError,
        },
    },
    ops::custom::ExtensionOp,
    types::type_param::{TypeArg, TypeParam},
};
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq, EnumIter, IntoStaticStr, EnumString)]
#[non_exhaustive]
/// Cell operation definitions.
pub enum CellOpDef {
    /// Create a new cell.
    New,
    /// Copy the stored value and return the cell. Requires a copyable value.
    Read,
    /// Replace a copyable stored value and return the cell.
    Write,
    /// Exchange the stored value with the input and return the cell and old value.
    Swap,
    /// Return two handles to the same cell, without copying its contents.
    Dup,
    /// Compare cell identity and return both handles in input order.
    /// Does not read or compare the stored values.
    Eq,
    /// Release a handle, returning the stored value only for the last handle.
    Free,
    /// Apply a function to the stored value and extra inputs, replacing the value.
    Map,
}

impl CellOpDef {
    /// Create a concrete cell operation with the given value type.
    /// For [`Self::Map`], the extra input and output rows are empty.
    /// Use [`CellOp::map`] to supply nonempty rows.
    #[must_use]
    pub fn with_type(self, ty: Type) -> CellOp {
        CellOp::new(self, ty)
    }
}

impl MakeOpDef for CellOpDef {
    fn opdef_id(&self) -> OpName {
        <&'static str>::from(self).into()
    }

    fn from_def(op_def: &OpDef) -> Result<Self, OpLoadError>
    where
        Self: Sized,
    {
        hugr_core::extension::simple_op::try_from_name(op_def.name(), op_def.extension_id())
    }

    fn init_signature(&self, extension_ref: &Weak<Extension>) -> SignatureFunc {
        let bound = match self {
            Self::Read | Self::Write => TypeBound::Copyable,
            _ => TypeBound::Linear,
        };
        let param = TypeParam::TypeKind(bound);
        let elem = Type::new_var_use(0, bound);
        let cell: Type = cell_custom_type(elem.clone(), extension_ref).into();
        let signature = match self {
            Self::New => Signature::new([elem], [cell]),
            Self::Read => Signature::new([cell.clone()], [cell, elem]),
            Self::Write => Signature::new([cell.clone(), elem], [cell]),
            Self::Swap => Signature::new([cell.clone(), elem.clone()], [cell, elem]),
            Self::Dup => Signature::new([cell.clone()], [cell.clone(), cell]),
            Self::Eq => {
                Signature::new([cell.clone(), cell.clone()], [cell.clone(), cell, bool_t()])
            }
            Self::Free => Signature::new([cell], [option_type([elem]).into()]),
            Self::Map => {
                let row_param =
                    TypeParam::ListKind(Box::new(TypeParam::TypeKind(TypeBound::Linear)));
                let params = [param, row_param.clone(), row_param];
                let [input_row, output_row] =
                    [1, 2].map(|i| TypeRowRV::new_var_use(i, TypeBound::Linear));
                let func_t = Type::new_function(FuncValueType::new(
                    TypeRowRV::from(vec![elem.clone()]).concat(input_row.clone()),
                    TypeRowRV::from(vec![elem]).concat(output_row.clone()),
                ));
                return PolyFuncTypeRV::new(
                    params,
                    FuncValueType::new(
                        TypeRowRV::from(vec![cell.clone(), func_t]).concat(input_row),
                        TypeRowRV::from(vec![cell]).concat(output_row),
                    ),
                )
                .into();
            }
        };
        PolyFuncType::new([param], signature).into()
    }

    fn extension(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }

    fn description(&self) -> String {
        match self {
            CellOpDef::New => "Create a new cell from a value.".into(),
            CellOpDef::Read => "Copy the stored value and return the cell.".into(),
            CellOpDef::Write => "Replace a copyable stored value and return the cell.".into(),
            CellOpDef::Swap => "Exchange the stored value, returning the cell and old value.".into(),
            CellOpDef::Dup => "Return two handles to the same cell without copying the stored value.".into(),
            CellOpDef::Eq => "Compare cell identity without reading the stored values, returning both handles in input order and a boolean.".into(),
            CellOpDef::Free => "Release a handle, returning Some(value) for the last handle and None otherwise.".into(),
            CellOpDef::Map => "Apply a function to the stored value and extra inputs, replacing the stored value and returning the cell and extra outputs.".into(),
        }
    }
}

/// Name of cell extension.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.cell");
/// Name of cell type.
pub const CELL_TYPE_ID: TypeName = TypeName::new_inline("cell");
const TYPE_PARAMS: [TypeParam; 1] = [TypeParam::TypeKind(TypeBound::Linear)];
/// Extension version.
pub const VERSION: hugr_core::extension::Version = hugr_core::extension::Version::new(0, 1, 0);

/// Extension for cell operations.
fn extension() -> Arc<Extension> {
    Extension::new_arc(EXTENSION_ID, VERSION, |extension, extension_ref| {
        extension
            .add_type(
                CELL_TYPE_ID,
                TYPE_PARAMS.into(),
                "Linear handle to a shared mutable cell.".into(),
                TypeDefBound::Explicit {
                    bound: TypeBound::Linear,
                },
                extension_ref,
            )
            .unwrap();
        CellOpDef::load_all_ops(extension, extension_ref).unwrap();
    })
}

/// Reference to the cell extension.
pub static EXTENSION: LazyLock<Arc<Extension>> = LazyLock::new(extension);

/// Construct a cell type without initializing the extension recursively.
fn cell_custom_type(ty: impl Into<Type>, extension_ref: &Weak<Extension>) -> CustomType {
    let ty = ty.into();
    CustomType::new(
        CELL_TYPE_ID,
        [ty.into()],
        EXTENSION_ID,
        VERSION,
        TypeBound::Linear,
        extension_ref,
    )
}

/// A linear handle to a shared mutable cell containing the given type.
pub fn cell_type(ty: impl Into<Type>) -> Type {
    cell_custom_type(ty, &Arc::<Extension>::downgrade(&EXTENSION)).into()
}

#[derive(Clone, Debug, PartialEq)]
/// A concrete cell operation.
pub struct CellOp {
    /// The operation definition.
    pub def: CellOpDef,
    /// Type of the value stored in the cell.
    pub ty: Type,
    /// Extra input and output rows for [`CellOpDef::Map`].
    /// Other operations use empty rows.
    pub map_signature: Signature,
}

impl CellOp {
    /// Create a map operation with the given extra input and output rows.
    /// The callback also takes and returns the stored value as its first value.
    pub fn map(ty: Type, inputs: impl Into<TypeRow>, outputs: impl Into<TypeRow>) -> Self {
        Self {
            def: CellOpDef::Map,
            ty,
            map_signature: Signature::new(inputs, outputs),
        }
    }

    fn new(op: CellOpDef, ty: Type) -> Self {
        Self {
            def: op,
            ty,
            map_signature: Signature::new_endo(vec![]),
        }
    }
}

impl MakeExtensionOp for CellOp {
    fn op_id(&self) -> OpName {
        self.def.opdef_id()
    }

    fn from_extension_op(ext_op: &ExtensionOp) -> Result<Self, OpLoadError> {
        let def = CellOpDef::from_def(ext_op.def())?;
        def.instantiate(ext_op.args())
    }

    fn type_args(&self) -> Vec<TypeArg> {
        let mut args = vec![self.ty.clone().into()];
        if self.def == CellOpDef::Map {
            args.extend([
                self.map_signature.input().clone().into(),
                self.map_signature.output().clone().into(),
            ]);
        }
        args
    }
}

impl MakeRegisteredOp for CellOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID.clone()
    }

    fn extension_ref(&self) -> Arc<Extension> {
        EXTENSION.clone()
    }
}

/// An extension trait for [Dataflow] providing methods to add cell
/// operations.
pub trait CellOpBuilder: Dataflow {
    /// Add a `tket.cell.New` op.
    fn add_new_cell(&mut self, val_wire: Wire) -> Result<Wire, BuildError> {
        let ty = self.get_wire_type(val_wire)?;
        let handle = self.add_dataflow_op(CellOpDef::New.with_type(ty), [val_wire])?;

        Ok(handle.out_wire(0))
    }

    /// Copy a stored value, returning the cell and the copied value.
    fn add_read_cell(&mut self, cell_wire: Wire, ty: Type) -> Result<(Wire, Wire), BuildError> {
        let op = CellOpDef::Read.with_type(ty).to_extension_op()?;
        let handle = self.add_dataflow_op(op, [cell_wire])?;
        Ok((handle.out_wire(0), handle.out_wire(1)))
    }

    /// Replace a copyable stored value, returning the cell.
    fn add_write_cell(&mut self, cell_wire: Wire, val_wire: Wire) -> Result<Wire, BuildError> {
        let ty = self.get_wire_type(val_wire)?;

        let op = CellOpDef::Write.with_type(ty).to_extension_op()?;
        let handle = self.add_dataflow_op(op, [cell_wire, val_wire])?;
        Ok(handle.out_wire(0))
    }

    /// Exchange the stored value, returning the cell and the previous value.
    fn add_swap_cell(
        &mut self,
        cell_wire: Wire,
        val_wire: Wire,
    ) -> Result<(Wire, Wire), BuildError> {
        let ty = self.get_wire_type(val_wire)?;
        let handle = self.add_dataflow_op(CellOpDef::Swap.with_type(ty), [cell_wire, val_wire])?;
        Ok((handle.out_wire(0), handle.out_wire(1)))
    }

    /// Return two handles to the same cell containing values of type `ty`.
    fn add_dup_cell(&mut self, cell_wire: Wire, ty: Type) -> Result<(Wire, Wire), BuildError> {
        let handle = self.add_dataflow_op(CellOpDef::Dup.with_type(ty), [cell_wire])?;
        Ok((handle.out_wire(0), handle.out_wire(1)))
    }

    /// Compare the identity of two cells with the same stored type `ty`.
    /// Return both linear handles in input order, followed by the comparison result.
    /// The stored values are neither read nor compared.
    fn add_eq_cell(
        &mut self,
        lhs: Wire,
        rhs: Wire,
        ty: Type,
    ) -> Result<(Wire, Wire, Wire), BuildError> {
        let handle = self.add_dataflow_op(CellOpDef::Eq.with_type(ty), [lhs, rhs])?;
        Ok((handle.out_wire(0), handle.out_wire(1), handle.out_wire(2)))
    }

    /// Release a handle to a cell containing values of type `ty`.
    /// The returned wire has type `option<ty>` and contains the stored value only
    /// when this was the last handle.
    fn add_free_cell(&mut self, cell_wire: Wire, ty: Type) -> Result<Wire, BuildError> {
        let handle = self.add_dataflow_op(CellOpDef::Free.with_type(ty), [cell_wire])?;
        Ok(handle.out_wire(0))
    }

    /// Apply `func_wire` to the stored value and the extra `inputs`.
    ///
    /// The callback takes and returns the stored type `ty` first. Its remaining
    /// input types are inferred from `inputs`, and its remaining output types
    /// are given by `output_types`. Return the cell and the extra outputs in
    /// callback order. Both extra rows may be empty or contain linear values.
    fn add_map_cell(
        &mut self,
        cell_wire: Wire,
        func_wire: Wire,
        ty: Type,
        inputs: impl IntoIterator<Item = Wire>,
        output_types: impl Into<TypeRow>,
    ) -> Result<(Wire, Vec<Wire>), BuildError> {
        let inputs = inputs.into_iter().collect::<Vec<_>>();
        let input_types = inputs
            .iter()
            .map(|&wire| self.get_wire_type(wire))
            .collect::<Result<Vec<_>, _>>()?;
        let op = CellOp::map(ty, input_types, output_types);
        let handle = self.add_dataflow_op(op, [cell_wire, func_wire].into_iter().chain(inputs))?;
        Ok((handle.out_wire(0), handle.outputs().skip(1).collect()))
    }
}

impl<D: Dataflow> CellOpBuilder for D {}

impl HasConcrete for CellOpDef {
    type Concrete = CellOp;

    fn instantiate(&self, type_args: &[TypeArg]) -> Result<Self::Concrete, OpLoadError> {
        let op = match (self, type_args) {
            (Self::Map, [ty, inputs, outputs]) => CellOp::map(
                Type::try_from(ty.clone())?,
                TypeRow::try_from(inputs.clone())?,
                TypeRow::try_from(outputs.clone())?,
            ),
            (def, [ty]) if *def != Self::Map => def.with_type(Type::try_from(ty.clone())?),
            _ => return Err(SignatureError::InvalidTypeArgs.into()),
        };
        op.clone().to_extension_op()?;
        Ok(op)
    }
}

impl HasDef for CellOp {
    type Def = CellOpDef;
}

#[cfg(test)]
pub(crate) mod test {
    use cool_asserts::assert_matches;
    use hugr_core::builder::DFGBuilder;
    use hugr_core::extension::prelude::{bool_t, qb_t};
    use hugr_core::ops::ExtensionOp;
    use hugr_core::{HugrView, PortIndex};
    use hugr_core::{
        builder::{Dataflow, DataflowHugr},
        std_extensions::arithmetic::int_types::INT_TYPES,
    };
    use std::sync::Arc;
    use strum::IntoEnumIterator;

    use super::*;
    use hugr_core::std_extensions::arithmetic::float_types::float64_type;
    fn get_opdef(op: impl Into<&'static str>) -> Option<&'static Arc<OpDef>> {
        EXTENSION.get_op(op.into())
    }

    #[test]
    fn create_extension() {
        assert_eq!(EXTENSION.name(), &EXTENSION_ID);

        for o in CellOpDef::iter() {
            assert_eq!(CellOpDef::from_def(get_opdef(o).unwrap()), Ok(o));
        }
    }

    #[test]
    fn test_ops() {
        let ops = [
            CellOp::new(CellOpDef::New, bool_t().clone()),
            CellOp::new(CellOpDef::Read, float64_type()),
            CellOp::new(CellOpDef::Write, INT_TYPES[5].clone()),
        ];
        for op in ops {
            let op_t: ExtensionOp = op.clone().to_extension_op().unwrap();
            let def_op = CellOpDef::from_op(&op_t).unwrap();
            assert_eq!(op.def, def_op);
            let new_op = CellOp::from_op(&op_t).unwrap();
            assert_eq!(new_op, op);
        }
    }

    #[test]
    fn test_build() {
        let in_row = vec![bool_t(), float64_type()];

        let hugr = {
            let mut builder = DFGBuilder::new(Signature::new(in_row.clone(), vec![])).unwrap();

            let in_wires: [Wire; 2] = builder.input_wires_arr();
            for (ty, w) in in_row.into_iter().zip(in_wires.iter()) {
                let new_cell = builder.add_new_cell(*w).unwrap();
                let (cell, read) = builder.add_read_cell(new_cell, ty.clone()).unwrap();
                let cell = builder.add_write_cell(cell, read).unwrap();
                builder
                    .add_dataflow_op(CellOpDef::Free.with_type(ty), [cell])
                    .unwrap();
            }

            builder.finish_hugr_with_outputs([]).unwrap()
        };
        assert_matches!(hugr.validate(), Ok(()));
    }

    #[test]
    fn ptr_and_cell_reads_load_from_the_tket_registry() {
        use hugr_core::envelope::EnvelopeConfig;
        use hugr_core::std_extensions::ptr::{PtrOpBuilder, ptr_type};

        let sig = Signature::new(
            [ptr_type(bool_t()), cell_type(bool_t())],
            [bool_t(), cell_type(bool_t()), bool_t()],
        );
        let mut builder = DFGBuilder::new(sig).unwrap();
        let [ptr, cell] = builder.input_wires_arr();
        let ptr_value = builder.add_read_ptr(ptr, bool_t()).unwrap();
        let (cell, cell_value) = builder.add_read_cell(cell, bool_t()).unwrap();
        let hugr = builder
            .finish_hugr_with_outputs([ptr_value, cell, cell_value])
            .unwrap();
        let serialized = hugr.store_str(EnvelopeConfig::text()).unwrap();
        hugr_core::Hugr::load_str(serialized, Some(&crate::extension::REGISTRY))
            .unwrap()
            .validate()
            .unwrap();
    }

    #[test]
    fn read_copyable_type_variable() {
        use hugr_core::builder::FunctionBuilder;
        let bound = TypeBound::Copyable;
        let ty = Type::new_var_use(0, bound);
        let cell = cell_type(ty.clone());
        let sig = Signature::new([cell.clone()], [cell, ty.clone()]);
        let mut builder =
            FunctionBuilder::new("read", PolyFuncType::new([TypeParam::TypeKind(bound)], sig))
                .unwrap();
        let [cell] = builder.input_wires_arr();
        let (cell, value) = builder.add_read_cell(cell, ty).unwrap();
        builder.finish_hugr_with_outputs([cell, value]).unwrap();
    }

    #[test]
    fn read_rejects_linear_payload() {
        let cell = cell_type(qb_t());
        let mut builder = DFGBuilder::new(Signature::new_endo([cell])).unwrap();
        let [cell] = builder.input_wires_arr();
        assert!(builder.add_read_cell(cell, qb_t()).is_err());
        builder.finish_hugr_with_outputs([cell]).unwrap();
    }

    #[test]
    fn write_rejects_linear_payload() {
        let sig = Signature::new_endo([cell_type(qb_t()), qb_t()]);
        let mut builder = DFGBuilder::new(sig).unwrap();
        let [cell, value] = builder.input_wires_arr();
        assert!(builder.add_write_cell(cell, value).is_err());
        builder.finish_hugr_with_outputs([cell, value]).unwrap();
    }

    #[test]
    fn signatures_and_bounds() {
        use hugr_core::ops::DataflowOpTrait;
        for ty in [bool_t(), qb_t()] {
            let cell = cell_type(ty.clone());
            assert_eq!(cell.least_upper_bound(), TypeBound::Linear);
            let declared: Type = EXTENSION
                .get_type(&CELL_TYPE_ID)
                .unwrap()
                .instantiate(vec![ty.clone().into()])
                .unwrap()
                .into();
            assert_eq!(cell, declared);
            let cases = [
                (CellOpDef::New, vec![ty.clone()], vec![cell.clone()]),
                (
                    CellOpDef::Eq,
                    vec![cell.clone(), cell.clone()],
                    vec![cell.clone(), cell.clone(), bool_t()],
                ),
                (
                    CellOpDef::Swap,
                    vec![cell.clone(), ty.clone()],
                    vec![cell.clone(), ty.clone()],
                ),
                (
                    CellOpDef::Dup,
                    vec![cell.clone()],
                    vec![cell.clone(), cell.clone()],
                ),
                (
                    CellOpDef::Free,
                    vec![cell.clone()],
                    vec![option_type([ty.clone()]).into()],
                ),
            ];
            for (def, inputs, outputs) in cases {
                let op = def.with_type(ty.clone());
                let ext = op.clone().to_extension_op().unwrap();
                assert_eq!(
                    ext.signature().into_owned(),
                    Signature::new(inputs, outputs)
                );
                assert_eq!(CellOp::from_op(&ext).unwrap(), op);
            }
            for (def, inputs, outputs) in [
                (
                    CellOpDef::Read,
                    vec![cell.clone()],
                    vec![cell.clone(), ty.clone()],
                ),
                (
                    CellOpDef::Write,
                    vec![cell.clone(), ty.clone()],
                    vec![cell.clone()],
                ),
            ] {
                let result = def.with_type(ty.clone()).to_extension_op();
                if ty == qb_t() {
                    assert!(result.is_err());
                    assert!(def.instantiate(&[ty.clone().into()]).is_err());
                } else {
                    assert_eq!(
                        result.unwrap().signature().into_owned(),
                        Signature::new(inputs, outputs)
                    );
                }
            }
        }
    }

    #[test]
    fn map_rows_and_roundtrip() {
        use hugr_core::ops::DataflowOpTrait;
        for (inputs, outputs) in [
            (vec![], vec![]),
            (vec![bool_t(), qb_t()], vec![qb_t()]),
            (vec![], vec![bool_t(), qb_t()]),
            (vec![qb_t()], vec![]),
        ] {
            let ty = qb_t();
            let op = CellOp::map(ty.clone(), inputs.clone(), outputs.clone());
            let mut callback_inputs = vec![ty.clone()];
            callback_inputs.extend(inputs.clone());
            let mut callback_outputs = vec![ty.clone()];
            callback_outputs.extend(outputs.clone());
            let callback =
                Type::new_function(FuncValueType::new(callback_inputs, callback_outputs));
            let mut expected_inputs = vec![cell_type(ty.clone()), callback];
            expected_inputs.extend(inputs);
            let mut expected_outputs = vec![cell_type(ty)];
            expected_outputs.extend(outputs.clone());
            let expected = Signature::new(expected_inputs, expected_outputs);
            let ext = op.clone().to_extension_op().unwrap();
            assert_eq!(ext.signature().into_owned(), expected);
            assert_eq!(CellOp::from_op(&ext).unwrap(), op);
            let mut builder = DFGBuilder::new(expected).unwrap();
            let wires = builder.input_wires().collect::<Vec<_>>();
            let (cell, results) = builder
                .add_map_cell(
                    wires[0],
                    wires[1],
                    qb_t(),
                    wires[2..].iter().copied(),
                    outputs,
                )
                .unwrap();
            assert_eq!(builder.get_wire_type(cell).unwrap(), cell_type(qb_t()));
            let outputs = std::iter::once(cell).chain(results);
            builder
                .finish_hugr_with_outputs(outputs)
                .unwrap()
                .validate()
                .unwrap();
        }
        assert!(CellOpDef::Map.instantiate(&[qb_t().into()]).is_err());
        assert!(
            CellOpDef::Map
                .instantiate(&[
                    qb_t().into(),
                    bool_t().into(),
                    TypeArg::new_list::<Type>([]),
                ])
                .is_err()
        );
    }

    #[test]
    fn linear_cell_lifecycle() {
        let option: Type = option_type([qb_t()]).into();
        let mut builder = DFGBuilder::new(Signature::new(
            [qb_t(), qb_t()],
            [option.clone(), option, qb_t()],
        ))
        .unwrap();
        let [value, replacement] = builder.input_wires_arr();
        let cell = builder.add_new_cell(value).unwrap();
        let (cell, other) = builder.add_dup_cell(cell, qb_t()).unwrap();
        let (cell, old_value) = builder.add_swap_cell(cell, replacement).unwrap();
        let first = builder.add_free_cell(cell, qb_t()).unwrap();
        let last = builder.add_free_cell(other, qb_t()).unwrap();
        builder
            .finish_hugr_with_outputs([first, last, old_value])
            .unwrap()
            .validate()
            .unwrap();
    }

    #[test]
    fn eq_preserves_linear_handles() {
        let cell = cell_type(qb_t());
        let signature = Signature::new(
            [cell.clone(), cell.clone()],
            [cell.clone(), cell.clone(), bool_t()],
        );
        let mut builder = DFGBuilder::new(signature).unwrap();
        let [lhs, rhs] = builder.input_wires_arr();
        let (lhs, rhs, equal) = builder.add_eq_cell(lhs, rhs, qb_t()).unwrap();
        assert_eq!(lhs.source().index(), 0);
        assert_eq!(rhs.source().index(), 1);
        assert_eq!(equal.source().index(), 2);
        builder
            .finish_hugr_with_outputs([lhs, rhs, equal])
            .unwrap()
            .validate()
            .unwrap();

        for copy in [false, true] {
            let outputs = if copy {
                vec![cell.clone(), cell.clone(), cell.clone(), bool_t()]
            } else {
                vec![cell.clone(), bool_t()]
            };
            let mut builder =
                DFGBuilder::new(Signature::new([cell.clone(), cell.clone()], outputs)).unwrap();
            let [lhs, rhs] = builder.input_wires_arr();
            let (lhs, rhs, equal) = builder.add_eq_cell(lhs, rhs, qb_t()).unwrap();
            let outputs = if copy {
                vec![lhs, rhs, rhs, equal]
            } else {
                vec![lhs, equal]
            };
            assert!(builder.finish_hugr_with_outputs(outputs).is_err());
        }
        assert!(CellOpDef::Eq.instantiate(&[]).is_err());
        assert!(
            CellOpDef::Eq
                .instantiate(&[qb_t().into(), bool_t().into()])
                .is_err()
        );
        let mut builder = DFGBuilder::new(Signature::new(
            [cell_type(qb_t()), cell_type(bool_t())],
            [cell_type(qb_t()), cell_type(qb_t()), bool_t()],
        ))
        .unwrap();
        let [lhs, rhs] = builder.input_wires_arr();
        let (lhs, rhs, equal) = builder.add_eq_cell(lhs, rhs, qb_t()).unwrap();
        assert!(builder.finish_hugr_with_outputs([lhs, rhs, equal]).is_err());
    }

    #[test]
    fn implicit_cell_copy_or_drop_is_rejected() {
        let cell = cell_type(bool_t());
        for outputs in [vec![], vec![cell.clone(), cell.clone()]] {
            let builder = DFGBuilder::new(Signature::new([cell.clone()], outputs.clone())).unwrap();
            let [input] = builder.input_wires_arr();
            assert!(
                builder
                    .finish_hugr_with_outputs(vec![input; outputs.len()])
                    .is_err()
            );
        }
    }
}
