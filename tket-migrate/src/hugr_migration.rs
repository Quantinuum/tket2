use crate::{
    default_maps::{get_measurement_migratation_op_map, get_measurement_migratation_type_map},
    update_maps::{OpReplacementTemplate, OpUpdateMap, TypeMapping, VersionedElement},
};
use hugr::types::Signature;
use hugr::{
    Extension, Hugr,
    builder::{DFGBuilder, Dataflow, DataflowHugr},
    extension::{ExtensionRegistry, resolution::WeakExtensionRegistry},
    std_extensions::{STD_REG, collections::array::ArrayOpBuilder},
};
use hugr::{HugrView, Node, ops::OpType};
use hugr::{hugr::hugrmut::HugrMut, ops::ExtensionOp};
use hugr::{
    hugr::patch::replace,
    ops::{DataflowOpTrait, OpTrait},
};
use tket::passes::{ComposablePass, ReplaceTypes, replace_types::NodeTemplate};

#[cfg(debug_assertions)]
const DEBUG: bool = false;

pub struct ExtensionUpdater {
    hugr: Hugr,
    op_update_map: OpUpdateMap,
    type_mapping: TypeMapping,
    new_extensions: Vec<Extension>,
}

impl ExtensionUpdater {
    pub fn new(
        hugr: Hugr,
        op_update_map: OpUpdateMap,
        type_mapping: TypeMapping,
        new_extensions: Vec<Extension>,
    ) -> Self {
        Self {
            hugr,
            op_update_map,
            type_mapping,
            new_extensions,
        }
    }

    pub fn migrate(&mut self) {
        self.add_new_extension();
        let mut replacer = ReplaceTypes::default();
        // NICOLA: TODO: we should not iterate over the nodes, but simply for each element in the update map add the entry in the replacer.
        // Now we need the node to get the signature when the replacement is to empty, we should be able to avid this (maybe by instantiating the versioned element that we need to replace).
        for node in self.hugr.nodes().collect::<Vec<_>>() {
            match self.hugr.get_optype(node) {
                OpType::ExtensionOp(op) => self.set_extension_op_replace(node, op, &mut replacer),
                _ => {}
            }
        }

        replacer.run(&mut self.hugr);
    }

    fn add_new_extension(&mut self) {
        let mut extensions = STD_REG.to_owned();
        extensions.extend(self.hugr.extensions().clone());
        let new_ext_registry = ExtensionRegistry::new_with_extension_resolution(
            self.new_extensions.clone(),
            &WeakExtensionRegistry::from(&extensions),
        )
        .unwrap();
        extensions.extend(new_ext_registry);
        self.hugr.use_extensions(extensions);
        if DEBUG {
            std::fs::write(
                "updated_extension_registry.json",
                serde_json::to_string_pretty(&self.hugr.extensions()).unwrap(),
            )
            .unwrap();
            println!("saved Hugr extensions");
        }
    }

    fn set_extension_op_replace(
        &self,
        old_node: Node,
        operation: &ExtensionOp,
        replacer: &mut ReplaceTypes,
    ) {
        let Some(replacement_template) = self.op_update_map.get_replacement(operation) else {
            return;
        };
        let node_template = match replacement_template {
            OpReplacementTemplate::TemplateInstance(template) => template.clone(),
            OpReplacementTemplate::Empty => self.get_node_template(old_node, &[]),
            OpReplacementTemplate::VersionedElements(v) => self.get_node_template(old_node, v),
        };
        replacer.set_replace_op(operation, node_template);
    }

    fn get_node_template(
        &self,
        old_node: Node,
        versioned_elements: &[VersionedElement],
    ) -> NodeTemplate {
        let operations = versioned_elements
            .iter()
            .map(|element| element.get_instantiated(&self.hugr))
            .collect::<Vec<_>>();

        let signature = match (operations.first(), operations.last()) {
            (Some(first), Some(last)) => Signature::new(
                first.signature().input().clone(),
                last.signature().output().clone(),
            ),
            _ => self
                .hugr
                .get_optype(old_node)
                .dataflow_signature()
                .expect("An empty replacement requires a dataflow operation")
                .into_owned(),
        };
        let mut builder = DFGBuilder::new(signature).expect("Failed to build replacement");
        let mut wires = builder.input_wires().collect::<Vec<_>>();
        for operation in operations {
            // NICOLA: TODO: we should have a proper error here
            wires = builder
                .add_dataflow_op(operation, wires)
                .expect("Replacement operations have incompatible signatures")
                .outputs()
                .collect();
        }
        // NICOLA: TODO: we should have a proper error here
        NodeTemplate::linked_hugr(builder.finish_hugr_with_outputs(wires).unwrap())
    }
}
