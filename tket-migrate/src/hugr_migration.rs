use crate::update_maps::{OpMapping, TypeMapping};
use hugr::HugrView;
use hugr::hugr::hugrmut::HugrMut;
use hugr::{
    Extension, Hugr,
    extension::{ExtensionRegistry, resolution::WeakExtensionRegistry},
    std_extensions::STD_REG,
};
use tket::passes::{ComposablePass, ReplaceTypes};

#[cfg(debug_assertions)]
const DEBUG: bool = false;

pub struct ExtensionUpdater {
    hugr: Hugr,
    op_mapping: OpMapping,
    type_mapping: TypeMapping,
    new_extensions: Vec<Extension>,
}

impl ExtensionUpdater {
    pub fn new(
        hugr: Hugr,
        op_mapping: OpMapping,
        type_mapping: TypeMapping,
        new_extensions: Vec<Extension>,
    ) -> Self {
        Self {
            hugr,
            op_mapping,
            type_mapping,
            new_extensions,
        }
    }

    pub fn migrate(&mut self) {
        self.add_new_extension();
        let mut replacer = ReplaceTypes::default();
        // NICOLA: TODO: we should not iterate over the nodes, but simply for each element in the update map add the entry in the replacer.
        // Now we need the node to get the signature when the replacement is to empty, we should be able to avid this (maybe by instantiating the versioned element that we need to replace).

        self.op_mapping
            .iter()
            .for_each(|(old_element, replacement)| {
                let old_op = old_element.get_instantiated_op(&self.hugr);
                replacer.set_replace_op(&old_op, replacement.get_op_replace(&old_op, &self.hugr));
            });

        self.type_mapping
            .iter()
            .for_each(|(old_type, replacement)| {
                let old_type = old_type.get_type(&self.hugr);
                replacer.set_replace_type(old_type, replacement);
            });

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
}
