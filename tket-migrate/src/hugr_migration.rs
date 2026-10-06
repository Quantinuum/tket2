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

    pub fn get_hugr(&self) -> &Hugr {
        &self.hugr
    }

    pub fn migrate(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.add_new_extension()?;
        let mut replacer = ReplaceTypes::default();

        for (old_type, replacement) in self.type_mapping.iter() {
            let Some(old_type) = old_type.get_type(&self.hugr)? else {
                continue;
            };
            replacer.set_replace_type(old_type, replacement.get_type(&self.hugr)?);
        }

        for (old_element, replacement) in self.op_mapping.iter() {
            let Some(old_op) = old_element.get_instantiated_op(&self.hugr)? else {
                continue;
            };
            let template = replacement.get_op_replace(&old_op, &self.hugr, &replacer)?;
            replacer.set_replace_op(&old_op, template);
        }

        replacer.run(&mut self.hugr)?;

        // remove unused extensions
        // let registry = self.hugr.extensions().clone();
        // self.hugr.resolve_extension_defs(&registry)?;
        Ok(())
    }

    fn add_new_extension(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let mut extensions = STD_REG.to_owned();
        extensions.extend(self.hugr.extensions().clone());
        let new_ext_registry = ExtensionRegistry::new_with_extension_resolution(
            std::mem::take(&mut self.new_extensions),
            &WeakExtensionRegistry::from(&extensions),
        )?;
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
        Ok(())
    }
}
