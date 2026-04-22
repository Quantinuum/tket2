// Patch for ops that have enough info to put together a qualified id but do not for some reason
#[cfg(test)]
pub(crate) mod test {
    use hugr_core::extension::simple_op::MakeOpDef;
    use hugr_core::ops::OpName;

    pub trait HasQualifiedId {
        fn qualified_id(&self) -> OpName;
    }

    #[cfg(test)]
    impl<T: MakeOpDef> HasQualifiedId for T {
        fn qualified_id(&self) -> OpName {
            format!("{}.{}", self.extension(), self.opdef_id()).into()
        }
    }
}
