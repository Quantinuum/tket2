use hugr::{Wire, builder::BuildError};

/// Builder trait for lowering `TketOp`s into a target operation set.
///
/// This is a pure interface: implementors are free to use any underlying gate
/// set. There are no constraints on the implementing type beyond what each
/// method requires of its inputs and outputs.
///
/// For Quantinuum platforms (Helios, Sol), the blanket impl via
/// [`PhasedXRzSynth`] provides all methods automatically.
pub(super) trait SynthesizeTketOp {
    /// Build a Hadamard gate.
    fn build_h(&mut self, qb: Wire) -> Result<Wire, BuildError>;
    /// Build an X gate.
    fn build_x(&mut self, qb: Wire) -> Result<Wire, BuildError>;
    /// Build a Y gate.
    fn build_y(&mut self, qb: Wire) -> Result<Wire, BuildError>;
    /// Build a Z gate.
    fn build_z(&mut self, qb: Wire) -> Result<Wire, BuildError>;
    /// Build an S gate.
    fn build_s(&mut self, qb: Wire) -> Result<Wire, BuildError>;
    /// Build an Sdg gate.
    fn build_sdg(&mut self, qb: Wire) -> Result<Wire, BuildError>;
    /// Build a V gate.
    fn build_v(&mut self, qb: Wire) -> Result<Wire, BuildError>;
    /// Build a Vdg gate.
    fn build_vdg(&mut self, qb: Wire) -> Result<Wire, BuildError>;
    /// Build a T gate.
    fn build_t(&mut self, qb: Wire) -> Result<Wire, BuildError>;
    /// Build a Tdg gate.
    fn build_tdg(&mut self, qb: Wire) -> Result<Wire, BuildError>;
    /// Build an RX gate.
    fn build_rx(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError>;
    /// Build an RY gate.
    fn build_ry(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError>;
    /// Build an RZ gate.
    fn build_rz(&mut self, qb: Wire, theta: Wire) -> Result<Wire, BuildError>;
    /// Build a projective measurement with a conditional flip.
    fn build_measure_flip(&mut self, qb: Wire) -> Result<[Wire; 2], BuildError>;
    /// Build a qalloc operation that panics on failure.
    fn build_qalloc(&mut self) -> Result<Wire, BuildError>;
    /// Build a CNOT gate.
    fn build_cx(&mut self, c: Wire, t: Wire) -> Result<[Wire; 2], BuildError>;
    /// Build a CY gate.
    fn build_cy(&mut self, c: Wire, t: Wire) -> Result<[Wire; 2], BuildError>;
    /// Build a CZ gate.
    fn build_cz(&mut self, c: Wire, t: Wire) -> Result<[Wire; 2], BuildError>;
    /// Build a CRZ gate.
    fn build_crz(&mut self, c: Wire, t: Wire, theta: Wire) -> Result<[Wire; 2], BuildError>;
    /// Build a Toffoli (CCX) gate.
    fn build_toffoli(&mut self, a: Wire, b: Wire, c: Wire) -> Result<[Wire; 3], BuildError>;
}
