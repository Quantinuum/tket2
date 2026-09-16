use crate::packed_pg_slice::PackedBackend;
use crate::tqe::TQEType;
use pg_bitpacked::apply_u8_tqe_slice;
use pg_qm_tableau::Tableau;

/// A `PackedBackend` implementation that uses scalar operations.
#[derive(Clone, Copy)]
pub(crate) struct ScalarBackend;

impl PackedBackend for ScalarBackend {
    fn apply_tableau_to_pauli(
        &self,
        tableau: &Tableau,
        z_bits: &[u64],
        x_bits: &[u64],
    ) -> (Vec<u64>, Vec<u64>, bool) {
        tableau.apply_to_pauli(z_bits, x_bits)
    }

    fn invert_tableau(&self, tableau: &Tableau) -> Tableau {
        tableau.invert()
    }

    fn compose_tableau(&self, lhs: &mut Tableau, rhs: &Tableau) {
        lhs.compose(rhs);
    }

    fn apply_tqe_all(
        &self,
        zb_q0: &mut [u64],
        xb_q0: &mut [u64],
        zb_q1: &mut [u64],
        xb_q1: &mut [u64],
        sign_bits: &mut [u64],
        gate_type: TQEType,
    ) {
        apply_u8_tqe_slice(zb_q0, xb_q0, zb_q1, xb_q1, sign_bits, gate_type);
    }
}
