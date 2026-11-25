//! The BREP-10 defect corpus (`testdata/brep/defects/<name>.brep`), shared by `tests/heal.rs` and
//! the `gen_brep_defects` generator.
#![allow(dead_code)]

use amg_brep::HealOpts;

/// Every defect fixture, in generation order.
pub const DEFECTS: [&str; 17] = [
    "vertex_off_edges",
    "edge_off_faces",
    "missing_pcurve",
    "pcurve_off_edge",
    "reversed_lateral",
    "reversed_face",
    "same_parameter_off",
    "same_range_off",
    "wire_gaps",
    "wire_order",
    "reversed_wire",
    "inside_out",
    "vertex_tolerance",
    "small_edge",
    "bow_tie",
    "open_solid",
    "overlapping_boxes",
];

/// The healing options of the corpus: gaps up to 1e-4 mm are closed, tolerances stay ≤ 1e-3 mm.
pub fn heal_opts() -> HealOpts {
    HealOpts {
        tol: 1e-4,
        fix_small_edges: true,
        fix_orientation: true,
        max_tol: 1e-3,
    }
}
