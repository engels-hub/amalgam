//! The BREP-05 completeness check (S02 §6): every op WP adds its op to the history corpus
//! test through `completeness_error`.
#![allow(dead_code, reason = "shared test helper")]

use amg_brep::{KernelWarning, OpResult, OpRole, Shape, SubKind};

/// Faces, edges and vertices of `s`.
pub fn counts(s: &Shape) -> [u32; 3] {
    [
        s.count(SubKind::Face),
        s.count(SubKind::Edge),
        s.count(SubKind::Vertex),
    ]
}

/// Why `r` (the result of an op run on `operands`) breaks the history contract, or `None`:
/// one `operand_counts` entry per operand, equal to the operand's counts; every input and
/// output index in range; every output face `Modified`/`Generated`/`Kept` from some input or
/// carrying a role other than `Unexplained`; no `PartialHistory` warning.
pub fn completeness_error(r: &OpResult, operands: &[&Shape]) -> Option<String> {
    let h = &r.history;
    if h.operand_counts.len() != operands.len() {
        return Some(format!(
            "{} operand counts for {} operands",
            h.operand_counts.len(),
            operands.len()
        ));
    }
    for (i, (have, s)) in h.operand_counts.iter().zip(operands).enumerate() {
        if *have != counts(s) {
            return Some(format!(
                "operand {i}: counts {have:?}, shape {:?}",
                counts(s)
            ));
        }
    }
    if let Some(e) = h.range_error(counts(&r.shape)) {
        return Some(e);
    }
    let unexplained = h.unexplained_faces(r.shape.count(SubKind::Face));
    if !unexplained.is_empty() {
        return Some(format!("faces without source or role: {unexplained:?}"));
    }
    if h.roles.iter().any(|(_, role)| *role == OpRole::Unexplained) {
        return Some("an Unexplained role".into());
    }
    if r.warnings.contains(&KernelWarning::PartialHistory) {
        return Some("a PartialHistory warning".into());
    }
    None
}
