//! Headless facade shared by the CLI and Python: recompute reports, export, convert and
//! physical properties. Every report serializes to JSON with a `schema` field
//! ([`REPORT_SCHEMA`]); the shapes are pinned by `tests/schema/*.json`.

mod export;
mod props;
mod recompute;

use std::path::Path;

use amg_doc::{BodyId, Design, Evaluated};
use amg_io::{Format, IoError};

use crate::ScriptError;

pub use export::{ConvertRequest, ExportRequest, ExportSummary, convert, export};
pub use props::{BodyProps, PropsReport, properties};
pub use recompute::{FeatureHealthRow, RecomputeReport, recompute};

/// Version of the JSON report shapes. Bumped when a field changes meaning or disappears.
pub const REPORT_SCHEMA: u32 = 1;

/// Which bodies an operation works on.
#[derive(Clone, Debug)]
pub enum BodyFilter {
    /// Every visible body with a visible placement.
    AllVisible,
    /// Bodies with one of these names (all of them if a name repeats), visible or not. An
    /// unnamed body is called `Body <feature>.<slot>`, as in the reports.
    Names(Vec<String>),
    /// These bodies, visible or not.
    Ids(Vec<BodyId>),
}

/// The name of a body in reports and in [`BodyFilter::Names`].
fn body_name(design: &Design, id: BodyId) -> String {
    design
        .body_props(id)
        .name
        .unwrap_or_else(|| format!("Body {}.{}", id.origin.0, id.slot))
}

/// The ids a `Names` or `Ids` filter selects, sorted and without repeats; `None` for
/// `AllVisible`.
///
/// # Errors
/// [`ScriptError::NotFound`] for a name or id that no evaluated body has.
fn explicit_bodies(
    design: &Design,
    eval: &Evaluated,
    filter: &BodyFilter,
) -> Result<Option<Vec<BodyId>>, ScriptError> {
    let mut ids = match filter {
        BodyFilter::AllVisible => return Ok(None),
        BodyFilter::Ids(ids) => {
            if let Some(id) = ids.iter().find(|id| eval.body(**id).is_none()) {
                return Err(ScriptError::NotFound {
                    what: "body",
                    name: format!("{}.{}", id.origin.0, id.slot),
                });
            }
            ids.clone()
        }
        BodyFilter::Names(names) => {
            let all: Vec<(BodyId, String)> = eval
                .bodies()
                .map(|(id, _)| (id, body_name(design, id)))
                .collect();
            let mut ids = Vec::new();
            for name in names {
                let before = ids.len();
                ids.extend(all.iter().filter(|(_, n)| n == name).map(|(id, _)| *id));
                if ids.len() == before {
                    return Err(ScriptError::NotFound {
                        what: "body",
                        name: name.clone(),
                    });
                }
            }
            ids
        }
    };
    ids.sort_unstable();
    ids.dedup();
    Ok(Some(ids))
}

/// `explicit`, or the format of `path`'s extension.
fn output_format(explicit: Option<Format>, path: &Path) -> Result<Format, ScriptError> {
    explicit
        .or_else(|| {
            path.extension()
                .and_then(|e| e.to_str())
                .and_then(Format::from_extension)
        })
        .ok_or(ScriptError::Io(IoError::UnknownFormat))
}
