//! The embedded built-in library (`data/builtin/*.toml`, parsed once).

use std::sync::OnceLock;

use crate::id::MaterialId;
use crate::library::Library;

/// Id of the built-in library.
pub const BUILTIN_LIBRARY_ID: &str = "amg.builtin";

const MATERIALS: &str = include_str!("../data/builtin/materials.toml");
const APPEARANCES: &str = include_str!("../data/builtin/appearances.toml");

/// The embedded, validated, read-only built-in library (parsed lazily, once).
///
/// Library v1 (MAT-05): 17 materials and 18 appearances, every value cited per value
/// (`data/builtin/materials.toml`); data rules are tested in `tests/builtin_data.rs`.
pub fn builtin() -> &'static Library {
    static LIB: OnceLock<Library> = OnceLock::new();
    LIB.get_or_init(|| {
        let src = format!("{MATERIALS}\n{APPEARANCES}");
        #[allow(clippy::expect_used, reason = "embedded data; a unit test parses it")]
        let mut lib = Library::from_toml_str(&src, "builtin").expect("embedded built-in library");
        lib.read_only = true;
        lib
    })
}

/// `amg.builtin/pla.fff`, the default for unassigned bodies (QUESTIONS #28).
pub fn default_material_id() -> MaterialId {
    #[allow(clippy::expect_used, reason = "constant, valid id")]
    MaterialId::new(BUILTIN_LIBRARY_ID, "pla.fff").expect("valid id")
}
