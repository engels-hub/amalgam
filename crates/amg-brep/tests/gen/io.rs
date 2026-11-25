//! Regenerates `testdata/brep/step/*.step` and `testdata/brep/iges/*.igs`, the BREP-13 STEP and
//! IGES fixtures, from the models in `tests/common/io_models.rs` (built with the kernel).
//!
//! ```sh
//! cargo run -p amg-brep --features occt --example gen_brep_io
//! ```
//!
//! Each file is read back and printed as a tree (names, colours, solids, volumes), so a change
//! in what OCCT writes shows up in the diff of the files and in this output.
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::print_stdout,
    reason = "fixture generator"
)]

#[path = "../common/io_models.rs"]
mod io_models;

use amg_brep::io::{read_iges, read_step, write_iges, write_step};
use amg_brep::{ForeignModel, ForeignNode, ImportOpts, JobCtx};
use io_models::{IGES_FIXTURES, STEP_FIXTURES, iges_model, iges_path, kernel, shape_facts};
use io_models::{step_model, step_path};

fn print_node(node: &ForeignNode, depth: usize) {
    let facts = node.shape.as_ref().map(shape_facts);
    println!(
        "{:indent$}{:?} proto {:?} colour {:?} faces coloured {} t {:?} {:?}",
        "",
        node.name,
        node.instance_of,
        node.color,
        node.face_colors.len(),
        node.transform.translation,
        facts,
        indent = depth * 2
    );
    for child in &node.children {
        print_node(child, depth + 1);
    }
}

fn print_model(model: &ForeignModel) {
    println!(
        "  unit {} mm, {} prototypes, warnings {:?}",
        model.source_unit_mm,
        model.prototypes.len(),
        model.warnings
    );
    for root in &model.roots {
        print_node(root, 1);
    }
}

fn main() {
    let k = kernel();
    let ctx = JobCtx::detached();
    for (name, schema) in STEP_FIXTURES {
        let path = step_path(name);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        write_step(&path, &step_model(&k, name), schema, &ctx)
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        let back = read_step(&path, &ImportOpts::default(), &ctx)
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        println!(
            "{name} ({schema:?}, {} bytes)",
            std::fs::metadata(&path).unwrap().len()
        );
        print_model(&back);
    }
    for name in IGES_FIXTURES {
        let path = iges_path(name);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        write_iges(&path, &iges_model(&k, name), &ctx).unwrap_or_else(|e| panic!("{name}: {e}"));
        let back = read_iges(&path, &ImportOpts::default(), &ctx)
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        println!(
            "{name} (IGES, {} bytes)",
            std::fs::metadata(&path).unwrap().len()
        );
        print_model(&back);
    }
}
