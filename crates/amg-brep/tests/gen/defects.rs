//! Regenerates `testdata/brep/defects/*.brep`, the BREP-10 healing and check corpus.
//!
//! ```sh
//! cargo run -p amg-brep --features fixture-gen --example gen_brep_defects
//! ```
//!
//! Each shape is a valid part with one defect made by construction (`cpp/occt_fixtures.cpp`,
//! `defect_*`). The generator checks that the text file reads back with the same `check` report,
//! so the defect survives the round trip, and prints what `check` and `heal` say about it. The
//! expected problem kinds live in `tests/heal.rs`, written from the construction.
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::print_stdout,
    reason = "fixture generator"
)]

#[path = "../common/defects.rs"]
mod defects;

use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;

use amg_brep::{BrepKernel, CheckOpts, JobCtx, KernelConfig, OcctKernel, Shape, Validate};
use defects::{DEFECTS, heal_opts};

fn main() {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../testdata/brep/defects");
    std::fs::create_dir_all(&dir).expect("create testdata/brep/defects");
    let k = OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    });
    let full = CheckOpts {
        self_interference: true,
    };
    for name in DEFECTS {
        let shape =
            Shape::fixture(&format!("defect_{name}")).unwrap_or_else(|e| panic!("{name}: {e}"));
        let path = dir.join(format!("{name}.brep"));
        let file = File::create(&path).expect("create defect file");
        shape.write_text(BufWriter::new(file)).expect("write text");
        let back = Shape::read_text(File::open(&path).expect("open")).expect("read back");
        let before = k.check(&shape, &full);
        let after = k.check(&back, &full);
        assert_eq!(
            before.problem_kinds(),
            after.problem_kinds(),
            "{name}: the defect changed on the round trip"
        );
        let healed = k.heal(&back, &heal_opts(), &JobCtx::detached());
        let healed = match healed {
            Ok(r) => format!(
                "valid {} {:?} warnings {:?}",
                k.check(&r.shape, &CheckOpts::default()).valid,
                k.check(&r.shape, &CheckOpts::default()).problem_kinds(),
                r.warnings
            ),
            Err(e) => format!("Err {e}"),
        };
        println!(
            "{name}: brepcheck valid {} {:?}; full {:?}; heal: {healed}",
            k.check(&back, &CheckOpts::default()).valid,
            k.check(&back, &CheckOpts::default()).problems,
            after.problems,
        );
    }
}
