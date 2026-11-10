//! Regenerates `testdata/brep/*.brep` and `expected.json` (BREP-02).
//!
//! ```sh
//! cargo run -p amg-brep --features fixture-gen --example gen_brep_fixtures
//! ```
//!
//! Every part's face, edge and vertex counts and its class histogram are checked against values
//! derived by hand from its construction, so the expectation file is not just what the code
//! under test happened to produce. Only the order of the per-face classes is recorded as built.
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::print_stdout,
    reason = "fixture generator"
)]

#[path = "../common/mod.rs"]
mod common;

use std::collections::BTreeMap;
use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;

use amg_brep::{Shape, SubKind};
use common::{FIXTURES, face_classes};

const MAX_FIXTURE_BYTES: u64 = 2 * 1024 * 1024;

struct Expected {
    faces: u32,
    edges: u32,
    vertices: u32,
    /// `(class letter, count)`.
    classes: &'static [(char, usize)],
}

/// Hand-derived from each part's construction.
fn expected(name: &str) -> Expected {
    let (faces, edges, vertices, classes): (u32, u32, u32, &'static [(char, usize)]) = match name {
        "box" => (6, 12, 8, &[('P', 6)]),
        // Lateral plus two caps; two circles plus the seam; one vertex per circle.
        "cylinder" | "cylinder_mirrored" => (3, 3, 2, &[('P', 2), ('Y', 1)]),
        "cone" => (3, 3, 2, &[('P', 2), ('C', 1)]),
        // One face; the seam and two degenerated pole edges; a vertex at each pole.
        "sphere" => (1, 3, 2, &[('S', 1)]),
        // One face; two seams meeting at one vertex.
        "torus" => (1, 2, 1, &[('T', 1)]),
        // 6 planes, 12 edge blends, 8 corner patches. Planes keep 4 edges each (24); each of
        // the 8 corners adds 3 arcs (24) and a degenerated pole edge (8); each corner has 3
        // vertices, the pole sharing one of them.
        "filleted_box" => (26, 56, 24, &[('P', 6), ('Y', 12), ('S', 8)]),
        // A box plus 32 × 32 through holes, each one lateral face, 2 circles, a seam, 2 vertices.
        "plate_with_holes" => (
            6 + 1024,
            12 + 3 * 1024,
            8 + 2 * 1024,
            &[('P', 6), ('Y', 1024)],
        ),
        // The box twice, at distinct locations.
        "located_compound" => (12, 24, 16, &[('P', 12)]),
        "bspline_patch" => (1, 4, 4, &[('B', 1)]),
        // One face; two circles and the profile, which is the seam.
        "revolved_profile" => (1, 3, 2, &[('O', 1)]),
        other => panic!("no expectation for fixture {other}"),
    };
    Expected {
        faces,
        edges,
        vertices,
        classes,
    }
}

fn check(name: &str, shape: &Shape, classes: &str) {
    let want = expected(name);
    let got = (
        shape.count(SubKind::Face),
        shape.count(SubKind::Edge),
        shape.count(SubKind::Vertex),
    );
    assert_eq!(
        got,
        (want.faces, want.edges, want.vertices),
        "{name}: face/edge/vertex counts"
    );
    for (letter, count) in want.classes {
        let actual = classes.chars().filter(|c| c == letter).count();
        assert_eq!(actual, *count, "{name}: faces of class {letter}");
    }
    assert_eq!(
        classes.len(),
        want.classes.iter().map(|(_, n)| n).sum::<usize>(),
        "{name}: unexpected face classes in {classes}"
    );
}

fn main() {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../testdata/brep");
    std::fs::create_dir_all(&dir).expect("create testdata/brep");

    let mut expectations = BTreeMap::new();
    for name in FIXTURES {
        let shape = Shape::fixture(name).unwrap_or_else(|e| panic!("{name}: {e}"));
        let classes = face_classes(&shape);
        check(name, &shape, &classes);

        let path = dir.join(format!("{name}.brep"));
        let file = File::create(&path).expect("create fixture");
        shape.write_text(BufWriter::new(file)).expect("write text");
        let bytes = std::fs::metadata(&path).expect("stat fixture").len();
        assert!(bytes < MAX_FIXTURE_BYTES, "{name} is {bytes} bytes");
        println!("{name}: {bytes} bytes, {classes}");

        expectations.insert(
            name,
            serde_json::json!({
                "faces": shape.count(SubKind::Face),
                "edges": shape.count(SubKind::Edge),
                "vertices": shape.count(SubKind::Vertex),
                "face_classes": classes,
            }),
        );
    }
    let text = serde_json::to_string_pretty(&expectations).expect("serialize expectations");
    std::fs::write(dir.join("expected.json"), text + "\n").expect("write expectations");
}
