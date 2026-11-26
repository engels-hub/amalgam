//! Shared helpers of the IO-06 STEP tests: corpus lookup and scene comparisons.
#![allow(dead_code, reason = "each test file uses a subset")]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use amg_brep::{ForeignModel, ForeignNode, Shape};
use amg_core::Transform;
use amg_io::brep::{BrepContent, BrepNode, BrepScene, srgb_from_linear};
use amg_mesh::Rgba8;

/// Relative tolerance on volumes (Done-when 2).
pub const VOLUME_REL: f64 = 1e-9;
/// Relative error bound of the adaptive volume integration.
pub const VOLUME_EPS: f64 = 1e-12;
/// Absolute tolerance on bounding boxes, mm (Done-when 2).
pub const BBOX_MM: f64 = 1e-6;
/// Absolute tolerance on transform entries.
pub const TRANSFORM_EPS: f64 = 1e-9;

pub fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

pub fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// A checked-in BREP-13 STEP fixture, `None` (skip) for an un-fetched LFS pointer.
pub fn fixture(name: &str) -> Option<PathBuf> {
    amg_testkit::testdata(&format!("brep/step/{name}.step"))
}

/// Every `*.step`/`*.stp` file of `$AMG_TESTDATA_DIR/<rel>`, sorted; `None` (skip, with a
/// notice) when the variable is unset or the directory is absent. Un-fetched LFS pointers are
/// skipped too.
pub fn external_step_files(rel: &str) -> Option<Vec<PathBuf>> {
    let Some(root) = std::env::var_os("AMG_TESTDATA_DIR").map(PathBuf::from) else {
        eprintln!("SKIP: AMG_TESTDATA_DIR is not set ({rel})");
        return None;
    };
    let dir = root.join(rel);
    let Ok(entries) = std::fs::read_dir(&dir) else {
        eprintln!("SKIP: {} does not exist", dir.display());
        return None;
    };
    let mut files: Vec<PathBuf> = entries
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| {
            let ext = p
                .extension()
                .and_then(|e| e.to_str())
                .map(str::to_ascii_lowercase);
            matches!(ext.as_deref(), Some("step" | "stp")) && !is_lfs_pointer(p)
        })
        .collect();
    files.sort();
    if files.is_empty() {
        eprintln!("SKIP: no STEP files in {}", dir.display());
        return None;
    }
    Some(files)
}

pub fn is_lfs_pointer(p: &Path) -> bool {
    use std::io::Read;
    let Ok(mut f) = std::fs::File::open(p) else {
        return false;
    };
    let mut head = [0u8; 42];
    f.read_exact(&mut head).is_ok() && head.starts_with(b"version https://git-lfs.github.com")
}

pub fn tmp(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("io-step");
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

pub fn same_transform(a: &Transform, b: &Transform) -> bool {
    a.to_cols_array()
        .iter()
        .zip(b.to_cols_array().iter())
        .all(|(x, y)| (x - y).abs() <= TRANSFORM_EPS)
}

/// The effective sRGB colour of a node: its own, else its part's.
fn effective(s: &BrepScene, n: &BrepNode) -> Option<Rgba8> {
    match &n.content {
        BrepContent::Shape(o) => n.color.or(s.objects[*o as usize].color),
        BrepContent::Group(_) => n.color,
    }
}

/// Done-when 1: the scene has the same tree as the model BREP-13 read (names, transforms,
/// part/assembly kind, colours in sRGB, shared prototypes as shared objects, face colours).
pub fn assert_matches_foreign(s: &BrepScene, m: &ForeignModel, what: &str) {
    assert_eq!(s.roots.len(), m.roots.len(), "{what}: root count");
    let mut proto_to_obj: BTreeMap<u32, u32> = BTreeMap::new();
    fn walk(
        s: &BrepScene,
        m: &ForeignModel,
        b: &BrepNode,
        f: &ForeignNode,
        path: &str,
        map: &mut BTreeMap<u32, u32>,
    ) {
        let clean = |n: &Option<String>| n.clone().filter(|n| !n.trim().is_empty());
        assert_eq!(b.name, clean(&f.name), "{path}: name");
        assert!(
            same_transform(&b.transform, &f.transform),
            "{path}: transform"
        );
        assert_eq!(
            effective(s, b),
            f.color.map(srgb_from_linear),
            "{path}: colour"
        );
        match (&b.content, f.instance_of, f.shape.is_some()) {
            (BrepContent::Shape(o), Some(p), _) => {
                let prev = map.insert(p, *o);
                assert!(prev.is_none_or(|q| q == *o), "{path}: sharing");
                let obj = &s.objects[*o as usize];
                let proto = &m.prototypes[p as usize];
                assert_eq!(obj.name, clean(&proto.name), "{path}: part name");
                let want: Vec<(u32, Rgba8)> = {
                    let mut v: Vec<_> = proto
                        .face_colors
                        .iter()
                        .map(|&(face, c)| (face.0, srgb_from_linear(c)))
                        .collect();
                    v.sort_by_key(|x| x.0);
                    v.dedup_by_key(|x| x.0);
                    v
                };
                assert_eq!(obj.face_colors, want, "{path}: face colours");
            }
            (BrepContent::Shape(_), None, true) => {}
            (BrepContent::Group(children), None, false) => {
                assert_eq!(children.len(), f.children.len(), "{path}: child count");
                for (i, (bc, fc)) in children.iter().zip(&f.children).enumerate() {
                    walk(s, m, bc, fc, &format!("{path}/{i}"), map);
                }
            }
            other => panic!("{path}: kind differs: {other:?}"),
        }
    }
    for (i, (b, f)) in s.roots.iter().zip(&m.roots).enumerate() {
        walk(s, m, b, f, &format!("{what}:{i}"), &mut proto_to_obj);
    }
}

/// Facts of one shape compared by the round trips.
#[derive(Debug)]
pub struct Facts {
    pub solids: usize,
    pub faces: u32,
    pub volume: Option<f64>,
    pub bbox: [f64; 6],
}

pub fn facts(shape: &Shape) -> Facts {
    let bb = shape.bbox();
    Facts {
        solids: shape.solids().len(),
        faces: shape.count(amg_brep::SubKind::Face),
        volume: if shape.solids().is_empty() {
            None
        } else {
            // Adaptive integration: the plain query's fixed-order quadrature is only ~1e-4
            // relative on faces whose boundaries are not iso-lines, and a STEP write changes
            // the pcurves, so it would differ by more than the 1e-9 bound.
            shape.volume_adaptive(VOLUME_EPS).ok()
        },
        bbox: [bb.min.x, bb.min.y, bb.min.z, bb.max.x, bb.max.y, bb.max.z],
    }
}

/// How the parts of a round trip compared.
#[derive(Debug, Default)]
pub struct RoundTripStats {
    /// Parts at kernel tolerance (max tolerance <= 1e-6 mm), held to 1e-9 / 1e-6 mm.
    pub strict: usize,
    /// Parts whose edges or vertices carry larger tolerances (sloppy input).
    pub tolerant: usize,
    /// Largest relative volume difference of a strict part.
    pub max_rel_strict: f64,
    /// Largest relative volume difference of a tolerant part.
    pub max_rel_tolerant: f64,
    /// Largest bbox difference, mm.
    pub max_bbox_mm: f64,
    /// Unnamed nodes or parts that came back with OCCT's generated `Product <n>` name.
    pub generated_names: usize,
}

impl RoundTripStats {
    pub fn add(&mut self, o: &RoundTripStats) {
        self.strict += o.strict;
        self.tolerant += o.tolerant;
        self.max_rel_strict = self.max_rel_strict.max(o.max_rel_strict);
        self.max_rel_tolerant = self.max_rel_tolerant.max(o.max_rel_tolerant);
        self.max_bbox_mm = self.max_bbox_mm.max(o.max_bbox_mm);
        self.generated_names += o.generated_names;
    }
}

/// OCCT's STEP writer names every product: a label without a name (or with a blank one) is
/// written as `Product <n>` (BREP-13 leftover, S02 Interface changes request from IO-06). So
/// an unnamed node may come back with exactly that generated name; any other difference fails.
pub fn same_name(a: &Option<String>, b: &Option<String>, path: &str, stats: &mut RoundTripStats) {
    if a == b {
        return;
    }
    let generated = b.as_deref().is_some_and(|n| {
        n.strip_prefix("Product ")
            .is_some_and(|d| !d.is_empty() && d.bytes().all(|c| c.is_ascii_digit()))
    });
    assert!(a.is_none() && generated, "{path}: name {a:?} vs {b:?}");
    stats.generated_names += 1;
}

/// The kernel's linear tolerance (S02): parts at or below it are "exact".
pub const KERNEL_TOL: f64 = 1e-6;

/// Done-when 2: two scenes have the same tree (names, transforms, colours), the same sharing
/// of objects (a consistent bijection), the same part names and face colours, and every part
/// the same solid and face count, volume within 1e-9 relative and bbox within 1e-6 mm.
///
/// STEP stores no per-edge or per-vertex tolerance, so a part whose B-rep carries tolerances
/// above the kernel's (`max_tolerance > 1e-6 mm`, common in third-party files) is rebuilt by
/// the reader with recomputed tolerances and pcurves, and its exact geometry moves within its
/// own tolerance. For such a part the bound is what that tolerance allows: the volume within
/// `area * max_tolerance` (a skin as thick as the tolerance) and the bbox within the tolerance.
pub fn assert_same_scene(a: &BrepScene, b: &BrepScene, what: &str) -> RoundTripStats {
    assert_eq!(a.roots.len(), b.roots.len(), "{what}: root count");
    let mut map: BTreeMap<u32, u32> = BTreeMap::new();
    let mut stats = RoundTripStats::default();
    fn walk(
        a: &BrepScene,
        b: &BrepScene,
        x: &BrepNode,
        y: &BrepNode,
        path: &str,
        map: &mut BTreeMap<u32, u32>,
        stats: &mut RoundTripStats,
    ) {
        same_name(&x.name, &y.name, path, stats);
        assert!(
            same_transform(&x.transform, &y.transform),
            "{path}: transform"
        );
        assert_eq!(effective(a, x), effective(b, y), "{path}: colour");
        match (&x.content, &y.content) {
            (BrepContent::Shape(i), BrepContent::Shape(j)) => {
                let prev = map.insert(*i, *j);
                assert!(prev.is_none_or(|p| p == *j), "{path}: sharing");
                if prev.is_some() {
                    return;
                }
                let (oa, ob) = (&a.objects[*i as usize], &b.objects[*j as usize]);
                same_name(&oa.name, &ob.name, path, stats);
                assert_eq!(oa.face_colors, ob.face_colors, "{path}: face colours");
                let (fa, fb) = (facts(&oa.shape), facts(&ob.shape));
                assert_eq!(
                    (fa.solids, fa.faces),
                    (fb.solids, fb.faces),
                    "{path}: topology"
                );
                let tol = oa.shape.max_tolerance().max(ob.shape.max_tolerance());
                let strict = tol <= KERNEL_TOL;
                if strict {
                    stats.strict += 1;
                } else {
                    stats.tolerant += 1;
                }
                if let (Some(va), Some(vb)) = (fa.volume, fb.volume) {
                    let rel = (va - vb).abs() / va.abs().max(1e-300);
                    let bound = if strict {
                        VOLUME_REL * va.abs()
                    } else {
                        let area: f64 = oa.shape.faces().map(|f| oa.shape.face_area(f)).sum();
                        (VOLUME_REL * va.abs()).max(area * tol)
                    };
                    assert!(
                        (va - vb).abs() <= bound,
                        "{path}: volume {va} vs {vb} (tolerance {tol} mm, bound {bound})"
                    );
                    if strict {
                        stats.max_rel_strict = stats.max_rel_strict.max(rel);
                    } else {
                        stats.max_rel_tolerant = stats.max_rel_tolerant.max(rel);
                    }
                } else {
                    assert_eq!(fa.volume.is_some(), fb.volume.is_some(), "{path}: volume");
                }
                let bbox_bound = if strict { BBOX_MM } else { BBOX_MM.max(tol) };
                // Parts without geometry (empty PMI compounds) have empty boxes on both sides.
                for (p, q) in fa.bbox.iter().zip(&fb.bbox).filter(|(p, q)| p != q) {
                    let d = (p - q).abs();
                    stats.max_bbox_mm = stats.max_bbox_mm.max(d);
                    assert!(
                        d <= bbox_bound,
                        "{path}: bbox {:?} vs {:?}",
                        fa.bbox,
                        fb.bbox
                    );
                }
            }
            (BrepContent::Group(cx), BrepContent::Group(cy)) => {
                assert_eq!(cx.len(), cy.len(), "{path}: child count");
                for (k, (p, q)) in cx.iter().zip(cy).enumerate() {
                    walk(a, b, p, q, &format!("{path}/{k}"), map, stats);
                }
            }
            other => panic!("{path}: kind differs: {other:?}"),
        }
    }
    for (i, (x, y)) in a.roots.iter().zip(&b.roots).enumerate() {
        walk(a, b, x, y, &format!("{what}:{i}"), &mut map, &mut stats);
    }
    let mut targets: Vec<u32> = map.values().copied().collect();
    targets.sort_unstable();
    let n = targets.len();
    targets.dedup();
    assert_eq!(targets.len(), n, "{what}: two objects merged into one");
    stats
}

/// A one-line summary of a scene for the logs.
pub fn summary(s: &BrepScene) -> String {
    let inst = s.instances().len();
    let solids: usize = s.objects.iter().map(|o| o.shape.solids().len()).sum();
    let faces: u32 = s
        .objects
        .iter()
        .map(|o| o.shape.count(amg_brep::SubKind::Face))
        .sum();
    let coloured = s.objects.iter().filter(|o| o.color.is_some()).count();
    let face_cols: usize = s.objects.iter().map(|o| o.face_colors.len()).sum();
    format!(
        "{} parts, {inst} instances, {solids} solids, {faces} faces, {coloured} coloured parts, \
         {face_cols} face colours, assemblies: {}",
        s.objects.len(),
        s.has_assemblies()
    )
}
