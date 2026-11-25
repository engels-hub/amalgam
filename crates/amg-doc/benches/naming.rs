//! DOC-06 benches of the persistent-naming note §9 (results in the note's §13):
//! - `naming/finish-1000`: `NamingBuilder::finish_topo` on a 1000-face / 3000-edge /
//!   2000-vertex body whose faces all pass through the op 1:1 (names and edge `Arc`s reused);
//! - `naming/finish-1000-fresh`: the same body named for the first time (explicit face names);
//! - `naming/index-1000`: building the resolver's lookup index of that body;
//! - `naming/resolve`: stage 1 (an edge and a face, unique) and stage 2 (two split pieces,
//!   fingerprint ratio test).
//!
//! Memory per element is estimated with `BodyNaming::estimated_heap_bytes` (unique name
//! allocations plus vectors; no counting allocator, which would need `unsafe` outside the
//! allowed crates) and printed. Timing budgets are asserted only with `AMG_STRICT_TIMING=1`
//! (the machine is shared).

#![allow(
    clippy::unwrap_used,
    clippy::print_stdout,
    missing_docs,
    reason = "benchmark harness"
)]

use std::collections::HashMap;
use std::hint::black_box;
use std::time::{Duration, Instant};

use amg_brep::{InputSub, Relation, ShapeHistory, SubKind, SubShape};
use amg_core::{Aabb3, Point3, Tolerance, Vec3};
use amg_doc::naming::{GeomSource, NamingTopology, NamingView, SyntheticTopology, resolve_with};
use amg_doc::stub::Shape;
use amg_doc::{
    BodyGeom, BodyId, BodyNaming, FeatureId, GeomClass, GeomFingerprint, NameSeg, NamingBuilder,
    OperandNaming, Role, TopoKind, TopoName, TopoRef,
};
use criterion::{Criterion, criterion_group, criterion_main};

const NF: u32 = 1000;

/// 1000 faces; edge `k` joins face `k % 1000` and `k % 1000 + {1, 14, 27}`; vertex `v` meets
/// edges `v` and `v + 1`. Every edge multiset is unique.
fn topology() -> SyntheticTopology {
    let offs = [1, 14, 27];
    let edge_faces = (0..3 * NF)
        .map(|k| {
            let a = k % NF;
            let b = (a + offs[(k / NF) as usize]) % NF;
            let mut v = vec![a, b];
            v.sort_unstable();
            v
        })
        .collect();
    let vertex_edges = (0..2 * NF).map(|v| vec![v, (v + 1) % (3 * NF)]).collect();
    SyntheticTopology {
        faces: NF,
        edge_faces,
        vertex_edges,
        fingerprints: HashMap::new(),
    }
}

fn face_name(i: u32) -> TopoName {
    TopoName::new(vec![NameSeg::Gen {
        feature: FeatureId(1),
        role: Role::new("Imported", i),
        from: None,
    }])
}

fn fresh(topo: &SyntheticTopology) -> BodyNaming {
    let mut b = NamingBuilder::new(FeatureId(1));
    for f in 0..NF {
        b.generated(TopoKind::Face, f, Role::new("Imported", f), None);
    }
    b.finish_topo(topo).unwrap().0
}

/// A history that keeps every sub-shape (a feature that touches nothing of this body).
fn kept_history(topo: &SyntheticTopology) -> ShapeHistory {
    let [nf, ne, nv] = topo.counts();
    let mut entries = Vec::new();
    for (kind, n) in [
        (SubKind::Face, nf),
        (SubKind::Edge, ne),
        (SubKind::Vertex, nv),
    ] {
        for index in 0..n {
            let sub = SubShape { kind, index };
            entries.push((InputSub { operand: 0, sub }, Relation::Kept(sub)));
        }
    }
    entries.sort_by_key(|(i, _)| *i);
    ShapeHistory {
        entries,
        operand_counts: vec![[nf, ne, nv]],
        ..ShapeHistory::default()
    }
}

struct View {
    naming: BodyNaming,
    geom: BodyGeom,
}

const BODY: BodyId = BodyId {
    origin: FeatureId(1),
    slot: 0,
};

impl NamingView for View {
    fn body_geom(&self, id: BodyId) -> Option<&BodyGeom> {
        (id == BODY).then_some(&self.geom)
    }
    fn naming(&self, id: BodyId) -> Option<&BodyNaming> {
        (id == BODY).then_some(&self.naming)
    }
    fn successors(&self, _id: BodyId) -> &[BodyId] {
        &[]
    }
}

struct Fps(HashMap<(TopoKind, u32), GeomFingerprint>);

impl GeomSource for Fps {
    fn fingerprint(&self, _b: BodyId, kind: TopoKind, index: u32) -> Option<GeomFingerprint> {
        self.0.get(&(kind, index)).cloned()
    }
    fn all(&self, _b: BodyId, _kind: TopoKind) -> Vec<(u32, GeomFingerprint)> {
        Vec::new()
    }
}

fn line_fp(x: f64, len: f64) -> GeomFingerprint {
    GeomFingerprint {
        class: GeomClass::Line,
        measure: len,
        centroid: Point3::new(x, 0.0, 40.0),
        axis: Some(Vec3::X),
        radius: None,
        bbox: Aabb3::new(
            Point3::new(x - len / 2.0, 0.0, 40.0),
            Point3::new(x + len / 2.0, 0.0, 40.0),
        ),
    }
}

/// The 1000-face body plus a split face 0 whose two pieces share two edges with face 1.
fn split_view() -> (View, Fps, TopoRef) {
    let topo = topology();
    let mut naming = fresh(&topo);
    let base = face_name(0);
    let piece = |i| {
        base.with(NameSeg::Split {
            feature: FeatureId(5),
            index: i,
            count: 2,
        })
    };
    naming.faces[0] = piece(0);
    naming.faces.push(piece(1));
    let edge = |p: TopoName| {
        let mut fs = vec![p, naming.faces[1].clone()];
        fs.sort();
        TopoName::new(vec![NameSeg::Bound {
            faces: fs,
            index: 0,
            count: 1,
        }])
    };
    let (e0, e1) = (edge(piece(0)), edge(piece(1)));
    let n = u32::try_from(naming.edges.len()).unwrap();
    naming.edges.push(e0);
    naming.edges.push(e1.clone());
    let fps = Fps(HashMap::from([
        ((TopoKind::Edge, n), line_fp(35.0, 70.0)),
        ((TopoKind::Edge, n + 1), line_fp(85.0, 30.0)),
    ]));
    let r = TopoRef {
        body: BODY,
        kind: TopoKind::Edge,
        name: e1,
        fingerprint: Some(line_fp(80.0, 40.0)),
        key: 1,
    };
    (
        View {
            naming,
            geom: BodyGeom::Solid(Shape::new()),
        },
        fps,
        r,
    )
}

fn strict() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

fn median(mut f: impl FnMut(), n: usize) -> Duration {
    let mut v: Vec<Duration> = (0..n)
        .map(|_| {
            let t = Instant::now();
            f();
            t.elapsed()
        })
        .collect();
    v.sort();
    v[n / 2]
}

fn memory_report() {
    let topo = topology();
    let first = fresh(&topo);
    let fresh_bytes = first.estimated_heap_bytes(None);
    let elements = first.faces.len() + first.edges.len() + first.vertices.len();
    let history = kept_history(&topo);
    let mut b = NamingBuilder::new(FeatureId(2));
    b.from_history(&[(0, OperandNaming::Named(&first))], &history);
    let reused = b.finish_topo(&topo).unwrap().0;
    let reused_bytes = reused.estimated_heap_bytes(Some(&first));
    let shared = first
        .edges
        .iter()
        .zip(&reused.edges)
        .filter(|(a, b)| a.ptr_eq(b))
        .count();
    #[allow(clippy::cast_precision_loss, reason = "report")]
    let per = |b: u64| b as f64 / elements as f64;
    println!(
        "naming/memory: first naming {fresh_bytes} B for {elements} elements ({:.1} B/element); \
         kept op {reused_bytes} B new ({:.1} B/element, {shared}/{} edge names shared)",
        per(fresh_bytes),
        per(reused_bytes),
        reused.edges.len()
    );
    black_box((first, reused));
}

fn benches(c: &mut Criterion) {
    memory_report();
    let topo = topology();
    let first = fresh(&topo);
    let history = kept_history(&topo);
    let tol = Tolerance::default();

    let mut g = c.benchmark_group("naming");
    g.sample_size(20);
    g.bench_function("finish-1000", |b| {
        b.iter(|| {
            let mut nb = NamingBuilder::new(FeatureId(2));
            nb.from_history(&[(0, OperandNaming::Named(&first))], &history);
            black_box(nb.finish_topo(&topo).unwrap())
        });
    });
    g.bench_function("finish-1000-fresh", |b| b.iter(|| black_box(fresh(&topo))));
    g.bench_function("index-1000", |b| {
        b.iter(|| {
            let n = first.clone();
            let r = TopoRef {
                body: BODY,
                kind: TopoKind::Face,
                name: n.faces[3].clone(),
                fingerprint: None,
                key: 1,
            };
            let v = View {
                naming: n,
                geom: BodyGeom::Solid(Shape::new()),
            };
            black_box(resolve_with(&v, &Fps(HashMap::new()), &tol, &r))
        });
    });
    let view = View {
        naming: first.clone(),
        geom: BodyGeom::Solid(Shape::new()),
    };
    let edge = TopoRef {
        body: BODY,
        kind: TopoKind::Edge,
        name: first.edges[1234].clone(),
        fingerprint: Some(line_fp(0.0, 1.0)),
        key: 1,
    };
    let face = TopoRef {
        body: BODY,
        kind: TopoKind::Face,
        name: first.faces[567].clone(),
        fingerprint: None,
        key: 1,
    };
    let none = Fps(HashMap::new());
    let _ = resolve_with(&view, &none, &tol, &edge); // builds the index once
    g.bench_function("resolve/stage1-edge", |b| {
        b.iter(|| black_box(resolve_with(&view, &none, &tol, &edge)));
    });
    g.bench_function("resolve/stage1-face", |b| {
        b.iter(|| black_box(resolve_with(&view, &none, &tol, &face)));
    });
    let (sv, sfps, sr) = split_view();
    let rec = resolve_with(&sv, &sfps, &tol, &sr);
    assert!(rec.result.is_ok(), "{rec:?}");
    g.bench_function("resolve/stage2-split", |b| {
        b.iter(|| black_box(resolve_with(&sv, &sfps, &tol, &sr)));
    });
    g.finish();

    if strict() {
        let finish = median(
            || {
                let mut nb = NamingBuilder::new(FeatureId(2));
                nb.from_history(&[(0, OperandNaming::Named(&first))], &history);
                black_box(nb.finish_topo(&topo).unwrap());
            },
            11,
        );
        assert!(
            finish < Duration::from_millis(3),
            "finish-1000 {finish:?} > 3 ms"
        );
        let s1 = median(
            || drop(black_box(resolve_with(&view, &none, &tol, &edge))),
            101,
        );
        assert!(s1 < Duration::from_micros(20), "stage 1 {s1:?} > 20 µs");
        let s2 = median(|| drop(black_box(resolve_with(&sv, &sfps, &tol, &sr))), 101);
        assert!(s2 < Duration::from_micros(100), "stage 2 {s2:?} > 100 µs");
    }
}

/// With OCCT: the naming share of a real op (a box with its 4 top edges filleted): the kernel
/// op versus `NamingBuilder::finish` on its result (note §9, last row: ≤ 5 %).
#[cfg(feature = "occt")]
fn occt_share(c: &mut Criterion) {
    use amg_brep::{BrepKernel, FilletOpts, FilletSpec, SubShape};
    use amg_core::Plane;
    use amg_doc::naming::default_kernel;
    let k = default_kernel();
    let jb = amg_jobs::JobCtx::detached();
    let bx = k
        .make_box(&Plane::XY, Vec3::new(100.0, 60.0, 40.0))
        .unwrap();
    let mut nb = NamingBuilder::new(FeatureId(1));
    for f in 0..6 {
        nb.generated(TopoKind::Face, f, Role::new("Imported", f), None);
    }
    let base = nb.finish(k, &bx.shape).unwrap();
    let top: Vec<u32> = (0..bx.shape.count(SubKind::Edge))
        .filter(|&e| {
            let g = k
                .sub_geom(
                    &bx.shape,
                    SubShape {
                        kind: SubKind::Edge,
                        index: e,
                    },
                )
                .unwrap();
            (g.centroid.z - 40.0).abs() < 1e-9
        })
        .collect();
    let edges: Vec<_> = top
        .iter()
        .map(|e| (amg_brep::EdgeId(*e), FilletSpec::Constant(5.0)))
        .collect();
    let name = |r: &amg_brep::OpResult| {
        let mut nb = NamingBuilder::new(FeatureId(2));
        nb.from_history(&[(0, OperandNaming::Named(&base))], &r.history);
        for (i, e) in (1u32..).zip(&top) {
            nb.selection(
                0,
                SubShape {
                    kind: SubKind::Edge,
                    index: *e,
                },
                i,
            );
        }
        nb.finish(k, &r.shape).unwrap()
    };
    let r = k
        .fillet(&bx.shape, &edges, &FilletOpts::default(), &jb)
        .unwrap();
    let mut g = c.benchmark_group("naming");
    g.sample_size(20);
    g.bench_function("occt/fillet-4-op", |b| {
        b.iter(|| {
            black_box(
                k.fillet(&bx.shape, &edges, &FilletOpts::default(), &jb)
                    .unwrap(),
            )
        });
    });
    g.bench_function("occt/fillet-4-finish", |b| b.iter(|| black_box(name(&r))));
    g.finish();
    let op = median(
        || {
            drop(black_box(k.fillet(
                &bx.shape,
                &edges,
                &FilletOpts::default(),
                &jb,
            )))
        },
        21,
    );
    let fin = median(|| drop(black_box(name(&r))), 21);
    #[allow(clippy::cast_precision_loss, reason = "report")]
    let share = fin.as_secs_f64() / (op.as_secs_f64() + fin.as_secs_f64()) * 100.0;
    println!("naming/occt: fillet op {op:?}, finish {fin:?}, naming share {share:.1} %");
    if strict() {
        assert!(share <= 5.0, "naming share {share:.1} % > 5 %");
    }
}

#[cfg(not(feature = "occt"))]
fn occt_share(_c: &mut Criterion) {}

criterion_group!(naming, benches, occt_share);
criterion_main!(naming);
