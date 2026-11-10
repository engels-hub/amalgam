//! ASM-13: pending positions, Capture Position and Revert (S06 note §3): layered transforms,
//! undo, save/load and crash recovery, recompute counters, markers, grounding and direct mode,
//! deleted occurrences, exact events, and a proptest of capture/revert/undo sequences.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use std::collections::BTreeMap;
use std::fs;
use std::path::PathBuf;
use std::process::Command as Process;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;

use amg_core::{PersistentId, Quat, Transform, Vec3};
use amg_doc::DocEvent::{
    HealthChanged, OccurrenceTransformChanged, PendingPositionsChanged, RecomputeFinished,
    RecomputeStarted, TimelineChanged,
};
use amg_doc::joints::{JointState, ValueSel, capture_command, out_of_date};
use amg_doc::testing::TestBox;
use amg_doc::{
    CAPTURE_POSITION_KIND, CapturePositionFeature, Command, CommandError, ComponentId, CreatedId,
    Dependency, DesignMode, DocEvent, Document, Feature, FeatureBox, FeatureError, FeatureId,
    FeatureOutput, Health, OccurrenceId, RecomputeCtx, RecoveryConfig, RecoverySession,
    SaveOptions, load, recover_with_report, save,
};
use amg_jobs::{JobSystem, JobSystemConfig};
use proptest::prelude::*;

fn jobs() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

fn tr(x: f64, y: f64, z: f64) -> Transform {
    Transform::from_translation(Vec3::new(x, y, z))
}

fn rot(axis: Vec3, angle: f64, t: Vec3) -> Transform {
    Transform::from_rotation_translation(Quat::from_axis_angle(axis.normalize(), angle), t)
}

fn max_diff(a: &Transform, b: &Transform) -> f64 {
    a.to_cols_array()
        .iter()
        .zip(b.to_cols_array())
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

// ---- a probe feature: records what an in-context reader sees ---------------------------------

static SEEN: Mutex<BTreeMap<(PersistentId, FeatureId), Transform>> = Mutex::new(BTreeMap::new());

/// Reads one occurrence and records the transform `RecomputeCtx::occurrence_transform` gave.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct Probe {
    occ: OccurrenceId,
}

impl Feature for Probe {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Occurrence(self.occ)]
    }
    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let t = ctx.occurrence_transform(self.occ).expect("occurrence");
        SEEN.lock()
            .unwrap()
            .insert((ctx.design().meta().uuid, ctx.feature_id()), t);
        Ok(FeatureOutput::default())
    }
}

amg_doc::register_feature!(Probe, "test.asm13_probe", version = 1);

fn seen(doc: &Document, f: FeatureId) -> Transform {
    SEEN.lock().unwrap()[&(doc.design().meta().uuid, f)]
}

// ---- fixture ----------------------------------------------------------------------------------

struct Fx {
    doc: Document,
    js: JobSystem,
    c: ComponentId,
    o: OccurrenceId,
    p: OccurrenceId,
    g: OccurrenceId,
}

fn created_occ(out: &amg_doc::CommandOutcome) -> (ComponentId, OccurrenceId) {
    let mut c = None;
    let mut o = None;
    for id in &out.created {
        match id {
            CreatedId::Component(x) => c = Some(*x),
            CreatedId::Occurrence(x) => o = Some(*x),
            _ => {}
        }
    }
    (c.unwrap(), o.unwrap())
}

const BASE_O: fn() -> Transform = || rot(Vec3::Z, 0.3, Vec3::new(1.0, 2.0, 3.0));

impl Fx {
    fn new() -> Fx {
        amg_doc::testing::link();
        let mut doc = Document::new("asm13");
        let root = doc.design().root();
        let mut comp = |name: &str, t: Transform| {
            created_occ(
                &doc.apply(Command::NewComponent {
                    parent: root,
                    name: name.into(),
                    transform: t,
                })
                .unwrap(),
            )
        };
        let (c, o) = comp("C", BASE_O());
        let (_, p) = comp("D", tr(5.0, 0.0, 0.0));
        let (_, g) = comp("G", tr(0.0, 9.0, 0.0));
        doc.apply(Command::SetOccurrenceGrounded {
            occ: g,
            grounded: true,
        })
        .unwrap();
        let js = jobs();
        doc.recompute_blocking(&js).unwrap();
        doc.drain_events();
        Fx {
            doc,
            js,
            c,
            o,
            p,
            g,
        }
    }

    fn apply(&mut self, cmd: Command) -> amg_doc::CommandOutcome {
        self.doc.apply(cmd).unwrap()
    }

    fn recompute(&mut self) {
        self.doc.recompute_blocking(&self.js).unwrap();
    }

    fn local(&self, o: OccurrenceId) -> Transform {
        self.doc.evaluated().placements().local(o).unwrap()
    }

    fn pend(&mut self, o: OccurrenceId, t: Transform) {
        self.apply(Command::SetPendingPositions {
            moves: vec![(o, t)],
            joint_states: Vec::new(),
        });
        self.recompute();
    }

    fn capture(&mut self) -> FeatureId {
        let cmd = capture_command(self.doc.design(), self.doc.evaluated(), None).unwrap();
        let CreatedId::Feature(f) = self.apply(cmd).created[0] else {
            panic!()
        };
        self.recompute();
        f
    }

    fn revert(&mut self) {
        self.apply(Command::RevertPositions);
        self.recompute();
    }

    fn probe(&mut self, occ: OccurrenceId) -> FeatureId {
        let root = self.doc.design().root();
        let CreatedId::Feature(f) = self
            .apply(Command::AddFeature {
                at: None,
                feature: FeatureBox::new(Probe { occ }),
                context: root,
                name: None,
            })
            .created[0]
        else {
            panic!()
        };
        f
    }

    fn marker(&mut self, index: usize) {
        self.apply(Command::SetMarker { index });
        self.recompute();
    }

    fn events(&mut self) -> Vec<DocEvent> {
        self.doc.drain_events()
    }
}

fn captured_nodes(doc: &Document) -> Vec<FeatureId> {
    doc.design()
        .timeline()
        .nodes()
        .filter(|n| n.feature.kind() == CAPTURE_POSITION_KIND)
        .map(|n| n.id)
        .collect()
}

// ---- 1. move, capture, move, revert -------------------------------------------------------------

#[test]
fn move_capture_move_revert_and_four_undo_entries() {
    let mut fx = Fx::new();
    let (p1, p2) = (
        rot(Vec3::X, 0.5, Vec3::new(7.0, 7.0, 7.0)),
        tr(-3.0, 4.0, 5.0),
    );
    let base = BASE_O();

    fx.pend(fx.o, p1);
    assert!(fx.doc.design().has_pending_positions());
    assert!(max_diff(&fx.local(fx.o), &p1) < 1e-12);
    assert_eq!(fx.doc.design().occurrence(fx.o).unwrap().transform, base);

    let node = fx.capture();
    assert!(!fx.doc.design().has_pending_positions());
    assert_eq!(captured_nodes(&fx.doc), vec![node]);
    assert!(max_diff(&fx.local(fx.o), &p1) < 1e-12);
    assert_eq!(fx.doc.design().occurrence(fx.o).unwrap().transform, base);

    fx.pend(fx.o, p2);
    assert!(max_diff(&fx.local(fx.o), &p2) < 1e-12);
    fx.revert();
    assert!(!fx.doc.design().has_pending_positions());
    assert!(
        max_diff(&fx.local(fx.o), &p1) < 1e-12,
        "back to the capture"
    );

    // Four entries; undo walks them back in order.
    assert!(fx.doc.undo());
    assert_eq!(fx.doc.design().pending_positions().get(fx.o), Some(p2));
    fx.recompute();
    assert!(max_diff(&fx.local(fx.o), &p2) < 1e-12, "undo of revert");

    assert!(fx.doc.undo());
    assert!(!fx.doc.design().has_pending_positions());
    assert_eq!(captured_nodes(&fx.doc), vec![node]);

    assert!(fx.doc.undo());
    assert!(
        captured_nodes(&fx.doc).is_empty(),
        "undo of the capture removes the node"
    );
    assert_eq!(
        fx.doc.design().pending_positions().get(fx.o),
        Some(p1),
        "and restores the pending move"
    );

    assert!(fx.doc.undo());
    assert!(!fx.doc.design().has_pending_positions());
    fx.recompute();
    assert!(max_diff(&fx.local(fx.o), &base) < 1e-12);
    // The next undo is the setup (grounding), not a fifth position entry.
    assert_eq!(fx.doc.undo_label(), Some("Ground"));

    // Redo replays all four.
    for _ in 0..4 {
        assert!(fx.doc.redo());
    }
    fx.recompute();
    assert!(max_diff(&fx.local(fx.o), &p1) < 1e-12);
    assert!(!fx.doc.redo());
}

#[test]
fn capture_command_covers_pending_occurrences_only_and_none_when_idle() {
    let mut fx = Fx::new();
    assert!(capture_command(fx.doc.design(), fx.doc.evaluated(), None).is_none());
    assert!(out_of_date(fx.doc.design(), fx.doc.evaluated()).is_empty());
    fx.pend(fx.o, tr(1.0, 1.0, 1.0));
    let Some(Command::CapturePositions { moves, .. }) =
        capture_command(fx.doc.design(), fx.doc.evaluated(), Some("Pose".into()))
    else {
        panic!()
    };
    assert_eq!(moves.len(), 1);
    assert_eq!(moves[0].0, fx.o);
    fx.revert();
    // Revert refuses when nothing is pending.
    let e = fx.doc.apply(Command::RevertPositions).unwrap_err();
    assert!(matches!(e, CommandError::Invalid(_)), "{e:?}");
}

// ---- 2. save/load and crash recovery ----------------------------------------------------------

struct TempDir(PathBuf);

impl TempDir {
    fn new() -> TempDir {
        static N: AtomicU32 = AtomicU32::new(0);
        let p = std::env::temp_dir().join(format!(
            "amg-doc-asm13-{}-{}",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&p).unwrap();
        TempDir(p)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn design_json(doc: &Document) -> Vec<u8> {
    serde_json::to_vec_pretty(doc.design()).unwrap()
}

#[test]
fn pending_moves_survive_save_and_load() {
    let mut fx = Fx::new();
    let joint = fx.probe(fx.p); // any timeline feature can carry a joint state in the layer tests
    let st = JointState {
        values: vec![(ValueSel::Rotation, 7.5)],
    };
    fx.apply(Command::SetPendingPositions {
        moves: vec![(fx.o, rot(Vec3::Y, 1.0, Vec3::new(2.0, 3.0, 4.0)))],
        joint_states: vec![(joint, st.clone())],
    });
    let tmp = TempDir::new();
    let path = tmp.0.join("d.amg");
    save(&path, &fx.doc.snapshot(), &SaveOptions::default()).unwrap();
    let loaded = load(&path).unwrap();
    assert_eq!(
        serde_json::to_vec_pretty(&loaded.design).unwrap(),
        design_json(&fx.doc),
        "design.json equal"
    );
    assert!(loaded.design.has_pending_positions());
    assert_eq!(
        loaded.design.pending_positions().joint_state(joint),
        Some(&st)
    );
    assert_eq!(
        loaded.design.seed_transform(fx.o),
        fx.doc.design().seed_transform(fx.o)
    );
    // A design without pending moves saves no new field at all.
    let idle = Fx::new();
    let text = String::from_utf8(design_json(&idle.doc)).unwrap();
    assert!(!text.contains("pending"), "{text}");
}

/// Child half of [`pending_moves_survive_a_crash`].
#[test]
fn recovery_child() {
    let (Some(dir), Some(out)) = (
        std::env::var_os("AMG_ASM13_DIR"),
        std::env::var_os("AMG_ASM13_OUT"),
    ) else {
        return;
    };
    let mut fx = Fx::new();
    let cfg = RecoveryConfig {
        dir: PathBuf::from(dir),
        snapshot_interval: Duration::ZERO,
    };
    let session = RecoverySession::start(&cfg, fx.doc.design().meta().uuid).unwrap();
    session.attach(&mut fx.doc);
    // The fixture was built before the session started: recovery needs a baseline snapshot.
    session.snapshot_now(&fx.doc).unwrap();
    fx.pend(fx.o, tr(1.0, 2.0, 3.0));
    fx.capture();
    fx.pend(fx.o, tr(9.0, 8.0, 7.0));
    fx.pend(fx.p, rot(Vec3::Z, 0.25, Vec3::ZERO));
    session.flush();
    fs::write(out, design_json(&fx.doc)).unwrap();
    std::process::abort();
}

#[test]
fn pending_moves_survive_a_crash() {
    let tmp = TempDir::new();
    let rec = tmp.0.join("recovery");
    let out = tmp.0.join("expected.json");
    let status = Process::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "recovery_child",
            "--nocapture",
            "--test-threads=1",
        ])
        .env("AMG_ASM13_DIR", &rec)
        .env("AMG_ASM13_OUT", &out)
        .status()
        .unwrap();
    assert!(!status.success());
    let expected = fs::read(&out).unwrap();
    let cfg = RecoveryConfig {
        dir: rec,
        snapshot_interval: Duration::ZERO,
    };
    let found = amg_doc::find_recoverable(&cfg);
    assert_eq!(found.len(), 1);
    let r = recover_with_report(&found[0]).unwrap();
    assert!(r.stopped.is_none(), "{:?}", r.stopped);
    assert_eq!(design_json(&r.document), expected);
    assert!(r.document.design().has_pending_positions());
    assert_eq!(captured_nodes(&r.document).len(), 1);
}

// ---- 3. what a pending move or a capture recomputes -----------------------------------------------

#[test]
fn pending_recomputes_nothing_and_capture_reruns_only_later_in_context_features() {
    let mut fx = Fx::new();
    // [F1 reads o, F3 reads p, F2 reads o]; the capture goes between F1 and F3 (marker 1).
    let f1 = fx.probe(fx.o);
    let f3 = fx.probe(fx.p);
    let f2 = fx.probe(fx.o);
    fx.recompute();
    let total = fx.doc.design().timeline().len();
    assert_eq!(total, 3);
    fx.marker(1);
    let p1 = rot(Vec3::Z, 1.1, Vec3::new(4.0, 4.0, 4.0));
    fx.pend(fx.o, p1);
    assert!(
        fx.doc.last_run().unwrap().computed.is_empty(),
        "pending move computes nothing"
    );
    let cap = fx.capture();
    assert_eq!(fx.doc.design().timeline().marker(), 2);
    // Only the capture node itself computed; nothing else was rolled forward yet.
    assert_eq!(fx.doc.last_run().unwrap().computed, vec![cap]);

    // Roll forward again: F3 and F2 compute for the first time; F2 sees the captured position,
    // F1 (before the node) the base one.
    fx.marker(4);
    assert_eq!(fx.doc.design().timeline().len(), 4);
    let key_f1 = fx.doc.evaluated().node(f1).unwrap().key;
    let key_f2 = fx.doc.evaluated().node(f2).unwrap().key;
    assert!(max_diff(&seen(&fx.doc, f1), &BASE_O()) < 1e-15);
    assert!(max_diff(&seen(&fx.doc, f2), &p1) < 1e-15);
    assert!(max_diff(&seen(&fx.doc, f3), &tr(5.0, 0.0, 0.0)) < 1e-15);

    // A pending move on o: still nothing recomputed, keys unchanged.
    fx.pend(fx.o, tr(50.0, 0.0, 0.0));
    assert!(fx.doc.last_run().unwrap().computed.is_empty());
    assert_eq!(fx.doc.evaluated().node(f2).unwrap().key, key_f2);
    assert!(
        max_diff(&seen(&fx.doc, f2), &p1) < 1e-15,
        "features never see pending moves"
    );
    // Captured at the end of the timeline: only the new node computes (nothing follows it).
    fx.capture();
    let run = fx.doc.last_run().unwrap();
    let new_cap = *captured_nodes(&fx.doc).last().unwrap();
    assert_eq!(run.computed, vec![new_cap]);

    // Insert a capture of p between F1 and F3: only F3 follows and reads p.
    fx.marker(1);
    fx.pend(fx.p, tr(0.0, 0.0, 77.0));
    let cap_p = fx.capture();
    fx.marker(6);
    let run = fx.doc.last_run().unwrap();
    assert!(run.computed.contains(&f3));
    assert!(!run.computed.contains(&f1));
    assert_eq!(fx.doc.evaluated().node(f1).unwrap().key, key_f1);
    let _ = cap_p;
}

// ---- 4. the timeline example --------------------------------------------------------------------

#[test]
fn f1_sees_base_f2_sees_the_capture_and_marker_or_suppression_hides_it() {
    let mut fx = Fx::new();
    let f1 = fx.probe(fx.o);
    fx.recompute();
    let p1 = tr(11.0, 12.0, 13.0);
    fx.pend(fx.o, p1);
    let c1 = fx.capture();
    let f2 = fx.probe(fx.o);
    fx.recompute();
    assert!(max_diff(&seen(&fx.doc, f1), &BASE_O()) < 1e-15);
    assert!(max_diff(&seen(&fx.doc, f2), &p1) < 1e-15);
    let key_f2_with = fx.doc.evaluated().node(f2).unwrap().key;
    assert!(max_diff(&fx.local(fx.o), &p1) < 1e-12);

    let d = fx.doc.design();
    let idx = |f| d.timeline().node(f).unwrap().0;
    assert_eq!((idx(f1), idx(c1), idx(f2)), (0, 1, 2));
    assert_eq!(d.captured_transform_at(fx.o, idx(f1)), Some(BASE_O()));
    assert_eq!(d.captured_transform_at(fx.o, idx(c1)), Some(BASE_O()));
    assert_eq!(d.captured_transform_at(fx.o, idx(f2)), Some(p1));
    assert_eq!(d.captured_transform_at(fx.o, d.timeline().len()), Some(p1));
    assert_eq!(d.seed_transform(fx.o), Some(p1));

    // Marker before C1: placements use the base transform.
    fx.marker(1);
    assert!(max_diff(&fx.local(fx.o), &BASE_O()) < 1e-12);
    assert_eq!(fx.doc.design().seed_transform(fx.o), Some(BASE_O()));
    fx.marker(3);
    assert!(max_diff(&fx.local(fx.o), &p1) < 1e-12);

    // Suppressing C1 likewise; F2 is back to the key it would have without the capture.
    fx.apply(Command::SetSuppressed {
        ids: vec![c1],
        suppressed: true,
    });
    fx.recompute();
    assert!(max_diff(&fx.local(fx.o), &BASE_O()) < 1e-12);
    // Without the capture F2 reads the base transform like F1 does: same feature data and same
    // captured view, so the very same cache key (a hit on F1's output).
    let key = |f| fx.doc.evaluated().node(f).unwrap().key;
    assert_eq!(key(f2), key(f1));
    assert_ne!(key(f2), key_f2_with);
    fx.apply(Command::SetSuppressed {
        ids: vec![c1],
        suppressed: false,
    });
    fx.recompute();
    assert_eq!(fx.doc.evaluated().node(f2).unwrap().key, key_f2_with);
}

// ---- 5. SetMarker drops pending ---------------------------------------------------------------

#[test]
fn set_marker_clears_pending_in_the_same_undo_entry() {
    let mut fx = Fx::new();
    fx.probe(fx.o);
    fx.probe(fx.p);
    fx.recompute();
    fx.pend(fx.o, tr(3.0, 3.0, 3.0));
    fx.apply(Command::SetMarker { index: 1 });
    assert!(!fx.doc.design().has_pending_positions());
    assert_eq!(fx.doc.design().timeline().marker(), 1);
    assert!(fx.doc.undo());
    assert!(
        fx.doc.design().has_pending_positions(),
        "one entry brings both back"
    );
    assert_eq!(fx.doc.design().timeline().marker(), 2);
    // Moving the marker onto itself keeps pending.
    fx.apply(Command::SetMarker { index: 2 });
    assert!(fx.doc.design().has_pending_positions());
    // Other commands keep pending moves.
    fx.apply(Command::SetOccurrenceVisible {
        occ: fx.p,
        visible: false,
    });
    assert!(fx.doc.design().has_pending_positions());
}

// ---- 6. refusals and direct mode ---------------------------------------------------------------

#[test]
fn grounded_and_direct_mode_are_refused_and_direct_writes_the_base() {
    let mut fx = Fx::new();
    let e = fx
        .doc
        .apply(Command::SetPendingPositions {
            moves: vec![(fx.g, tr(1.0, 0.0, 0.0))],
            joint_states: Vec::new(),
        })
        .unwrap_err();
    assert!(matches!(e, CommandError::Invalid(_)), "{e:?}");
    // All or none: a grounded entry spoils the whole command.
    let e = fx
        .doc
        .apply(Command::SetPendingPositions {
            moves: vec![(fx.o, tr(1.0, 0.0, 0.0)), (fx.g, tr(1.0, 0.0, 0.0))],
            joint_states: Vec::new(),
        })
        .unwrap_err();
    assert!(matches!(e, CommandError::Invalid(_)), "{e:?}");
    assert!(!fx.doc.design().has_pending_positions());
    // Scaled, unknown and duplicate entries.
    for moves in [
        vec![(fx.o, Transform::from_scale(Vec3::splat(2.0)))],
        vec![(OccurrenceId(999_999), Transform::IDENTITY)],
        vec![(fx.o, Transform::IDENTITY), (fx.o, Transform::IDENTITY)],
        vec![],
    ] {
        assert!(
            fx.doc
                .apply(Command::SetPendingPositions {
                    moves,
                    joint_states: Vec::new()
                })
                .is_err()
        );
    }
    // Pending, then Direct: the seeds are folded into the base, nothing stays pending.
    let p1 = tr(6.0, 6.0, 6.0);
    fx.pend(fx.o, p1);
    fx.apply(Command::SetDesignMode {
        mode: DesignMode::Direct,
    });
    assert!(!fx.doc.design().has_pending_positions());
    assert_eq!(fx.doc.design().occurrence(fx.o).unwrap().transform, p1);

    let e = fx
        .doc
        .apply(Command::SetPendingPositions {
            moves: vec![(fx.o, tr(1.0, 0.0, 0.0))],
            joint_states: Vec::new(),
        })
        .unwrap_err();
    assert!(matches!(e, CommandError::Invalid(_)), "{e:?}");
    let e = fx
        .doc
        .apply(Command::CapturePositions {
            moves: vec![(fx.o, tr(1.0, 0.0, 0.0))],
            joint_states: Vec::new(),
            name: None,
        })
        .unwrap_err();
    assert!(matches!(e, CommandError::Invalid(_)), "{e:?}");
    // Direct mode writes the base layer.
    let t = tr(8.0, 8.0, 8.0);
    fx.apply(Command::SetOccurrenceTransforms {
        moves: vec![(fx.o, t)],
    });
    assert_eq!(fx.doc.design().occurrence(fx.o).unwrap().transform, t);
    assert!(!fx.doc.design().has_pending_positions());
    assert!(capture_command(fx.doc.design(), fx.doc.evaluated(), None).is_none());
}

// ---- 7. SetOccurrenceTransforms edits the effective layer ------------------------------------------

#[test]
fn set_occurrence_transforms_edits_the_capture_node_not_the_base() {
    let mut fx = Fx::new();
    let p1 = tr(1.0, 1.0, 1.0);
    fx.pend(fx.o, p1);
    let c1 = fx.capture();
    // p is not listed by any node: base. o is listed: the node.
    fx.pend(fx.o, tr(40.0, 0.0, 0.0));
    let t_o = rot(Vec3::X, 0.4, Vec3::new(2.0, 2.0, 2.0));
    let t_p = tr(0.0, 6.0, 0.0);
    let undo_base = fx.doc.design().occurrence(fx.o).unwrap().transform;
    fx.apply(Command::SetOccurrenceTransforms {
        moves: vec![(fx.o, t_o), (fx.p, t_p)],
    });
    fx.recompute();
    let d = fx.doc.design();
    assert_eq!(
        d.occurrence(fx.o).unwrap().transform,
        undo_base,
        "base untouched"
    );
    assert_eq!(d.occurrence(fx.p).unwrap().transform, t_p, "unlisted: base");
    let node = d.timeline().node(c1).unwrap().1;
    let c = node
        .feature
        .downcast_ref::<CapturePositionFeature>()
        .unwrap();
    assert_eq!(c.get(fx.o), Some(t_o));
    assert!(
        !d.has_pending_positions(),
        "pending entries of those occurrences are gone"
    );
    assert!(max_diff(&fx.local(fx.o), &t_o) < 1e-12);
    assert!(max_diff(&fx.local(fx.p), &t_p) < 1e-12);
    // One undo entry restores the node and the pending move.
    assert!(fx.doc.undo());
    let c = fx
        .doc
        .design()
        .timeline()
        .node(c1)
        .unwrap()
        .1
        .feature
        .downcast_ref::<CapturePositionFeature>()
        .unwrap()
        .get(fx.o);
    assert_eq!(c, Some(p1));
    assert!(fx.doc.design().has_pending_positions());
}

// ---- 8. deleted occurrences -----------------------------------------------------------------------

#[test]
fn deleting_an_occurrence_drops_pending_and_warns_on_the_capture_node() {
    let mut fx = Fx::new();
    fx.pend(fx.o, tr(1.0, 0.0, 0.0));
    fx.pend(fx.p, tr(2.0, 0.0, 0.0));
    let c1 = fx.capture();
    assert_eq!(fx.doc.evaluated().node(c1).unwrap().health, Health::Ok);
    fx.pend(fx.o, tr(3.0, 0.0, 0.0));
    fx.pend(fx.p, tr(4.0, 0.0, 0.0));
    fx.apply(Command::DeleteOccurrences { occs: vec![fx.o] });
    assert_eq!(fx.doc.design().pending_positions().get(fx.o), None);
    assert_eq!(
        fx.doc.design().pending_positions().get(fx.p),
        Some(tr(4.0, 0.0, 0.0))
    );
    fx.recompute();
    assert!(matches!(
        fx.doc.evaluated().node(c1).unwrap().health,
        Health::Warning(_)
    ));
    // The remaining entry still applies.
    assert_eq!(
        fx.doc.design().captured_transform_at(fx.p, 1),
        Some(tr(2.0, 0.0, 0.0))
    );
    assert_eq!(fx.doc.design().captured_transform_at(fx.o, 1), None);
    // Undo brings the occurrence, its pending entry and a healthy node back.
    assert!(fx.doc.undo());
    fx.recompute();
    assert_eq!(fx.doc.evaluated().node(c1).unwrap().health, Health::Ok);
    assert_eq!(
        fx.doc.design().pending_positions().get(fx.o),
        Some(tr(3.0, 0.0, 0.0))
    );
}

#[test]
fn deleting_a_joint_feature_drops_its_pending_state() {
    let mut fx = Fx::new();
    let j = fx.probe(fx.o);
    fx.apply(Command::SetPendingPositions {
        moves: Vec::new(),
        joint_states: vec![(
            j,
            JointState {
                values: vec![(ValueSel::Slide, 2.0)],
            },
        )],
    });
    assert!(fx.doc.design().joint_state(j).is_some());
    fx.apply(Command::DeleteFeatures { ids: vec![j] });
    assert!(!fx.doc.design().has_pending_positions());
    // Unknown joints are refused.
    assert!(
        fx.doc
            .apply(Command::SetPendingPositions {
                moves: Vec::new(),
                joint_states: vec![(FeatureId(424_242), JointState::default())],
            })
            .is_err()
    );
}

#[test]
fn joint_states_follow_the_layers() {
    let mut fx = Fx::new();
    let j = fx.probe(fx.o);
    fx.recompute();
    let s1 = JointState {
        values: vec![(ValueSel::Rotation, 1.0)],
    };
    let s2 = JointState {
        values: vec![(ValueSel::Rotation, 9.0)],
    };
    fx.apply(Command::SetPendingPositions {
        moves: vec![(fx.o, tr(1.0, 0.0, 0.0))],
        joint_states: vec![(j, s1.clone())],
    });
    fx.recompute();
    fx.capture();
    assert_eq!(fx.doc.design().joint_state(j), Some(&s1));
    fx.apply(Command::SetPendingPositions {
        moves: Vec::new(),
        joint_states: vec![(j, s2.clone())],
    });
    assert_eq!(fx.doc.design().joint_state(j), Some(&s2));
    fx.revert();
    assert_eq!(fx.doc.design().joint_state(j), Some(&s1));
}

// ---- 9. events ---------------------------------------------------------------------------------

#[test]
fn exact_event_sets() {
    let mut fx = Fx::new();
    let o = fx.o;
    let rev = |fx: &Fx| fx.doc.history().revision();
    let finished = |r| RecomputeFinished {
        revision: r,
        failed: 0,
    };

    // SetPendingPositions: the pending layer changes at once; the placement follows with the
    // recompute.
    fx.apply(Command::SetPendingPositions {
        moves: vec![(o, tr(9.0, 9.0, 9.0))],
        joint_states: Vec::new(),
    });
    assert_eq!(fx.events(), vec![PendingPositionsChanged]);
    let r = rev(&fx);
    fx.recompute();
    assert_eq!(
        fx.events(),
        vec![
            RecomputeStarted(r),
            OccurrenceTransformChanged(o),
            finished(r)
        ]
    );

    // Capture: a timeline node and the pending layer go; no placement changes.
    let cmd = capture_command(fx.doc.design(), fx.doc.evaluated(), None).unwrap();
    let CreatedId::Feature(cap) = fx.apply(cmd).created[0] else {
        panic!()
    };
    assert_eq!(fx.events(), vec![TimelineChanged, PendingPositionsChanged]);
    let r = rev(&fx);
    fx.recompute();
    assert_eq!(
        fx.events(),
        vec![RecomputeStarted(r), HealthChanged(cap), finished(r)]
    );

    // Revert after another move.
    fx.pend(o, tr(1.0, 1.0, 1.0));
    fx.events();
    fx.apply(Command::RevertPositions);
    assert_eq!(fx.events(), vec![PendingPositionsChanged]);
    let r = rev(&fx);
    fx.recompute();
    assert_eq!(
        fx.events(),
        vec![
            RecomputeStarted(r),
            OccurrenceTransformChanged(o),
            finished(r)
        ]
    );

    // Undo and redo of the revert raise the same sets.
    assert!(fx.doc.undo());
    assert_eq!(fx.events(), vec![PendingPositionsChanged]);
    let r = rev(&fx);
    fx.recompute();
    assert_eq!(
        fx.events(),
        vec![
            RecomputeStarted(r),
            OccurrenceTransformChanged(o),
            finished(r)
        ]
    );
    assert!(fx.doc.redo());
    assert_eq!(fx.events(), vec![PendingPositionsChanged]);
    fx.recompute();
    fx.events();

    // Undo of the revert, the second move and the capture: the node goes, its pending
    // move comes back.
    assert!(fx.doc.undo());
    assert!(fx.doc.undo());
    assert!(fx.doc.undo());
    assert_eq!(fx.events(), vec![TimelineChanged, PendingPositionsChanged]);
}

// ---- proptest: random sequences keep design.json exact -----------------------------------------------

#[derive(Clone, Debug)]
enum Step {
    Move(u8, f64, f64),
    Capture,
    Revert,
    Marker(u8),
    Undo,
    Redo,
    Place(u8, f64),
}

fn step() -> impl Strategy<Value = Step> {
    prop_oneof![
        (0u8..2, -50.0f64..50.0, -3.0f64..3.0).prop_map(|(o, x, a)| Step::Move(o, x, a)),
        Just(Step::Capture),
        Just(Step::Revert),
        (0u8..4).prop_map(Step::Marker),
        Just(Step::Undo),
        Just(Step::Redo),
        (0u8..2, -50.0f64..50.0).prop_map(|(o, x)| Step::Place(o, x)),
    ]
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 24, .. ProptestConfig::default() })]
    #[test]
    fn random_sequences_round_trip_and_placements_follow_the_seeds(
        steps in proptest::collection::vec(step(), 1..40)
    ) {
        let mut fx = Fx::new();
        for _ in 0..3 {
            fx.apply(Command::AddFeature {
                at: None,
                feature: FeatureBox::new(TestBox { size: [1.0, 2.0, 3.0] }),
                context: fx.c,
                name: None,
            });
        }
        let occs = [fx.o, fx.p];
        for s in steps {
            match s {
                Step::Move(i, x, a) => {
                    let _ = fx.doc.apply(Command::SetPendingPositions {
                        moves: vec![(occs[usize::from(i)], rot(Vec3::Z, a, Vec3::new(x, 0.0, 1.0)))],
                        joint_states: Vec::new(),
                    });
                }
                Step::Capture => {
                    fx.recompute();
                    if let Some(cmd) = capture_command(fx.doc.design(), fx.doc.evaluated(), None) {
                        fx.doc.apply(cmd).unwrap();
                    }
                }
                Step::Revert => { let _ = fx.doc.apply(Command::RevertPositions); }
                Step::Marker(m) => {
                    let n = fx.doc.design().timeline().len();
                    let _ = fx.doc.apply(Command::SetMarker { index: usize::from(m).min(n) });
                }
                Step::Undo => { fx.doc.undo(); }
                Step::Redo => { fx.doc.redo(); }
                Step::Place(i, x) => {
                    let _ = fx.doc.apply(Command::SetOccurrenceTransforms {
                        moves: vec![(occs[usize::from(i)], tr(x, 1.0, 2.0))],
                    });
                }
            }
            let d = fx.doc.design();
            let text = serde_json::to_string(d).unwrap();
            let back: amg_doc::Design = serde_json::from_str(&text).unwrap();
            prop_assert_eq!(serde_json::to_string(&back).unwrap(), text);
            fx.recompute();
            for o in occs {
                let want = fx.doc.design().seed_transform(o).unwrap();
                prop_assert!(max_diff(&fx.local(o), &want) < 1e-12);
            }
        }
    }
}
