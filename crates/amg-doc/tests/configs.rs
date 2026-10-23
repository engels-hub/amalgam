//! DOC-10: configurations (S01 §10): overrides, `SetActiveConfig`, per-config cache keys,
//! persistence, events and crash recovery.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;

use amg_doc::testing::{TestCounter, TestParamBox, compute_count};
use amg_doc::{
    Command, CommandError, ConfigId, CreatedId, Dependency, Design, DocEvent, Document, ErrorCause,
    Expr, FeatureBox, FeatureId, Health, ParamId, RecoveryConfig, RecoverySession, SaveOptions,
    find_recoverable, load, recover, save,
};
use amg_jobs::{JobSystem, JobSystemConfig};

struct TempDir(PathBuf);

impl TempDir {
    fn new() -> TempDir {
        static N: AtomicU32 = AtomicU32::new(0);
        let p = std::env::temp_dir().join(format!(
            "amg-doc-configs-{}-{}",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&p).unwrap();
        TempDir(p)
    }
    fn path(&self, n: &str) -> PathBuf {
        self.0.join(n)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn jobs() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

/// A design with parameters `w`, `h`, `d` (10 mm each), a parametric box reading them, and a
/// counter that reads the box.
struct Rig {
    doc: Document,
    jobs: JobSystem,
    w: ParamId,
    h: ParamId,
    d: ParamId,
    boxf: FeatureId,
    reader: FeatureId,
}

fn rig_design() -> (Design, [ParamId; 3], FeatureId, FeatureId) {
    amg_doc::testing::link();
    let mut d = Design::new("Configs");
    let root = d.root();
    let w = d.add_param("wx", "10 mm", None, "").unwrap();
    let h = d.add_param("hy", "10 mm", None, "").unwrap();
    let dd = d.add_param("dz", "10 mm", None, "").unwrap();
    let boxf = d
        .add_feature(
            None,
            FeatureBox::new(TestParamBox {
                size: [Expr::Param(w), Expr::Param(h), Expr::Param(dd)],
            }),
            root,
            None,
        )
        .unwrap();
    let reader = d
        .add_feature(
            None,
            FeatureBox::new(TestCounter {
                deps: vec![Dependency::Body(amg_doc::BodyId {
                    origin: boxf,
                    slot: 0,
                })],
                value: 1,
                fail: false,
            }),
            root,
            None,
        )
        .unwrap();
    (d, [w, h, dd], boxf, reader)
}

impl Rig {
    fn new() -> Rig {
        let (d, [w, h, dd], boxf, reader) = rig_design();
        Rig {
            doc: Document::from_design(d),
            jobs: jobs(),
            w,
            h,
            d: dd,
            boxf,
            reader,
        }
    }

    fn apply(&mut self, cmd: Command) -> Vec<CreatedId> {
        self.doc.apply(cmd).unwrap().created
    }

    fn add_config(&mut self, name: &str) -> ConfigId {
        match self.apply(Command::AddConfig {
            name: name.to_owned(),
        })[..]
        {
            [CreatedId::Config(c)] => c,
            ref other => panic!("{other:?}"),
        }
    }

    fn set(&mut self, c: ConfigId, p: ParamId, expr: &str) {
        self.apply(Command::SetConfigParam {
            config: c,
            param: p,
            expr: Some(expr.to_owned()),
        });
    }

    fn activate(&mut self, c: Option<ConfigId>) {
        self.apply(Command::SetActiveConfig { config: c });
        self.doc.recompute_blocking(&self.jobs).unwrap();
    }

    /// Volume the parametric box reported in the current evaluated state.
    fn volume(&self) -> f64 {
        volume(&self.doc, self.boxf)
    }

    fn computed(&self) -> Vec<FeatureId> {
        self.doc.last_run().unwrap().computed.clone()
    }
}

/// `test.param_box` reports its volume as an informational warning (the stub geometry carries
/// no data), which is cached with the output like geometry.
fn volume(doc: &Document, f: FeatureId) -> f64 {
    match &doc.evaluated().node(f).unwrap().health {
        Health::Warning(ds) => ds[0]
            .message
            .strip_prefix("volume=")
            .unwrap()
            .parse()
            .unwrap(),
        other => panic!("no volume: {other:?}"),
    }
}

/// Small / Medium / Large: volumes 24, 120, 720 (mm³); the base design is 1000.
fn three_configs(r: &mut Rig) -> [ConfigId; 3] {
    let small = r.add_config("Small");
    r.set(small, r.w, "2 mm");
    r.set(small, r.h, "3 mm");
    r.set(small, r.d, "4 mm");
    let medium = r.add_config("Medium");
    r.set(medium, r.w, "4 mm");
    r.set(medium, r.h, "5 mm");
    r.set(medium, r.d, "6 mm");
    let large = r.add_config("Large");
    r.set(large, r.w, "8 mm");
    r.set(large, r.h, "9 mm");
    r.set(large, r.d, "10 mm");
    [small, medium, large]
}

// ---- Done-when 1: per-config volumes, switching back is a full cache hit ------------------

#[test]
fn switching_recomputes_per_config_and_switching_back_is_a_full_cache_hit() {
    let mut r = Rig::new();
    let [small, medium, large] = three_configs(&mut r);
    r.doc.recompute_blocking(&r.jobs).unwrap();
    assert_eq!(
        r.volume(),
        1000.0,
        "no configuration active: the base design"
    );
    assert_eq!(r.computed(), vec![r.boxf, r.reader]);

    let expected = [(small, 24.0), (medium, 120.0), (large, 720.0)];
    for (c, v) in expected {
        r.activate(Some(c));
        assert_eq!(r.volume(), v);
        assert_eq!(r.computed(), vec![r.boxf, r.reader], "first visit computes");
    }
    // Second round, in another order: everything is cached, nothing computes.
    let uuid = r.doc.design().meta().uuid;
    let counts = |r: &Rig| (compute_count(uuid, r.boxf), compute_count(uuid, r.reader));
    let before = counts(&r);
    for (c, v) in [expected[0], expected[2], expected[1], expected[0]] {
        r.activate(Some(c));
        assert_eq!(r.volume(), v);
        assert!(r.doc.last_run().unwrap().computed.is_empty(), "{c:?}");
        assert_eq!(r.doc.last_run().unwrap().cached, vec![r.boxf, r.reader]);
    }
    r.activate(None);
    assert_eq!(r.volume(), 1000.0);
    assert!(r.computed().is_empty(), "the base design is cached too");
    assert_eq!(counts(&r), before, "no feature ran during the second round");
}

#[test]
fn configs_with_equal_values_share_cache_entries() {
    let mut r = Rig::new();
    let a = r.add_config("A");
    r.set(a, r.w, "10 mm"); // same value as the base
    r.doc.recompute_blocking(&r.jobs).unwrap();
    r.activate(Some(a));
    assert!(
        r.computed().is_empty(),
        "keys are built from values, not config ids"
    );
}

#[test]
fn editing_the_base_design_recomputes_only_what_the_active_config_reads() {
    let mut r = Rig::new();
    let c = r.add_config("C");
    r.set(c, r.w, "3 mm");
    r.activate(Some(c));
    assert_eq!(r.volume(), 300.0);
    // `w` is overridden in the active configuration, so editing the base `w` changes nothing
    // the engine sees: the feature keys are unchanged.
    r.apply(Command::SetParamExpr {
        id: r.w,
        expr: "50 mm".to_owned(),
    });
    r.doc.recompute_blocking(&r.jobs).unwrap();
    assert_eq!(r.volume(), 300.0);
    assert!(r.computed().is_empty());
    // `h` is not overridden: it flows through.
    r.apply(Command::SetParamExpr {
        id: r.h,
        expr: "20 mm".to_owned(),
    });
    r.doc.recompute_blocking(&r.jobs).unwrap();
    assert_eq!(r.volume(), 600.0);
    r.activate(None);
    assert_eq!(r.volume(), 50.0 * 20.0 * 10.0);
}

#[test]
fn overrides_may_reference_parameters_and_each_other() {
    let mut r = Rig::new();
    let c = r.add_config("Derived");
    r.set(c, r.w, "hy * 2");
    r.set(c, r.h, "dz / 2");
    r.activate(Some(c));
    // d = 10, h = 5, w = 10.
    assert_eq!(r.volume(), 10.0 * 5.0 * 10.0);
    // The same overrides in the other order of application give the same result (they are
    // evaluated together).
    let eff = r.doc.design().effective();
    let v = |n: &str| {
        eff.params()
            .value(eff.params().by_name(n).unwrap())
            .unwrap()
            .value
    };
    assert_eq!((v("wx"), v("hy"), v("dz")), (10.0, 5.0, 10.0));
    // The base design is untouched.
    let base = r.doc.design();
    assert_eq!(base.params().value(r.w).unwrap().value, 10.0);
}

// ---- suppression overrides ------------------------------------------------------------------

#[test]
fn suppression_overrides_apply_per_config_and_are_cached() {
    let mut r = Rig::new();
    let no_box = r.add_config("No box");
    r.apply(Command::SetConfigSuppressed {
        config: no_box,
        feature: r.boxf,
        suppressed: Some(true),
    });
    let keep = r.add_config("Keep");
    r.doc.recompute_blocking(&r.jobs).unwrap();
    assert!(matches!(
        r.doc.evaluated().node(r.boxf).unwrap().health,
        Health::Warning(_)
    ));

    r.activate(Some(no_box));
    let ev = r.doc.evaluated().clone();
    assert_eq!(ev.node(r.boxf).unwrap().health, Health::Suppressed);
    match &ev.node(r.reader).unwrap().health {
        Health::Error(d) => assert_eq!(d.cause, Some(ErrorCause::Upstream(r.boxf))),
        other => panic!("{other:?}"),
    }
    assert!(ev.bodies().next().is_none());
    // The design's own flags are not touched.
    assert!(!r.doc.design().timeline().node(r.boxf).unwrap().1.suppressed);

    r.activate(Some(keep));
    assert_eq!(r.volume(), 1000.0);
    assert!(r.computed().is_empty(), "the base state was cached");

    // A config can also force a base-suppressed feature back on.
    r.apply(Command::SetSuppressed {
        ids: vec![r.boxf],
        suppressed: true,
    });
    r.apply(Command::SetConfigSuppressed {
        config: keep,
        feature: r.boxf,
        suppressed: Some(false),
    });
    r.doc.recompute_blocking(&r.jobs).unwrap();
    assert_eq!(r.volume(), 1000.0);
    r.activate(None);
    assert_eq!(
        r.doc.evaluated().node(r.boxf).unwrap().health,
        Health::Suppressed
    );
}

// ---- commands -------------------------------------------------------------------------------

#[test]
fn command_errors_leave_the_design_unchanged() {
    let mut r = Rig::new();
    let a = r.add_config("A");
    let b = r.add_config("B");
    r.set(a, r.w, "2 mm");
    let before = serde_json::to_string(r.doc.design()).unwrap();
    let rev = r.doc.history().revision();

    let cases: Vec<(Command, &str)> = vec![
        (Command::AddConfig { name: "A".into() }, "name"),
        (Command::AddConfig { name: "  ".into() }, "empty"),
        (
            Command::RenameConfig {
                config: b,
                name: "A".into(),
            },
            "name",
        ),
        (
            Command::RenameConfig {
                config: ConfigId(999),
                name: "Z".into(),
            },
            "no such",
        ),
        (
            Command::SetConfigParam {
                config: a,
                param: r.h,
                expr: Some("nope + 1".into()),
            },
            "nope",
        ),
        (
            Command::SetConfigParam {
                config: a,
                param: r.h,
                expr: Some("w *".into()),
            },
            "syntax",
        ),
        (
            Command::SetConfigParam {
                config: a,
                param: ParamId(999),
                expr: Some("1".into()),
            },
            "no such",
        ),
        (
            Command::SetConfigParam {
                config: ConfigId(999),
                param: r.h,
                expr: Some("1 mm".into()),
            },
            "no such",
        ),
        (
            Command::SetConfigSuppressed {
                config: a,
                feature: FeatureId(999),
                suppressed: Some(true),
            },
            "no such",
        ),
        (
            Command::SetActiveConfig {
                config: Some(ConfigId(999)),
            },
            "no such",
        ),
        (
            Command::DeleteConfig {
                config: ConfigId(999),
            },
            "no such",
        ),
    ];
    for (cmd, needle) in cases {
        let label = format!("{cmd:?}");
        let e = r.doc.apply(cmd).unwrap_err();
        assert!(
            e.to_string().to_lowercase().contains(needle),
            "{label}: {e}"
        );
        assert_eq!(
            serde_json::to_string(r.doc.design()).unwrap(),
            before,
            "{label}"
        );
        assert_eq!(r.doc.history().revision(), rev, "{label}");
    }
    // A cyclic set of overrides is refused too.
    r.set(a, r.h, "dz");
    let e = r
        .doc
        .apply(Command::SetConfigParam {
            config: a,
            param: r.d,
            expr: Some("hy".into()),
        })
        .unwrap_err();
    assert!(matches!(e, CommandError::Expr(_)), "{e:?}");
}

#[test]
fn duplicate_name_is_name_taken() {
    let mut r = Rig::new();
    r.add_config("A");
    let e = r
        .doc
        .apply(Command::AddConfig { name: "A".into() })
        .unwrap_err();
    assert_eq!(e, CommandError::NameTaken("A".into()));
}

#[test]
fn undo_and_redo_of_activation_recompute_from_cache() {
    let mut r = Rig::new();
    let [small, ..] = three_configs(&mut r);
    r.doc.recompute_blocking(&r.jobs).unwrap();
    r.activate(Some(small));
    assert_eq!(r.volume(), 24.0);
    assert_eq!(r.doc.undo_label(), Some("Activate configuration"));
    assert!(r.doc.undo());
    r.doc.recompute_blocking(&r.jobs).unwrap();
    assert_eq!(r.doc.design().active_config(), None);
    assert_eq!(r.volume(), 1000.0);
    assert!(r.computed().is_empty());
    assert!(r.doc.redo());
    r.doc.recompute_blocking(&r.jobs).unwrap();
    assert_eq!(r.volume(), 24.0);
    assert!(r.computed().is_empty());
}

#[test]
fn rename_param_rewrites_override_expressions() {
    let mut r = Rig::new();
    let c = r.add_config("C");
    r.set(c, r.h, "wx * 2");
    r.apply(Command::RenameParam {
        id: r.w,
        name: "width".into(),
    });
    let cfg = r.doc.design().configs().get(c).unwrap();
    assert_eq!(cfg.params[&r.h], "width * 2");
    r.activate(Some(c));
    assert_eq!(r.volume(), 10.0 * 20.0 * 10.0);
    // One undo entry reverts both (the activation above is its own entry).
    assert!(r.doc.undo());
    assert!(r.doc.undo());
    assert_eq!(
        r.doc.design().configs().get(c).unwrap().params[&r.h],
        "wx * 2"
    );
}

#[test]
fn deleting_params_and_features_drops_their_overrides() {
    let mut r = Rig::new();
    let extra = match r.apply(Command::AddParam {
        name: "extra".into(),
        expr: "1 mm".into(),
        unit: None,
        comment: String::new(),
    })[..]
    {
        [CreatedId::Param(p)] => p,
        ref o => panic!("{o:?}"),
    };
    let c = r.add_config("C");
    r.set(c, extra, "2 mm");
    r.set(c, r.w, "4 mm");
    r.apply(Command::SetConfigSuppressed {
        config: c,
        feature: r.reader,
        suppressed: Some(true),
    });
    r.apply(Command::DeleteParam { id: extra });
    r.apply(Command::DeleteFeatures {
        ids: vec![r.reader],
    });
    let cfg = r.doc.design().configs().get(c).unwrap();
    assert_eq!(cfg.params.keys().copied().collect::<Vec<_>>(), vec![r.w]);
    assert!(cfg.suppressed.is_empty());
    r.doc.design().validate().unwrap();
    // Undo brings the overrides back with the feature.
    assert!(r.doc.undo());
    assert_eq!(r.doc.design().configs().get(c).unwrap().suppressed.len(), 1);
}

#[test]
fn a_base_edit_that_would_break_a_config_is_refused() {
    let mut r = Rig::new();
    let c = r.add_config("C");
    r.set(c, r.w, "hy + 1 mm");
    // Base hy = wx would be fine on its own, but with the override wx = hy + 1 it is a cycle.
    let before = serde_json::to_string(r.doc.design()).unwrap();
    let e = r
        .doc
        .apply(Command::SetParamExpr {
            id: r.h,
            expr: "wx".into(),
        })
        .unwrap_err();
    assert!(e.to_string().contains("`C`"), "{e}");
    assert_eq!(serde_json::to_string(r.doc.design()).unwrap(), before);
    // Deleting a parameter an override expression reads is refused as well.
    let e = r.doc.apply(Command::DeleteParam { id: r.h }).unwrap_err();
    assert!(
        matches!(e, CommandError::InUse { .. } | CommandError::Invalid(_)),
        "{e:?}"
    );
}

#[test]
fn deleting_the_active_config_activates_the_base_design() {
    let mut r = Rig::new();
    let [small, ..] = three_configs(&mut r);
    r.activate(Some(small));
    r.apply(Command::DeleteConfig { config: small });
    assert_eq!(r.doc.design().active_config(), None);
    r.doc.recompute_blocking(&r.jobs).unwrap();
    assert_eq!(r.volume(), 1000.0);
}

#[test]
fn previews_see_the_active_config() {
    let mut r = Rig::new();
    let [small, ..] = three_configs(&mut r);
    r.activate(Some(small));
    let h = r
        .doc
        .preview(
            &r.jobs,
            &Command::SetConfigParam {
                config: small,
                param: r.w,
                expr: Some("3 mm".into()),
            },
        )
        .unwrap();
    let ev = h.wait_blocking().unwrap();
    assert_eq!(volume_of(&ev, r.boxf), 3.0 * 3.0 * 4.0);
    assert_eq!(r.volume(), 24.0, "the document itself is unchanged");
}

fn volume_of(ev: &amg_doc::Evaluated, f: FeatureId) -> f64 {
    match &ev.node(f).unwrap().health {
        Health::Warning(ds) => ds[0]
            .message
            .strip_prefix("volume=")
            .unwrap()
            .parse()
            .unwrap(),
        other => panic!("{other:?}"),
    }
}

// ---- Done-when 3: persistence ---------------------------------------------------------------

#[test]
fn configurations_survive_save_and_load() {
    let tmp = TempDir::new();
    let mut r = Rig::new();
    let [small, medium, large] = three_configs(&mut r);
    r.apply(Command::SetConfigSuppressed {
        config: large,
        feature: r.reader,
        suppressed: Some(true),
    });
    r.activate(Some(medium));
    assert_eq!(r.volume(), 120.0);
    let p = tmp.path("a.amg");
    save(&p, &r.doc.snapshot(), &SaveOptions::default()).unwrap();

    let l = load(&p).unwrap();
    assert_eq!(l.file_version, amg_doc::FORMAT_VERSION);
    assert_eq!(l.design.configs(), r.doc.design().configs());
    assert_eq!(l.design.active_config(), Some(medium));
    assert_eq!(l.design.configs().get(small).unwrap().name, "Small");
    assert_eq!(l.design.configs().get(large).unwrap().suppressed.len(), 1);
    // Saving again is byte-identical.
    let p2 = tmp.path("b.amg");
    save(
        &p2,
        &std::sync::Arc::new(l.design.clone()),
        &SaveOptions::default(),
    )
    .unwrap();
    let design_json = |p: &Path| {
        let mut z = zip::ZipArchive::new(fs::File::open(p).unwrap()).unwrap();
        let mut s = String::new();
        std::io::Read::read_to_string(&mut z.by_name("design.json").unwrap(), &mut s).unwrap();
        s
    };
    assert_eq!(design_json(&p), design_json(&p2));

    // The loaded design evaluates per configuration.
    let mut doc = l.into_document();
    let jobs = jobs();
    doc.recompute_blocking(&jobs).unwrap();
    assert_eq!(volume(&doc, r.boxf), 120.0);
    doc.apply(Command::SetActiveConfig {
        config: Some(small),
    })
    .unwrap();
    doc.recompute_blocking(&jobs).unwrap();
    assert_eq!(volume(&doc, r.boxf), 24.0);
}

#[test]
fn a_persisted_cache_serves_every_configuration_after_load() {
    let tmp = TempDir::new();
    let mut r = Rig::new();
    let cs = three_configs(&mut r);
    r.doc.recompute_blocking(&r.jobs).unwrap();
    for c in cs {
        r.activate(Some(c));
    }
    r.activate(None);
    let p = tmp.path("a.amg");
    let opts = SaveOptions {
        cache: Some(r.doc.cache_export()),
        ..SaveOptions::default()
    };
    save(&p, &r.doc.snapshot(), &opts).unwrap();

    let mut doc = load(&p).unwrap().into_document();
    let jobs = jobs();
    doc.recompute_blocking(&jobs).unwrap();
    assert!(doc.last_run().unwrap().computed.is_empty());
    for (c, v) in cs.into_iter().zip([24.0, 120.0, 720.0]) {
        doc.apply(Command::SetActiveConfig { config: Some(c) })
            .unwrap();
        doc.recompute_blocking(&jobs).unwrap();
        assert_eq!(volume(&doc, r.boxf), v);
        assert!(doc.last_run().unwrap().computed.is_empty(), "{c:?}");
    }
}

#[test]
fn a_loaded_file_with_a_bad_config_is_rejected() {
    let mut r = Rig::new();
    let c = r.add_config("C");
    r.set(c, r.w, "2 mm");
    let good = serde_json::to_value(r.doc.design()).unwrap();
    let load_with = |f: &dyn Fn(&mut serde_json::Value)| {
        let mut v = good.clone();
        f(&mut v);
        serde_json::from_value::<Design>(v)
    };
    assert!(load_with(&|_| {}).is_ok());
    let w = r.w.0.to_string();
    type Edit = Box<dyn Fn(&mut serde_json::Value)>;
    let bad: Vec<(&str, Edit)> = vec![
        (
            "unknown param",
            Box::new(|v| {
                v["configs"]["configs"][0][1]["params"] = serde_json::json!({"99999": "1 mm"});
            }),
        ),
        (
            "unknown feature",
            Box::new(|v| {
                v["configs"]["configs"][0][1]["suppressed"] = serde_json::json!({"99999": true});
            }),
        ),
        (
            "active missing",
            Box::new(|v| v["configs"]["active"] = 99_999.into()),
        ),
        (
            "empty name",
            Box::new(|v| v["configs"]["configs"][0][1]["name"] = "".into()),
        ),
        (
            "bad expr",
            Box::new(move |v| {
                v["configs"]["configs"][0][1]["params"][&w] = "nope".into();
            }),
        ),
        (
            "id clash",
            Box::new(|v| {
                let root = v["root"].clone();
                v["configs"]["configs"][0][0] = root;
            }),
        ),
    ];
    for (what, f) in bad {
        let e = load_with(&*f).expect_err(what);
        eprintln!("{what}: {e}");
    }
}

// ---- DOC-09 events --------------------------------------------------------------------------

#[test]
fn events_for_config_edits_and_activation() {
    let mut r = Rig::new();
    r.doc.recompute_blocking(&r.jobs).unwrap();
    r.doc.drain_events();

    let c = r.add_config("C");
    assert_eq!(r.doc.drain_events(), vec![DocEvent::ConfigsChanged]);

    r.set(c, r.w, "2 mm");
    r.apply(Command::SetConfigSuppressed {
        config: c,
        feature: r.reader,
        suppressed: Some(true),
    });
    assert_eq!(
        r.doc.drain_events(),
        vec![DocEvent::ConfigsChanged],
        "editing an inactive configuration changes nothing the document shows"
    );

    r.apply(Command::SetActiveConfig { config: Some(c) });
    let ev = r.doc.drain_events();
    assert!(ev.contains(&DocEvent::ConfigsChanged), "{ev:?}");
    assert!(
        ev.contains(&DocEvent::ParamsChanged),
        "effective values changed: {ev:?}"
    );
    assert!(
        ev.contains(&DocEvent::TimelineChanged),
        "effective suppression changed: {ev:?}"
    );

    r.doc.recompute_blocking(&r.jobs).unwrap();
    let ev = r.doc.drain_events();
    assert!(ev.contains(&DocEvent::HealthChanged(r.reader)), "{ev:?}");
    assert!(
        ev.contains(&DocEvent::BodyGeometryChanged(amg_doc::BodyId {
            origin: r.boxf,
            slot: 0
        })) || ev.contains(&DocEvent::BodyRemoved(amg_doc::BodyId {
            origin: r.boxf,
            slot: 0
        }))
    );

    // Editing the *active* configuration changes the effective values.
    r.set(c, r.h, "3 mm");
    let ev = r.doc.drain_events();
    assert!(
        ev.contains(&DocEvent::ConfigsChanged) && ev.contains(&DocEvent::ParamsChanged),
        "{ev:?}"
    );

    // An activation that is undone before the next drain emits nothing.
    r.apply(Command::SetActiveConfig { config: None });
    assert!(r.doc.undo());
    assert!(r.doc.drain_events().is_empty());
}

// ---- DOC-14 journal / recovery ---------------------------------------------------------------

#[test]
fn config_commands_are_journaled_and_replayed_after_a_crash() {
    let tmp = TempDir::new();
    let cfg = RecoveryConfig {
        dir: tmp.0.join("rec"),
        snapshot_interval: Duration::ZERO,
    };
    let (d, [w, ..], boxf, reader) = rig_design();
    let mut doc = Document::from_design(d);
    let session = RecoverySession::start(&cfg, doc.design().meta().uuid).unwrap();
    session.attach(&mut doc);
    session.snapshot_now(&doc).unwrap();

    let c = match doc
        .apply(Command::AddConfig {
            name: "Crash".into(),
        })
        .unwrap()
        .created[..]
    {
        [CreatedId::Config(c)] => c,
        ref o => panic!("{o:?}"),
    };
    doc.apply(Command::SetConfigParam {
        config: c,
        param: w,
        expr: Some("7 mm".into()),
    })
    .unwrap();
    let mut tx = doc.transaction("suppress and activate");
    tx.apply(Command::SetConfigSuppressed {
        config: c,
        feature: reader,
        suppressed: Some(true),
    })
    .unwrap();
    tx.apply(Command::SetActiveConfig { config: Some(c) })
        .unwrap();
    tx.commit();
    doc.apply(Command::RenameConfig {
        config: c,
        name: "Crashed".into(),
    })
    .unwrap();
    assert!(doc.undo());
    session.flush();
    let expected = serde_json::to_vec_pretty(doc.design()).unwrap();
    drop(session); // a crash looks like this to the next start

    let found = find_recoverable(&cfg);
    assert_eq!(found.len(), 1);
    let recovered = recover(&found[0]).unwrap();
    assert_eq!(
        serde_json::to_vec_pretty(recovered.design()).unwrap(),
        expected
    );
    assert_eq!(recovered.design().active_config(), Some(c));
    assert_eq!(recovered.design().configs().get(c).unwrap().name, "Crash");
    let jobs = jobs();
    let mut recovered = recovered;
    recovered.recompute_blocking(&jobs).unwrap();
    assert_eq!(volume(&recovered, boxf), 7.0 * 10.0 * 10.0);
}

// ---- format migration -----------------------------------------------------------------------

#[test]
fn v2_files_load_with_no_configurations_and_save_as_v3() {
    amg_doc::testing::link();
    let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/format/v2.amg");
    let l = load(&fixture).unwrap();
    assert_eq!(l.file_version, 2);
    assert!(l.design.configs().is_empty());
    assert_eq!(l.design.active_config(), None);
    assert_eq!(l.design.params().len(), 1);
    let tmp = TempDir::new();
    let p = tmp.path("again.amg");
    save(&p, &std::sync::Arc::new(l.design), &SaveOptions::default()).unwrap();
    let again = load(&p).unwrap();
    assert_eq!(again.file_version, amg_doc::FORMAT_VERSION);
    assert!(again.file_version >= 3);
}
