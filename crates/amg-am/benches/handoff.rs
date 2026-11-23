//! AM-06 budget (S16): export + spawn of a 1 M-triangle plate < 3 s (target 0.5 s). The slicer
//! is replaced by a harmless stand-in process. Run with `cargo bench -p amg-am --bench handoff`.
//! The medians are printed always and asserted only under `AMG_STRICT_TIMING=1`.

#![allow(
    clippy::expect_used,
    clippy::print_stderr,
    missing_docs,
    reason = "benchmark setup"
)]

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

use amg_am::handoff::{
    HandoffFormat, HandoffItem, HandoffPlate, SlicerConfig, export_plate, send_to_slicer,
};
use amg_am::machine::{MachineKey, MachineLibrary, SlicerKind};
use amg_am::place::{Placement, effective_transform};
use amg_core::glam::{DQuat, DVec2, DVec3};
use amg_jobs::JobCtx;
use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
use criterion::{Criterion, criterion_group, criterion_main};

fn stand_in() -> SlicerConfig {
    #[cfg(windows)]
    {
        let exe = PathBuf::from(std::env::var_os("ComSpec").unwrap_or_else(|| "cmd.exe".into()));
        let mut cfg = SlicerConfig::new(SlicerKind::Custom, exe);
        cfg.args = vec!["/C".into(), "rem".into(), "{file}".into()];
        cfg
    }
    #[cfg(not(windows))]
    {
        let mut cfg = SlicerConfig::new(SlicerKind::Custom, "/bin/sh");
        cfg.args = vec!["-c".into(), "true".into(), "{file}".into()];
        cfg
    }
}

fn median_ms(mut f: impl FnMut()) -> f64 {
    f(); // warm up
    let mut t: Vec<f64> = (0..7)
        .map(|_| {
            let s = Instant::now();
            f();
            s.elapsed().as_secs_f64() * 1e3
        })
        .collect();
    t.sort_by(f64::total_cmp);
    t[t.len() / 2]
}

fn bench(cr: &mut Criterion) {
    let machine = MachineLibrary::builtin()
        .get(&MachineKey("builtin:creality/ender-3-v3-ke-kcm".into()))
        .expect("machine")
        .clone();
    let mesh = Arc::new(benchmark_sphere(BenchSize::M1, 7));
    let transform = effective_transform(
        &mesh,
        &Placement::on_plate(
            DQuat::from_axis_angle(DVec3::new(1.0, 2.0, 3.0).normalize(), 0.7),
            DVec2::splat(110.0),
        ),
    );
    let plate = HandoffPlate {
        design_name: "bench".into(),
        plate_name: "Plate 1".into(),
        machine: &machine,
        items: vec![HandoffItem {
            name: "Sphere".into(),
            mesh,
            transform,
            filament: Some(1),
            color: None,
        }],
        thumbnail_png: None,
    };
    let dir = std::env::temp_dir().join("amg-am-handoff-bench");
    let ctx = JobCtx::detached();
    let cfg = stand_in();

    let export_ms = median_ms(|| {
        export_plate(
            &plate,
            HandoffFormat::ThreeMf,
            SlicerKind::OrcaSlicer,
            &dir.join("p.3mf"),
            &ctx,
        )
        .expect("export");
    });
    let send_ms = median_ms(|| {
        send_to_slicer(&plate, &cfg, &dir, &ctx).expect("send");
    });
    eprintln!("handoff/export_3mf_1m: {export_ms:.0} ms; export + prune + spawn: {send_ms:.0} ms");

    let mut g = cr.benchmark_group("handoff");
    g.sample_size(10);
    g.bench_function("export_3mf_1m", |b| {
        b.iter(|| {
            export_plate(
                &plate,
                HandoffFormat::ThreeMf,
                SlicerKind::OrcaSlicer,
                &dir.join("p.3mf"),
                &ctx,
            )
            .expect("export")
        });
    });
    g.bench_function("send_1m", |b| {
        b.iter(|| send_to_slicer(&plate, &cfg, &dir, &ctx).expect("send"));
    });
    g.finish();
    let _ = std::fs::remove_dir_all(&dir);

    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(
            send_ms < 3000.0,
            "export + spawn {send_ms:.0} ms (budget 3000)"
        );
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
