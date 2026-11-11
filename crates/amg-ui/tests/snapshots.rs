//! UI-01 Done-when 3: shell snapshots in light and dark, rendered with wgpu on WARP.
//!
//! Skipped when no adapter is available (unless `AMG_REQUIRE_GPU=1`). Bless with
//! `UPDATE_SNAPSHOTS=1` (kittest convention); images live in `tests/snapshots/`.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::SnapshotOptions;
use amg_ui::theme::Theme;

/// The software adapter (WARP on Windows), as in CI.
fn warp() -> Option<egui_wgpu::WgpuSetup> {
    if !amg_testkit::gpu_available() {
        return None;
    }
    let cfg = GpuConfig {
        force_fallback_adapter: true,
        ..GpuConfig::default()
    };
    let gpu = match Gpu::new_blocking(&cfg) {
        Ok(gpu) => gpu,
        Err(e) => {
            assert!(
                std::env::var("AMG_REQUIRE_GPU").as_deref() != Ok("1"),
                "no software adapter: {e}"
            );
            eprintln!("skipping: no software adapter ({e})");
            return None;
        }
    };
    Some(egui_wgpu::WgpuSetup::Existing(
        egui_wgpu::WgpuSetupExisting {
            instance: gpu.instance().clone(),
            adapter: gpu.adapter().clone(),
            device: gpu.device().clone(),
            queue: gpu.queue().clone(),
        },
    ))
}

fn snapshot(theme: Theme, name: &str) {
    let Some(setup) = warp() else {
        return;
    };
    let mut h = AppHarness::builder()
        .theme(theme)
        .size(egui::vec2(1280.0, 800.0))
        .wgpu(setup)
        .build();
    h.drive_until_idle();
    // WARP rasterises identically run to run; the tolerance covers driver updates.
    let opts = SnapshotOptions::new().threshold(0.6).max_failed_pixels(64);
    h.kittest().snapshot_options(name, &opts);
}

#[test]
fn gpu_shell_dark_snapshot() {
    snapshot(Theme::Dark, "shell_dark");
}

#[test]
fn gpu_shell_light_snapshot() {
    snapshot(Theme::Light, "shell_light");
}
