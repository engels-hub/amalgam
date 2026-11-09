//! RND-01 spike: a mesh spinning in an egui window, drawn on the shared `Gpu` device.
//!
//! ```text
//! cargo run -p amg-render --release --example spin                    # interactive
//! cargo run -p amg-render --release --example spin -- --tris 5000000
//! cargo run -p amg-render --release --example spin -- --stl part.stl
//! cargo run -p amg-render --release --example spin -- --bench 600     # p50/p99, then exits
//! cargo run -p amg-render --example spin -- --stress 10               # resize + MSAA toggling
//! cargo run -p amg-render --release --example spin --features static-dxc -- --measure-shaders
//! cargo run -p amg-render --release --example spin -- --measure-shaders --dxc-dll <dxcompiler.dll>
//! ```

#![allow(
    clippy::print_stdout,
    clippy::unwrap_used,
    clippy::expect_used,
    reason = "a benchmark tool that reports to stdout and stops on setup failures"
)]

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use amg_core::{Aabb3, Dir3, Point3, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_render::egui_glue::{self, Viewport};
use amg_render::{
    Camera, Instance, InstanceKey, MeshData, Projection, Renderer, RendererConfig, ViewId, wgpu,
};
use eframe::egui;
use egui_wgpu::RenderState;

/// Frames dropped before a bench starts measuring, once the window has its final size.
const WARM_UP_FRAMES: u32 = 60;
/// Frames a bench waits for the window to reach the requested size.
const RESIZE_TIMEOUT_FRAMES: u32 = 300;
const BENCH_SIZE_PX: [u32; 2] = [1920, 1080];
const TURNTABLE_RAD_PER_S: f64 = std::f64::consts::PI / 6.0;

#[derive(Clone, Debug)]
struct Args {
    tris: usize,
    stl: Option<PathBuf>,
    bench: Option<u32>,
    stress: Option<f64>,
    measure_shaders: bool,
    dxc_dll: Option<String>,
}

fn parse_args() -> Args {
    let mut out = Args {
        tris: 1_000_000,
        stl: None,
        bench: None,
        stress: None,
        measure_shaders: false,
        dxc_dll: None,
    };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        let mut value = |name: &str| it.next().unwrap_or_else(|| panic!("{name} needs a value"));
        match a.as_str() {
            "--tris" => out.tris = value("--tris").parse().expect("--tris N"),
            "--stl" => out.stl = Some(value("--stl").into()),
            "--bench" => out.bench = Some(value("--bench").parse().expect("--bench FRAMES")),
            "--stress" => out.stress = Some(value("--stress").parse().expect("--stress SECONDS")),
            "--measure-shaders" => out.measure_shaders = true,
            "--dxc-dll" => out.dxc_dll = Some(value("--dxc-dll")),
            other => panic!("unknown argument {other}"),
        }
    }
    out
}

/// An icosahedron subdivided at a frequency chosen so the sphere has at least `min_tris`
/// (`20 f^2`), displaced radially by a few sine waves, radius about 100 mm.
fn displaced_icosphere(min_tris: usize) -> MeshData {
    const T: f64 = 1.618_033_988_749_895;
    const CORNERS: [[f64; 3]; 12] = [
        [-1.0, T, 0.0],
        [1.0, T, 0.0],
        [-1.0, -T, 0.0],
        [1.0, -T, 0.0],
        [0.0, -1.0, T],
        [0.0, 1.0, T],
        [0.0, -1.0, -T],
        [0.0, 1.0, -T],
        [T, 0.0, -1.0],
        [T, 0.0, 1.0],
        [-T, 0.0, -1.0],
        [-T, 0.0, 1.0],
    ];
    const FACES: [[u32; 3]; 20] = [
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];

    let f = ((min_tris as f64 / 20.0).sqrt().ceil() as u32).max(1);
    let (fu, tris) = (f as usize, 20 * (f as usize) * (f as usize));
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(10 * fu * fu + 2);
    let mut indices: Vec<u32> = Vec::with_capacity(tris * 3);
    // Points on icosahedron edges and corners are shared between faces; interior ones are not.
    let mut shared: HashMap<(u32, u32, u32), u32> = HashMap::new();
    let corner = |i: u32| Vec3::from_array(CORNERS[i as usize]);

    let mut row: Vec<Vec<u32>> = Vec::with_capacity(fu + 1);
    for [a, b, c] in FACES {
        row.clear();
        for i in 0..=f {
            let mut ids = Vec::with_capacity(i as usize + 1);
            for j in 0..=i {
                // Barycentric weights of the corners (a, b, c), summing to f.
                let (wa, wb, wc) = (f - i, i - j, j);
                let key = match (wa > 0, wb > 0, wc > 0) {
                    (true, false, false) => Some((a, a, f)),
                    (false, true, false) => Some((b, b, f)),
                    (false, false, true) => Some((c, c, f)),
                    (true, true, false) => Some(edge_key(a, wa, b, wb)),
                    (false, true, true) => Some(edge_key(b, wb, c, wc)),
                    (true, false, true) => Some(edge_key(a, wa, c, wc)),
                    _ => None,
                };
                let new_point = |positions: &mut Vec<[f32; 3]>| {
                    let dir = (corner(a) * f64::from(wa)
                        + corner(b) * f64::from(wb)
                        + corner(c) * f64::from(wc))
                    .normalize();
                    positions.push((dir * displacement(dir)).as_vec3().to_array());
                    (positions.len() - 1) as u32
                };
                ids.push(match key {
                    Some(key) => *shared
                        .entry(key)
                        .or_insert_with(|| new_point(&mut positions)),
                    None => new_point(&mut positions),
                });
            }
            row.push(ids);
        }
        for i in 0..fu {
            for j in 0..=i {
                indices.extend([row[i][j], row[i + 1][j], row[i + 1][j + 1]]);
                if j < i {
                    indices.extend([row[i][j], row[i + 1][j + 1], row[i][j + 1]]);
                }
            }
        }
    }
    let normals = vertex_normals(&positions, &indices);
    MeshData {
        origin: Point3::ZERO,
        positions,
        normals,
        indices,
        closed: true,
    }
}

fn edge_key(a: u32, wa: u32, b: u32, wb: u32) -> (u32, u32, u32) {
    if a < b { (a, b, wa) } else { (b, a, wb) }
}

fn displacement(d: Vec3) -> f64 {
    100.0
        * (1.0
            + 0.05 * (11.0 * d.x).sin() * (9.0 * d.y).sin()
            + 0.03 * (23.0 * d.z + 7.0 * d.x).sin()
            + 0.015 * (47.0 * d.y + 31.0 * d.z).sin())
}

/// Area-weighted vertex normals.
fn vertex_normals(positions: &[[f32; 3]], indices: &[u32]) -> Vec<[f32; 3]> {
    let mut sum = vec![Vec3::ZERO; positions.len()];
    let p = |i: u32| Vec3::from_array(positions[i as usize].map(f64::from));
    for t in indices.chunks_exact(3) {
        let n = (p(t[1]) - p(t[0])).cross(p(t[2]) - p(t[0]));
        for &v in t {
            sum[v as usize] += n;
        }
    }
    sum.iter()
        .map(|n| n.normalize_or_zero().as_vec3().to_array())
        .collect()
}

/// Reads a binary STL (flat-shaded: no shared vertices, no normals), centred on its bounds.
fn read_stl(path: &PathBuf) -> MeshData {
    let bytes = std::fs::read(path).expect("read the STL file");
    assert!(
        bytes.len() >= 84,
        "not a binary STL: shorter than its header"
    );
    let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    assert_eq!(
        bytes.len(),
        84 + count * 50,
        "not a binary STL: the size does not match the triangle count"
    );
    let mut raw = Vec::with_capacity(count * 3);
    for record in bytes[84..].chunks_exact(50) {
        for v in 0..3 {
            let at = 12 + v * 12;
            let f = |o: usize| f32::from_le_bytes(record[at + o..at + o + 4].try_into().unwrap());
            raw.push(Vec3::new(f64::from(f(0)), f64::from(f(4)), f64::from(f(8))));
        }
    }
    let centre = Aabb3::from_points(raw.iter().copied()).center();
    MeshData {
        origin: centre,
        positions: raw
            .iter()
            .map(|p| (*p - centre).as_vec3().to_array())
            .collect(),
        normals: Vec::new(),
        indices: (0..raw.len() as u32).collect(),
        closed: false,
    }
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    let rank = (p * (sorted.len() - 1) as f64).round() as usize;
    sorted[rank.min(sorted.len() - 1)]
}

fn sorted(mut v: Vec<f64>) -> Vec<f64> {
    v.sort_by(f64::total_cmp);
    v
}

/// Bench progress: waits for the window size, warms up, then records.
struct Bench {
    frames: u32,
    wait: u32,
    warm: u32,
    wall_ms: Vec<f64>,
    gpu_ms: Vec<f64>,
    opaque_ms: Vec<f64>,
    final_ms: Vec<f64>,
}

impl Bench {
    fn new(frames: u32) -> Bench {
        Bench {
            frames,
            wait: 0,
            warm: 0,
            wall_ms: Vec::with_capacity(frames as usize),
            gpu_ms: Vec::with_capacity(frames as usize),
            opaque_ms: Vec::with_capacity(frames as usize),
            final_ms: Vec::with_capacity(frames as usize),
        }
    }
}

/// Stress progress for the validation run (RND-01 Done-when 5).
struct Stress {
    until: Instant,
    start: Instant,
    frames: u64,
    msaa_toggles: u64,
    resizes: u64,
    next_toggle: Instant,
    last_size: [u32; 2],
}

struct SpinApp {
    rs: RenderState,
    view: ViewId,
    instance: InstanceKey,
    triangles: u64,
    adapter: String,
    backend: String,
    target: Point3,
    distance: f64,
    azimuth: f64,
    elevation: f64,
    spin: f64,
    last: Instant,
    cpu_ms: f64,
    gpu_ms: f64,
    msaa4: bool,
    flat: bool,
    bench: Option<Bench>,
    stress: Option<Stress>,
    errors: Arc<AtomicUsize>,
}

impl SpinApp {
    fn camera(&self) -> Camera {
        let (sin_az, cos_az) = self.azimuth.sin_cos();
        let (sin_el, cos_el) = self.elevation.sin_cos();
        let dir = Vec3::new(cos_el * sin_az, -cos_el * cos_az, sin_el);
        Camera {
            eye: self.target + dir * self.distance,
            target: self.target,
            up: Dir3::Z,
            projection: Projection::Perspective {
                fov_y: 45f64.to_radians(),
            },
        }
    }

    /// Spins the instance, applies the camera and the toggles.
    fn update_scene(&mut self) {
        let camera = self.camera();
        let (view, instance, spin, msaa4, flat) =
            (self.view, self.instance, self.spin, self.msaa4, self.flat);
        let target = self.target;
        egui_glue::with_renderer(&self.rs, move |r: &mut Renderer| {
            if let Some(i) = r.scene_mut().instance_mut(instance) {
                i.transform = Transform::from_translation(target)
                    * Transform::from_rotation_z(spin)
                    * Transform::from_translation(-target);
            }
            if let Some(v) = r.view_mut(view) {
                v.camera = camera;
                v.style.flat_shading = flat;
            }
            let samples = if msaa4 { 4 } else { 1 };
            if r.msaa_samples() != samples {
                r.set_msaa_samples(samples).expect("1 or 4 samples");
            }
        });
    }

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("amg-render spin");
        ui.label(format!("adapter: {}", self.adapter));
        ui.label(format!("backend: {}", self.backend));
        ui.label(format!("triangles: {}", self.triangles));
        ui.label(format!("CPU frame: {:.2} ms", self.cpu_ms));
        ui.label(format!("GPU frame: {:.2} ms", self.gpu_ms));
        ui.separator();
        ui.label("MSAA");
        ui.horizontal(|ui| {
            ui.radio_value(&mut self.msaa4, false, "1x");
            ui.radio_value(&mut self.msaa4, true, "4x");
        });
        ui.checkbox(&mut self.flat, "flat shading");
    }

    fn navigate(&mut self, ui: &egui::Ui, response: &egui::Response) {
        if response.dragged_by(egui::PointerButton::Primary) {
            let delta = response.drag_delta();
            self.azimuth -= f64::from(delta.x) * 0.008;
            self.elevation = (self.elevation + f64::from(delta.y) * 0.008).clamp(-1.5, 1.5);
        }
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            self.distance *= (-f64::from(scroll) * 0.002).exp();
        }
    }

    /// Advances the bench; returns the window size command while it is still being reached.
    fn bench_step(&mut self, ui: &egui::Ui, size_px: [u32; 2], dt_ms: f64) {
        let view = self.view;
        let Some(bench) = self.bench.as_mut() else {
            return;
        };
        if size_px != BENCH_SIZE_PX && bench.wait < RESIZE_TIMEOUT_FRAMES {
            bench.wait += 1;
            let ppp = ui.ctx().pixels_per_point();
            ui.ctx()
                .send_viewport_cmd(egui::ViewportCommand::InnerSize(egui::vec2(
                    BENCH_SIZE_PX[0] as f32 / ppp,
                    BENCH_SIZE_PX[1] as f32 / ppp,
                )));
            return;
        }
        if bench.warm < WARM_UP_FRAMES {
            bench.warm += 1;
            egui_glue::with_renderer(&self.rs, |r| r.drain_gpu_timings(view));
            return;
        }
        bench.wall_ms.push(dt_ms);
        let timings =
            egui_glue::with_renderer(&self.rs, |r| r.drain_gpu_timings(view)).unwrap_or_default();
        for t in timings {
            bench.gpu_ms.push(f64::from(t.total_ms));
            for (name, ms) in t.passes {
                match name {
                    "opaque" => bench.opaque_ms.push(f64::from(ms)),
                    "final" => bench.final_ms.push(f64::from(ms)),
                    _ => {}
                }
            }
        }
        if bench.wall_ms.len() as u32 == bench.frames {
            self.report_bench(size_px);
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }

    fn report_bench(&self, size_px: [u32; 2]) {
        let Some(bench) = self.bench.as_ref() else {
            return;
        };
        let wall = sorted(bench.wall_ms.clone());
        let gpu = sorted(bench.gpu_ms.clone());
        let opaque = sorted(bench.opaque_ms.clone());
        let final_pass = sorted(bench.final_ms.clone());
        println!(
            "spin bench: adapter=\"{}\" backend={} size={}x{} msaa=4x triangles={} frames={} present=AutoNoVsync",
            self.adapter, self.backend, size_px[0], size_px[1], self.triangles, bench.frames
        );
        println!(
            "frame wall ms: p50={:.3} p99={:.3} max={:.3}",
            percentile(&wall, 0.5),
            percentile(&wall, 0.99),
            wall.last().copied().unwrap_or(f64::NAN)
        );
        println!(
            "gpu ms (opaque start to final end, excludes egui's pass): p50={:.3} p99={:.3} samples={}",
            percentile(&gpu, 0.5),
            percentile(&gpu, 0.99),
            gpu.len()
        );
        println!(
            "gpu pass p50 ms: opaque={:.3} final={:.3}",
            percentile(&opaque, 0.5),
            percentile(&final_pass, 0.5)
        );
        let size_ok = size_px == BENCH_SIZE_PX;
        let wall_ok = percentile(&wall, 0.99) <= 16.7;
        let gpu_ok = percentile(&gpu, 0.5) <= 4.0;
        println!(
            "done-when 1: size {} wall p99 <= 16.7 ms {} gpu p50 <= 4 ms {}",
            verdict(size_ok),
            verdict(wall_ok),
            verdict(gpu_ok)
        );
        println!(
            "wgpu validation errors: {}",
            self.errors.load(Ordering::Relaxed)
        );
        self.report_coverage();
    }

    /// Proves the bench drew the mesh: the share of final-image pixels that differ from the
    /// background, from a capture of the same view at the same size and MSAA.
    fn report_coverage(&self) {
        let view = self.view;
        let captured = egui_glue::with_renderer(&self.rs, |r| r.capture(view));
        let Some(Ok(capture)) = captured else {
            println!("coverage: capture failed");
            return;
        };
        let image = capture.wait().expect("read the capture back");
        let background = image.pixel(0, 0);
        let covered = image
            .pixels
            .chunks_exact(4)
            .filter(|p| p.iter().zip(&background).any(|(a, b)| a.abs_diff(*b) > 8))
            .count();
        let total = image.pixels.len() / 4;
        println!(
            "coverage: {covered} of {total} pixels ({:.1} %) differ from the background; capture {}x{}",
            100.0 * covered as f64 / total as f64,
            image.size[0],
            image.size[1]
        );
    }

    /// Advances the stress run; `false` when it has finished.
    fn stress_step(&mut self, ui: &egui::Ui, size_px: [u32; 2]) {
        let Some(stress) = self.stress.as_mut() else {
            return;
        };
        let now = Instant::now();
        stress.frames += 1;
        if size_px != stress.last_size {
            stress.resizes += 1;
            stress.last_size = size_px;
        }
        if now >= stress.next_toggle {
            stress.next_toggle = now + Duration::from_millis(350);
            self.msaa4 = !self.msaa4;
            stress.msaa_toggles += 1;
        }
        let t = (now - stress.start).as_secs_f32();
        ui.ctx()
            .send_viewport_cmd(egui::ViewportCommand::InnerSize(egui::vec2(
                900.0 + 400.0 * (t * 2.9).sin(),
                600.0 + 250.0 * (t * 2.1).cos(),
            )));
        if now >= stress.until {
            println!(
                "stress: {} frames, {} target-size changes, {} MSAA toggles, wgpu validation errors: {}",
                stress.frames,
                stress.resizes,
                stress.msaa_toggles,
                self.errors.load(Ordering::Relaxed)
            );
            self.stress = None;
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }
}

fn verdict(ok: bool) -> &'static str {
    if ok { "PASS" } else { "FAIL" }
}

impl eframe::App for SpinApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let now = Instant::now();
        let dt = now - self.last;
        self.last = now;
        let dt_s = dt.as_secs_f64();
        let dt_ms = dt_s * 1e3;
        self.spin += TURNTABLE_RAD_PER_S * dt_s;
        self.cpu_ms += (dt_ms - self.cpu_ms) * 0.05;
        if let Some(t) = egui_glue::with_renderer(&self.rs, |r| r.gpu_timings(self.view)).flatten()
        {
            self.gpu_ms += (f64::from(t.total_ms) - self.gpu_ms) * 0.05;
        }
        self.update_scene();

        if self.bench.is_none() {
            egui::Panel::right("spin_info").show(ui, |ui| self.side_panel(ui));
        }
        let view = self.view;
        let rs = self.rs.clone();
        let out = egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ui, |ui| Viewport { view }.show(ui, &rs))
            .inner;
        self.navigate(ui, &out.response);
        assert!(!out.lost, "the GPU device was lost");

        self.bench_step(ui, out.size_px, dt_ms);
        self.stress_step(ui, out.size_px);
        ui.ctx().request_repaint();
    }
}

/// Counts log records at Error level from wgpu (validation and the DX12 debug layer) and
/// prints warnings and errors from wgpu.
struct WgpuLogCounter {
    errors: Arc<AtomicUsize>,
}

impl log::Log for WgpuLogCounter {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        metadata.level() <= log::Level::Warn && metadata.target().starts_with("wgpu")
    }

    fn log(&self, record: &log::Record<'_>) {
        if self.enabled(record.metadata()) {
            eprintln!(
                "[{}] {}: {}",
                record.level(),
                record.target(),
                record.args()
            );
            if record.level() == log::Level::Error {
                self.errors.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    fn flush(&self) {}
}

fn measure_shaders(dxc_dll: Option<String>) {
    let mut compilers = vec![("FXC", wgpu::Dx12Compiler::Fxc)];
    #[cfg(feature = "static-dxc")]
    compilers.push(("StaticDxc", wgpu::Dx12Compiler::StaticDxc));
    if cfg!(not(feature = "static-dxc")) {
        println!("(built without --features static-dxc: no StaticDxc)");
    }
    if let Some(dxc_path) = dxc_dll {
        compilers.push(("DynamicDxc", wgpu::Dx12Compiler::DynamicDxc { dxc_path }));
    }
    for (name, compiler) in compilers {
        for round in 0..3 {
            let mut cfg = GpuConfig::from_env();
            cfg.optional_features |= amg_render::OPTIONAL_FEATURES;
            cfg.dx12_compiler = compiler.clone();
            let gpu = Gpu::new_blocking(&cfg).expect("open the device");
            let start = Instant::now();
            let mut renderer = Renderer::new(gpu, &RendererConfig::default()).expect("renderer");
            let core_total = start.elapsed().as_secs_f64() * 1e3;
            renderer.set_msaa_samples(1).expect("1 sample");
            let timings: Vec<String> = renderer
                .core_pipeline_timings()
                .iter()
                .map(|t| format!("{}={:.1}", t.label, t.ms))
                .collect();
            println!(
                "{name} round {round}: Renderer::new (core set at MSAA 4) {core_total:.1} ms; pipelines ms (mesh entries after the first set are the MSAA 1 set): {}",
                timings.join(" ")
            );
        }
    }
}

fn main() {
    let args = parse_args();
    if args.measure_shaders {
        measure_shaders(args.dxc_dll);
        return;
    }

    let errors = Arc::new(AtomicUsize::new(0));
    log::set_boxed_logger(Box::new(WgpuLogCounter {
        errors: Arc::clone(&errors),
    }))
    .expect("the first logger");
    log::set_max_level(log::LevelFilter::Warn);

    let mut gpu_config = GpuConfig::from_env();
    gpu_config.optional_features |= amg_render::OPTIONAL_FEATURES;
    if args.stress.is_some() {
        println!("instance flags: {:?}", gpu_config.instance_flags);
    }
    let gpu = Gpu::new_blocking(&gpu_config).expect("open the GPU device");
    let validation = Arc::clone(&errors);
    gpu.device().on_uncaptured_error(Arc::new(move |e| {
        validation.fetch_add(1, Ordering::Relaxed);
        eprintln!("wgpu validation error: {e}");
    }));

    let data = match &args.stl {
        Some(path) => read_stl(path),
        None => displaced_icosphere(args.tris),
    };
    let triangles = (data.indices.len() / 3) as u64;
    let bounds = Aabb3::from_points(
        data.positions
            .iter()
            .map(|p| data.origin + Vec3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))),
    );
    let target = bounds.center();
    let distance = bounds.diagonal() * 1.1;
    let upload_start = Instant::now();
    let mesh = amg_render::upload_mesh(&gpu, &data, &JobCtx::detached()).expect("upload the mesh");
    println!(
        "mesh: {triangles} triangles, {} vertices, {} MB on the GPU, uploaded in {:.1} ms",
        mesh.vertex_count(),
        mesh.gpu_bytes() / (1024 * 1024),
        upload_start.elapsed().as_secs_f64() * 1e3,
    );
    drop(data);

    let is_bench = args.bench.is_some();
    let mut wgpu_options = egui_wgpu::WgpuConfiguration {
        surface: egui_wgpu::SurfaceConfig::HIGH_THROUGHPUT,
        wgpu_setup: egui_wgpu::WgpuSetup::Existing(egui_wgpu::WgpuSetupExisting {
            instance: gpu.instance().clone(),
            adapter: gpu.adapter().clone(),
            device: gpu.device().clone(),
            queue: gpu.queue().clone(),
        }),
        ..Default::default()
    };
    if is_bench || args.stress.is_some() {
        // Uncapped, so the numbers measure the frame's cost rather than the monitor's refresh.
        wgpu_options.surface.present_mode = wgpu::PresentMode::AutoNoVsync;
    }
    let mut viewport = egui::ViewportBuilder::default()
        .with_inner_size([1280.0, 800.0])
        .with_title("amg-render spin");
    if is_bench {
        viewport = viewport
            .with_inner_size([BENCH_SIZE_PX[0] as f32, BENCH_SIZE_PX[1] as f32])
            .with_decorations(false)
            .with_position([0.0, 0.0]);
    }
    let native = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        wgpu_options,
        multisampling: 0,
        depth_buffer: 0,
        stencil_buffer: 0,
        viewport,
        ..Default::default()
    };

    let info = gpu.info().clone();
    let shared_gpu = Arc::clone(&gpu);
    let app_errors = Arc::clone(&errors);
    let stress = args.stress;
    let bench = args.bench;
    eframe::run_native(
        "amg-render spin",
        native,
        Box::new(move |cc| {
            let rs = cc
                .wgpu_render_state
                .clone()
                .expect("eframe runs on the wgpu renderer");
            // RND-01 Done-when 2: egui draws on the shared device.
            assert!(
                rs.device == *shared_gpu.device() && egui_glue::shares_device(&rs, &shared_gpu),
                "egui must share the Gpu device"
            );
            egui_glue::install(&rs, shared_gpu, &RendererConfig::default(), 1)?;
            let (view, instance) = egui_glue::with_renderer(&rs, |r| {
                let key = r.scene_mut().insert_mesh(mesh);
                let instance = r
                    .scene_mut()
                    .add_instance(Instance::new(key, Transform::IDENTITY));
                (r.create_view([1, 1]), instance)
            })
            .expect("install just ran");
            let now = Instant::now();
            Ok(Box::new(SpinApp {
                rs,
                view,
                instance,
                triangles,
                adapter: info.name.clone(),
                backend: format!("{:?}", info.backend),
                target,
                distance,
                azimuth: 0.6,
                elevation: 0.35,
                spin: 0.0,
                last: now,
                cpu_ms: 0.0,
                gpu_ms: 0.0,
                msaa4: true,
                flat: false,
                bench: bench.map(Bench::new),
                stress: stress.map(|secs| Stress {
                    until: now + Duration::from_secs_f64(secs),
                    start: now,
                    frames: 0,
                    msaa_toggles: 0,
                    resizes: 0,
                    next_toggle: now,
                    last_size: [0, 0],
                }),
                errors: app_errors,
            }))
        }),
    )
    .expect("the window runs");

    let total = errors.load(Ordering::Relaxed);
    if args.stress.is_some() && total > 0 {
        std::process::exit(1);
    }
}
