//! CMP-02 grid tests (S15 Done-when 1-5 and 7; 6 is `benches/grid.rs`).
//!
//! GPU tests are named `*gpu*` (nextest test-group `gpu`, serial); they skip without an
//! adapter and fail under `AMG_REQUIRE_GPU=1`. `AMG_WGPU_BACKEND=warp` runs them on WARP.
//! Golden values live in `tests/golden/compute/brick_sphere.toml`; `AMG_BLESS=1` rewrites it.

#![cfg(feature = "gpu")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]
#![allow(clippy::float_cmp, reason = "bit-exact comparisons are the point")]

#[macro_use]
mod common;

use std::sync::Arc;

use amg_compute::gpu::{ComputeCtx, GpuVec, Kernel};
use amg_compute::grid::{
    BrickGrid, BrickLayout, Fill, GpuBrickGrid, GpuEncoding, GridValue, Slot, TileBox,
    brick_grid_wgsl,
};
use amg_compute::prim::Primitives;
use amg_compute::testprob::sphere_bricks;
use amg_compute::{ComputeError, f16};
use amg_jobs::JobCtx;
use amg_jobs::gpu::Gpu;

/// Done-when 1: sphere r = 50 mm at 0.5 mm cells = 100 cells; band ±4 cells.
const R1: f64 = 100.0;
/// Done-when 5: sphere r = 100 mm at 0.1 mm cells = 1 000 cells.
const R5: f64 = 1000.0;
const BAND: f64 = 4.0;

/// Deterministic generator (splitmix64).
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    /// Uniform in [0, 1).
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

/// `n` points: half uniform in the grid's box, half in the band shell (|d| ≤ band). f32
/// coordinates, so the CPU and the GPU see the same points.
fn points(g: &BrickGrid<impl GridValue>, radius: f64, n: usize, seed: u64) -> Vec<[f32; 3]> {
    let (lo, hi) = g.tile_box().cell_box();
    let mut rng = Rng(seed);
    (0..n)
        .map(|i| {
            if i % 2 == 0 {
                [0, 1, 2].map(|a| (f64::from(lo[a]) + rng.unit() * f64::from(hi[a] - lo[a])) as f32)
            } else {
                // Random direction (normalised Gaussian-free: rejection in the unit ball).
                let dir = loop {
                    let v = [0, 1, 2].map(|_| rng.unit() * 2.0 - 1.0);
                    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
                    if l > 1e-3 && l <= 1.0 {
                        break v.map(|c| c / l);
                    }
                };
                let r = radius + (rng.unit() * 2.0 - 1.0) * BAND;
                dir.map(|c| (c * r) as f32)
            }
        })
        .collect()
}

fn distance(p: [f64; 3], radius: f64) -> f64 {
    (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() - radius
}

// ───────────── golden file ─────────────

#[derive(Debug, Default, serde::Serialize, serde::Deserialize, PartialEq)]
struct SphereGolden {
    radius_cells: f64,
    band_cells: f64,
    brick_count: u64,
    tile_nodes: u64,
    index_bytes: u64,
    brick_bytes: u64,
    /// Measured; the check is `max_band_error <= max_band_error_bound`.
    max_band_error: Option<f64>,
    max_band_error_bound: Option<f64>,
    index_fraction_bound: Option<f64>,
}

const GOLDEN_HEADER: &str = "\
# CMP-02 golden: narrow-band sphere SDF brick grids (`testprob::sphere_bricks::<f16>`).
# Checked by crates/amg-compute/tests/grid.rs (golden_*). Counts must match exactly; the
# measured band error must stay within its bound. Rewrite with AMG_BLESS=1.
";

fn golden_path() -> std::path::PathBuf {
    amg_testkit::golden_dir()
        .join("compute")
        .join("brick_sphere.toml")
}

fn check_golden(name: &str, got: &SphereGolden) {
    let path = golden_path();
    let text = std::fs::read_to_string(&path).unwrap_or_default();
    let mut file: toml::Table = toml::from_str(&text).expect("golden file parses");
    if std::env::var_os("AMG_BLESS").is_some_and(|v| v == "1") {
        assert!(
            std::env::var_os("CI").is_none(),
            "blessing is refused in CI"
        );
        file.insert(
            name.to_owned(),
            toml::Value::try_from(got).expect("serialisable"),
        );
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        let body = toml::to_string(&file).unwrap();
        std::fs::write(&path, format!("{GOLDEN_HEADER}\n{body}")).unwrap();
        return;
    }
    let want: SphereGolden = file
        .get(name)
        .unwrap_or_else(|| panic!("golden case `{name}` missing in {}", path.display()))
        .clone()
        .try_into()
        .expect("golden case parses");
    assert_eq!(
        (
            got.brick_count,
            got.tile_nodes,
            got.index_bytes,
            got.brick_bytes
        ),
        (
            want.brick_count,
            want.tile_nodes,
            want.index_bytes,
            want.brick_bytes
        ),
        "{name}: counts differ from {}",
        path.display()
    );
    if let (Some(err), Some(bound)) = (got.max_band_error, want.max_band_error_bound) {
        assert!(err <= bound, "{name}: band error {err} above {bound}");
    }
}

// ───────────── Done-when 1 ─────────────

/// Sphere r = 50 at cell 0.5 (100 cells) in `BrickGrid<f16>`: trilinear error ≤ 0.01 cell in
/// the band, sign correct everywhere in the box.
#[test]
fn golden_sphere_r50_cell05_error_and_sign() {
    let g = sphere_bricks::<f16>(R1, BAND);
    assert_eq!(g.layout(), BrickLayout::CORNERS);
    // Band error at 1 M points (half in the shell) plus a lattice over the whole box.
    let mut max_err = 0.0f64;
    let mut band_points = 0usize;
    let mut sign_checked = 0usize;
    let mut check = |p: [f64; 3]| {
        let d = distance(p, R1);
        let v = g.trilinear(p);
        if d.abs() <= BAND {
            band_points += 1;
            max_err = max_err.max((v - d).abs());
        }
        if d.abs() > 0.01 {
            sign_checked += 1;
            assert_eq!(v < 0.0, d < 0.0, "sign at {p:?}: value {v}, distance {d}");
        }
    };
    for p in points(&g, R1, 1_000_000, 1) {
        check(p.map(f64::from));
    }
    let (lo, hi) = g.tile_box().cell_box();
    let mut x = f64::from(lo[0]) + 0.37;
    while x < f64::from(hi[0]) {
        let mut y = f64::from(lo[1]) + 0.21;
        while y < f64::from(hi[1]) {
            let mut z = f64::from(lo[2]) + 0.53;
            while z < f64::from(hi[2]) {
                check([x, y, z]);
                z += 2.0;
            }
            y += 2.0;
        }
        x += 2.0;
    }
    eprintln!(
        "sphere r = {R1} cells: {} bricks, {} tile nodes, index {} B, bricks {} B; \
         max band error {max_err:.5} cell over {band_points} points; sign checked at {sign_checked}",
        g.brick_count(),
        g.tile_node_count(),
        g.index_bytes(),
        g.brick_bytes()
    );
    assert!(band_points > 500_000);
    assert!(max_err <= 0.01, "band error {max_err} cell");
    check_golden(
        "sphere_r50_cell0_5",
        &SphereGolden {
            radius_cells: R1,
            band_cells: BAND,
            brick_count: g.brick_count() as u64,
            tile_nodes: g.tile_node_count() as u64,
            index_bytes: g.index_bytes() as u64,
            brick_bytes: g.brick_bytes() as u64,
            max_band_error: Some(max_err),
            max_band_error_bound: Some(0.01),
            index_fraction_bound: None,
        },
    );
}

// ───────────── Done-when 5 ─────────────

/// Sphere r = 100 at cell 0.1 (1 000 cells): index (root + tiles) ≤ 10 % of the brick bytes.
#[test]
fn golden_sphere_r100_cell01_bytes() {
    let g = sphere_bricks::<f16>(R5, BAND);
    let index = g.index_bytes() as f64;
    let bricks = g.brick_bytes() as f64;
    eprintln!(
        "sphere r = {R5} cells: {} bricks, {} tile nodes, {} root entries; index {} B \
         ({:.2} MB), bricks {} B ({:.1} MB): index = {:.2} % of brick bytes; all heap {} B",
        g.brick_count(),
        g.tile_node_count(),
        g.tile_box().tile_count(),
        g.index_bytes(),
        index / 1e6,
        g.brick_bytes(),
        bricks / 1e6,
        100.0 * index / bricks,
        g.bytes()
    );
    assert!(index <= 0.1 * bricks);
    check_golden(
        "sphere_r100_cell0_1",
        &SphereGolden {
            radius_cells: R5,
            band_cells: BAND,
            brick_count: g.brick_count() as u64,
            tile_nodes: g.tile_node_count() as u64,
            index_bytes: g.index_bytes() as u64,
            brick_bytes: g.brick_bytes() as u64,
            max_band_error: None,
            max_band_error_bound: None,
            index_fraction_bound: Some(0.1),
        },
    );
}

// ───────────── Done-when 4 ─────────────

/// Rebuilds `g` slot by slot in the order given by `order` (a permutation of the candidate
/// indices of its tile box).
fn rebuild<T: GridValue>(g: &BrickGrid<T>, order: &[u64]) -> BrickGrid<T> {
    let tb = g.tile_box();
    let fill = [g.fill_value(Fill::Fill0), g.fill_value(Fill::Fill1)];
    let mut h = BrickGrid::new(g.layout(), fill, tb.cell_box()).unwrap();
    for &i in order {
        let b = tb.candidate_brick(i);
        match g.lookup(b) {
            Slot::Brick(_, data) => {
                h.insert(b, data).unwrap();
            }
            Slot::Fill(f) => h.set_fill(b, f).unwrap(),
        }
    }
    h
}

#[test]
fn content_hash_is_insertion_order_independent() {
    let g = sphere_bricks::<f16>(R1 / 2.0, BAND);
    let n = g.tile_box().candidate_count();
    let forward: Vec<u64> = (0..n).collect();
    let backward: Vec<u64> = (0..n).rev().collect();
    let mut shuffled = forward.clone();
    let mut rng = Rng(7);
    for i in (1..shuffled.len()).rev() {
        shuffled.swap(i, rng.below(i as u64 + 1) as usize);
    }
    let mut a = rebuild(&g, &backward);
    let mut b = rebuild(&g, &shuffled);
    assert_ne!(
        a, b,
        "different insertion orders give different storage orders"
    );
    a.canonicalize();
    b.canonicalize();
    assert_eq!(a.content_hash(), b.content_hash());
    assert_eq!(a.content_hash(), g.content_hash());
    assert_eq!(a, b);
    assert_eq!(a, g);
    let mut c = rebuild(&g, &forward);
    c.canonicalize();
    assert_eq!(c, g);
    eprintln!(
        "{} bricks; content hash {}",
        g.brick_count(),
        g.content_hash()
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect::<String>()
    );
}

// ───────────── Done-when 7 ─────────────

#[test]
fn root_over_2_pow_24_tiles_is_shape() {
    let edge = 256 * 64;
    let ok = BrickGrid::<u8>::new(BrickLayout::CELLS, [0, 1], ([0; 3], [edge; 3]));
    assert!(ok.is_ok(), "exactly 2^24 tiles is allowed");
    for cell_box in [
        ([0, 0, 0], [edge + 1, edge, edge]),
        ([-1, 0, 0], [edge, edge, edge]),
        ([i32::MIN; 3], [i32::MAX; 3]),
    ] {
        let err =
            BrickGrid::<f16>::new(BrickLayout::CORNERS, [f16::ZERO; 2], cell_box).unwrap_err();
        assert!(matches!(err, ComputeError::Shape(_)), "{cell_box:?}: {err}");
    }
    assert!(matches!(
        BrickGrid::<f32>::new(BrickLayout { apron: 2 }, [0.0; 2], ([0; 3], [8; 3])),
        Err(ComputeError::Shape(_))
    ));
}

// ───────────── GPU helpers ─────────────

fn job() -> JobCtx {
    JobCtx::detached()
}

fn read(gpu: &Gpu, v: &GpuVec<u32>) -> Vec<u32> {
    if v.is_empty() {
        return Vec::new();
    }
    v.read(gpu).wait().unwrap()
}

/// Evaluates `bg_trilinear` at `pts` on the GPU.
fn gpu_trilinear(gpu: &Arc<Gpu>, grid: &GpuBrickGrid, pts: &[[f32; 3]]) -> Vec<f32> {
    let src = format!(
        "{}
@group(0) @binding(0) var<storage, read> pts: array<f32>;
@group(0) @binding(1) var<storage, read_write> out: array<f32>;
@compute @workgroup_size(WG)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {{
    let i = amg_linear_id(gid, nwg);
    if (i >= arrayLength(&out)) {{ return; }}
    out[i] = bg_trilinear(vec3<f32>(pts[3u * i], pts[3u * i + 1u], pts[3u * i + 2u]));
}}
",
        brick_grid_wgsl(1, 0, grid.encoding(), false)
    );
    let k = Kernel::new(gpu, "bg_trilinear test", &src, "main", &[]).unwrap();
    let flat: Vec<f32> = pts.iter().flatten().copied().collect();
    let p = GpuVec::from_slice(gpu, "points", &flat).unwrap();
    let out = GpuVec::<f32>::zeroed(gpu, "values", pts.len() as u64).unwrap();
    let g0 = k.bind_group(gpu, 0, &[p.binding(), out.binding()]);
    let g1 = k.bind_group(gpu, 1, &grid.bindings());
    let mut cx = ComputeCtx::new(gpu.clone(), "bg_trilinear test");
    cx.dispatch_1d(&k, &[&g0, &g1], pts.len() as u64);
    cx.finish(&job()).unwrap();
    out.read(gpu).wait().unwrap()
}

/// Done-when 2 for one value type.
fn check_gpu_trilinear<T: GridValue>(gpu: &Arc<Gpu>) {
    let g = sphere_bricks::<T>(R1, BAND);
    let grid = GpuBrickGrid::upload(gpu, &g).unwrap();
    let pts = points(&g, R1, 1 << 20, 2);
    let got = gpu_trilinear(gpu, &grid, &pts);
    let mut max_diff = 0.0f64;
    let mut worst = [0.0f32; 3];
    for (p, &v) in pts.iter().zip(&got) {
        let want = g.trilinear(p.map(f64::from));
        let diff = (f64::from(v) - want).abs();
        if diff > max_diff {
            max_diff = diff;
            worst = *p;
        }
    }
    eprintln!(
        "{:?}: {} points, max |bg_trilinear - trilinear| = {max_diff:.3e} cell at {worst:?} ({})",
        T::ENCODING,
        pts.len(),
        gpu.info().name
    );
    assert!(max_diff <= 1e-3, "{max_diff} cell at {worst:?}");
}

#[test]
fn gpu_trilinear_matches_cpu() {
    let Some(gpu) = common::gpu() else { return };
    check_gpu_trilinear::<f16>(&gpu);
    check_gpu_trilinear::<f32>(&gpu);
}

/// `bg_sample` and the apron-0 path of `bg_trilinear` on a `CELLS` grid of u8.
#[test]
fn gpu_cells_layout_u8_matches_cpu() {
    let Some(gpu) = common::gpu() else { return };
    let g = sphere_bricks::<f32>(40.0, BAND);
    // Re-pack into a CELLS grid of u8: value = clamp(d + 128).
    let tb = g.tile_box();
    let mut c: BrickGrid<u8> =
        BrickGrid::new(BrickLayout::CELLS, [140, 100], tb.cell_box()).unwrap();
    for (b, _) in g.bricks() {
        let o = b.min_cell();
        let data: Vec<u8> = (0..512)
            .map(|k| {
                let p = [o[0] + k % 8, o[1] + (k / 8) % 8, o[2] + k / 64].map(f64::from);
                u8::from_f32((distance(p, 40.0) * 4.0 + 128.0) as f32)
            })
            .collect();
        c.insert(b, &data).unwrap();
    }
    c.set_tile_fill(tb.tile_of_root(0), Fill::Fill1).unwrap();
    c.canonicalize();
    let grid = GpuBrickGrid::upload(&gpu, &c).unwrap();
    let pts = points(&c, 40.0, 1 << 16, 3);
    let got = gpu_trilinear(&gpu, &grid, &pts);
    let mut max_diff = 0.0f64;
    for (p, &v) in pts.iter().zip(&got) {
        let want = c.trilinear(p.map(f64::from));
        max_diff = max_diff.max((f64::from(v) - want).abs());
    }
    eprintln!("CELLS u8: max diff {max_diff:.3e}");
    assert!(max_diff <= 1e-3, "{max_diff}");
}

// ───────────── Done-when 3 ─────────────

/// Round trip `upload` → `download` is bit-exact (also for NaN payloads).
fn check_round_trip<T: GridValue>(gpu: &Arc<Gpu>, g: &BrickGrid<T>) {
    let grid = GpuBrickGrid::upload(gpu, g).unwrap();
    assert_eq!(grid.brick_count() as usize, g.brick_count());
    let back: BrickGrid<T> = grid.download(gpu, &job()).unwrap();
    assert!(back == *g, "{:?} round trip differs", T::ENCODING);
    assert_eq!(back.content_hash(), g.content_hash());
    eprintln!(
        "{:?} {:?}: {} bricks round-trip bit-exactly",
        T::ENCODING,
        g.layout(),
        g.brick_count()
    );
}

#[test]
fn gpu_upload_download_round_trips_bit_exactly() {
    let Some(gpu) = common::gpu() else { return };
    let g = sphere_bricks::<f16>(R1 / 2.0, BAND);
    // Random bit patterns (NaNs and infinities included) in a non-canonical order.
    let mut rng = Rng(11);
    let mut bits = g.map(
        |v| f16::from_bits(v.to_bits().wrapping_mul(0x9e37) ^ 0x5a5a),
        [f16::from_bits(0x7e01), f16::NEG_INFINITY],
    );
    bits.remove(g.bricks().next().unwrap().0, Fill::Fill1);
    check_round_trip(&gpu, &g);
    check_round_trip(&gpu, &bits);
    check_round_trip(
        &gpu,
        &g.map(
            |v| f32::from_bits(v.to_f32().to_bits().wrapping_mul(0x9e37_79b9) ^ 0xa5a5_a5a5),
            [1.5, -2.5],
        ),
    );
    check_round_trip(
        &gpu,
        &g.map(|v| u8::from_f32(v.to_f32() * 8.0 + 100.0), [3, 250]),
    );
    let mut cells: BrickGrid<u8> =
        BrickGrid::new(BrickLayout::CELLS, [1, 2], ([-70, 0, 5], [10, 9, 200])).unwrap();
    for i in 0..300 {
        let b = cells
            .tile_box()
            .candidate_brick(rng.below(cells.tile_box().candidate_count()));
        let v = (i % 251) as u8;
        cells.insert(b, &[v; 512]).unwrap();
    }
    check_round_trip(&gpu, &cells);
    // An empty grid.
    check_round_trip(
        &gpu,
        &BrickGrid::<f32>::new(BrickLayout::CORNERS, [0.0, 1.0], ([0; 3], [1; 3])).unwrap(),
    );
}

/// A random mask over the candidates of `tb` and a per-tile fill, plus the CPU grid in
/// canonical form with zero payloads.
fn random_build<T: GridValue>(
    tb: TileBox,
    layout: BrickLayout,
    fill: [f32; 2],
    seed: u64,
) -> (Vec<u32>, Vec<u32>, BrickGrid<T>) {
    let mut rng = Rng(seed);
    let n = tb.candidate_count() as usize;
    let mut mask = vec![0u32; n];
    // Clustered: most tiles empty, a few dense, the rest sparse.
    for t in 0..tb.tile_count() as usize {
        let density = match rng.below(4) {
            0 => 0.0,
            1 => 0.9,
            _ => 0.02,
        };
        for k in 0..512 {
            if rng.unit() < density {
                mask[t * 512 + k] = 1 + rng.below(1000) as u32; // any non-zero marks
            }
        }
    }
    let unmarked: Vec<u32> = (0..tb.tile_count()).map(|_| rng.below(2) as u32).collect();
    let fills = fill.map(T::from_f32);
    let mut g = BrickGrid::new(layout, fills, tb.cell_box()).unwrap();
    for (t, &u) in unmarked.iter().enumerate() {
        if u != 0 {
            g.set_tile_fill(tb.tile_of_root(t as u32), Fill::Fill1)
                .unwrap();
        }
    }
    // Insert in a scrambled order; canonicalize must sort it out.
    let zero = vec![T::from_f32(0.0); layout.samples()];
    let mut marked: Vec<usize> = (0..n).filter(|&i| mask[i] != 0).collect();
    for i in (1..marked.len()).rev() {
        marked.swap(i, rng.below(i as u64 + 1) as usize);
    }
    for i in marked {
        g.insert(tb.candidate_brick(i as u64), &zero).unwrap();
    }
    g.canonicalize();
    (mask, unmarked, g)
}

/// Done-when 3: `allocate` is bit-identical over 3 runs and matches the canonical CPU grid.
fn check_allocate<T: GridValue>(gpu: &Arc<Gpu>, prim: &Primitives, layout: BrickLayout) {
    let cell_box = ([-100, -70, -30], [90, 60, 140]);
    let tb = TileBox::covering(cell_box).unwrap();
    let fill = [3.0, -3.0];
    let (mask, unmarked, want) = random_build::<T>(tb, layout, fill, 21);
    let mask_gpu = GpuVec::from_slice(gpu, "mask", &mask).unwrap();
    let unmarked_gpu = GpuVec::from_slice(gpu, "unmarked", &unmarked).unwrap();
    let mut runs = Vec::new();
    for _ in 0..3 {
        let mut cx = ComputeCtx::new(gpu.clone(), "allocate test");
        let grid = GpuBrickGrid::allocate(
            &mut cx,
            prim,
            layout,
            T::ENCODING,
            cell_box,
            &mask_gpu,
            fill,
            &unmarked_gpu,
        )
        .unwrap();
        cx.finish(&job()).unwrap();
        let buffers: Vec<Vec<u32>> = grid.buffers().iter().map(|b| read(gpu, b)).collect();
        let got: BrickGrid<T> = grid.download(gpu, &job()).unwrap();
        runs.push((buffers, got, grid.brick_count()));
    }
    assert_eq!(runs[0].0, runs[1].0, "run 1 vs 2");
    assert_eq!(runs[0].0, runs[2].0, "run 1 vs 3");
    assert_eq!(runs[0].2 as usize, want.brick_count());
    assert!(
        runs[0].1 == want,
        "{:?} {layout:?}: GPU allocate differs from canonical CPU",
        T::ENCODING
    );
    // The canonical CPU grid uploads to the same buffers.
    let up = GpuBrickGrid::upload(gpu, &want).unwrap();
    let up_buffers: Vec<Vec<u32>> = up.buffers().iter().map(|b| read(gpu, b)).collect();
    assert_eq!(up_buffers[0], runs[0].0[0], "root");
    assert_eq!(
        up_buffers[1][..],
        runs[0].0[1][..up_buffers[1].len()],
        "tiles"
    );
    eprintln!(
        "{:?} {layout:?}: {} tiles, {} nodes, {} bricks; 3 runs bit-identical, order = canonicalize",
        T::ENCODING,
        tb.tile_count(),
        want.tile_node_count(),
        want.brick_count()
    );
}

#[test]
fn gpu_allocate_is_deterministic_and_canonical() {
    let Some(gpu) = common::gpu() else { return };
    let prim = Primitives::new(&gpu).unwrap();
    check_allocate::<f16>(&gpu, &prim, BrickLayout::CORNERS);
    check_allocate::<f32>(&gpu, &prim, BrickLayout::CORNERS);
    check_allocate::<u8>(&gpu, &prim, BrickLayout::CORNERS);
    check_allocate::<f16>(&gpu, &prim, BrickLayout::CELLS);
    check_allocate::<u8>(&gpu, &prim, BrickLayout::CELLS);
}

#[test]
fn gpu_allocate_rejects_bad_shapes() {
    let Some(gpu) = common::gpu() else { return };
    let prim = Primitives::new(&gpu).unwrap();
    let mut cx = ComputeCtx::new(gpu.clone(), "allocate shapes");
    let small = GpuVec::<u32>::zeroed(&gpu, "mask", 10).unwrap();
    let one = GpuVec::<u32>::zeroed(&gpu, "unmarked", 1).unwrap();
    let alloc = |cx: &mut ComputeCtx, cell_box, mask: &GpuVec<u32>| {
        GpuBrickGrid::allocate(
            cx,
            &prim,
            BrickLayout::CORNERS,
            GpuEncoding::F16Packed,
            cell_box,
            mask,
            [0.0; 2],
            &one,
        )
    };
    let edge = 256 * 64;
    let err = alloc(&mut cx, ([0; 3], [edge + 1, edge, edge]), &small).unwrap_err();
    assert!(matches!(err, ComputeError::Shape(_)), "{err}");
    let err = alloc(&mut cx, ([0; 3], [64; 3]), &small).unwrap_err();
    assert!(matches!(err, ComputeError::Shape(_)), "{err}");
    // No marks at all: no nodes, no bricks.
    let mask = GpuVec::<u32>::zeroed(&gpu, "mask", 512).unwrap();
    let grid = alloc(&mut cx, ([0; 3], [64; 3]), &mask).unwrap();
    assert_eq!((grid.brick_count(), grid.tile_node_count()), (0, 0));
    let back: BrickGrid<f16> = grid.download(&gpu, &job()).unwrap();
    assert_eq!(back.brick_count(), 0);
    assert!(matches!(
        grid.download::<f32>(&gpu, &job()),
        Err(ComputeError::Shape(_))
    ));
}

/// The producer path: `allocate`, then a kernel writes every sample with `bg_store` (packed
/// encodings: neighbouring samples from different threads), then `download`.
fn check_store<T: GridValue>(gpu: &Arc<Gpu>, prim: &Primitives) {
    let layout = BrickLayout::CORNERS;
    let cell_box = ([-100, -70, -30], [90, 60, 140]);
    let tb = TileBox::covering(cell_box).unwrap();
    let (mask, unmarked, mut want) = random_build::<T>(tb, layout, [1.0, 2.0], 33);
    let mask_gpu = GpuVec::from_slice(gpu, "mask", &mask).unwrap();
    let unmarked_gpu = GpuVec::from_slice(gpu, "unmarked", &unmarked).unwrap();
    let mut cx = ComputeCtx::new(gpu.clone(), "bg_store test");
    let grid = GpuBrickGrid::allocate(
        &mut cx,
        prim,
        layout,
        T::ENCODING,
        cell_box,
        &mask_gpu,
        [1.0, 2.0],
        &unmarked_gpu,
    )
    .unwrap();
    // One thread per (candidate, sample); marked candidates write
    // value(b, local) = (7 x + 13 y + 29 z + local) mod 200, exact in f16, f32 and u8.
    let src = format!(
        "{}
@group(1) @binding(0) var<uniform> n_cand: vec4<u32>;
@compute @workgroup_size(WG)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {{
    let i = amg_linear_id(gid, nwg);
    let n = bg_meta.spa * bg_meta.spa * bg_meta.spa;
    if (i >= n_cand.x * n) {{ return; }}
    let b = bg_candidate_brick(i / n);
    let code = bg_slot(b);
    if (code >= BG_FILL1) {{ return; }}
    let local = i % n;
    let h = ((b.x * 7 + b.y * 13 + b.z * 29) % 200 + 200) % 200;
    bg_store(code, local, f32((u32(h) + local) % 200u));
}}
",
        brick_grid_wgsl(0, 0, T::ENCODING, true)
    );
    let k = Kernel::new(gpu, "bg_store test", &src, "main", &[]).unwrap();
    let [meta, root, tiles, _] = grid.bindings();
    let bind = k.bind_group(gpu, 0, &[meta, root, tiles, grid.data_mut_binding()]);
    let n_cand = tb.candidate_count() as u32;
    let slot = cx.uniform(&[n_cand, 0, 0, 0]);
    let bind1 = k.bind_group(gpu, 1, &[cx.uniform_binding(slot)]);
    let n = u64::from(n_cand) * layout.samples() as u64;
    cx.dispatch_1d(&k, &[&bind, &bind1], n);
    cx.finish(&job()).unwrap();
    let got: BrickGrid<T> = grid.download(gpu, &job()).unwrap();
    let coords: Vec<_> = want.bricks().map(|(b, _)| b).collect();
    for b in coords {
        let h = (b.0[0] * 7 + b.0[1] * 13 + b.0[2] * 29).rem_euclid(200) as usize;
        for (local, v) in want.brick_mut(b).unwrap().iter_mut().enumerate() {
            *v = T::from_f32(((h + local) % 200) as f32);
        }
    }
    assert!(got == want, "{:?}: bg_store result differs", T::ENCODING);
    eprintln!(
        "{:?}: bg_store wrote the samples of {} bricks ({n} threads)",
        T::ENCODING,
        grid.brick_count()
    );
}

#[test]
fn gpu_bg_store_fills_allocated_bricks() {
    let Some(gpu) = common::gpu() else { return };
    let prim = Primitives::new(&gpu).unwrap();
    check_store::<f16>(&gpu, &prim);
    check_store::<f32>(&gpu, &prim);
    check_store::<u8>(&gpu, &prim);
}
