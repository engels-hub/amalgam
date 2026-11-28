//! `GpuBrickGrid`: the GPU mirror of a brick grid.

use std::fmt;

use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, wgpu};

use super::wgsl;
use super::{BrickGrid, BrickLayout, FILL1_CODE, GpuEncoding, GridValue, TILE_SLOTS, TileBox};
use crate::gpu::{ComputeCtx, GpuVec, Kernel};
use crate::prim::{Primitives, ScanKind};
use crate::{ComputeError, ComputeResult};

/// The `BgMeta` uniform of `brick_grid_wgsl` (48 bytes, WGSL uniform layout).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
struct Meta {
    tile_origin: [i32; 3],
    spa: u32,
    tile_dims: [u32; 3],
    stride: u32,
    fill: [f32; 2],
    apron: u32,
    brick_count: u32,
}

/// GPU mirror. Bindings: meta (uniform), root, tiles, data (storage, read-only in user kernels).
///
/// The layout is the CPU grid's (module docs): `root` has one `u32` per tile, `tiles` 512 per
/// tile node, `data` `enc.brick_words(layout)` words per brick. Values are read and written
/// through the WGSL of [`brick_grid_wgsl`](super::brick_grid_wgsl).
pub struct GpuBrickGrid {
    meta: wgpu::Buffer,
    root: GpuVec<u32>,
    tiles: GpuVec<u32>,
    data: GpuVec<u32>,
    layout: BrickLayout,
    encoding: GpuEncoding,
    tile_box: TileBox,
    fill: [f32; 2],
    node_count: u32,
    brick_count: u32,
}

impl fmt::Debug for GpuBrickGrid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GpuBrickGrid")
            .field("layout", &self.layout)
            .field("encoding", &self.encoding)
            .field("tile_box", &self.tile_box)
            .field("fill", &self.fill)
            .field("node_count", &self.node_count)
            .field("brick_count", &self.brick_count)
            .finish_non_exhaustive()
    }
}

fn meta_buffer(gpu: &Gpu, meta: &Meta) -> wgpu::Buffer {
    let buffer = gpu.device().create_buffer(&wgpu::BufferDescriptor {
        label: Some("brick grid meta"),
        size: size_of::<Meta>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    gpu.queue()
        .write_buffer(&buffer, 0, bytemuck::bytes_of(meta));
    buffer
}

fn u32_len(what: &str, len: u64) -> ComputeResult<u32> {
    u32::try_from(len).map_err(|_| {
        ComputeError::Shape(format!("{what}: {len} elements exceed the u32 index range"))
    })
}

impl GpuBrickGrid {
    #[allow(clippy::too_many_arguments, reason = "internal constructor")]
    fn assemble(
        gpu: &Gpu,
        root: GpuVec<u32>,
        tiles: GpuVec<u32>,
        data: GpuVec<u32>,
        layout: BrickLayout,
        encoding: GpuEncoding,
        tile_box: TileBox,
        fill: [f32; 2],
        node_count: u32,
        brick_count: u32,
    ) -> Self {
        let meta = Meta {
            tile_origin: tile_box.origin,
            spa: layout.samples_per_axis(),
            tile_dims: tile_box.dims,
            stride: encoding.brick_words(layout),
            fill,
            apron: layout.apron,
            brick_count,
        };
        GpuBrickGrid {
            meta: meta_buffer(gpu, &meta),
            root,
            tiles,
            data,
            layout,
            encoding,
            tile_box,
            fill,
            node_count,
            brick_count,
        }
    }

    /// Uploads `g` as is (canonicalize first for the canonical order). The encoding is
    /// `T::ENCODING`.
    ///
    /// # Errors
    /// [`ComputeError::TooLarge`] when a buffer exceeds the device's binding limit.
    pub fn upload<T: GridValue>(gpu: &Gpu, g: &BrickGrid<T>) -> ComputeResult<Self> {
        let enc = T::ENCODING;
        let stride = enc.brick_words(g.layout) as usize;
        let spb = g.layout.samples();
        let bricks = g.brick_count();
        let mut words = vec![0u32; bricks * stride];
        {
            let bytes: &mut [u8] = bytemuck::cast_slice_mut(&mut words);
            let brick_bytes = spb * size_of::<T>();
            for (id, (_, samples)) in g.bricks().enumerate() {
                bytes[id * stride * 4..][..brick_bytes]
                    .copy_from_slice(bytemuck::cast_slice(samples));
            }
        }
        let root = GpuVec::from_slice(gpu, "brick grid root", &g.root)?;
        let tiles = GpuVec::from_slice(gpu, "brick grid tiles", &g.tiles)?;
        let data = GpuVec::from_slice(gpu, "brick grid data", &words)?;
        Ok(Self::assemble(
            gpu,
            root,
            tiles,
            data,
            g.layout,
            enc,
            g.tile_box,
            g.fill.map(T::to_f32),
            g.tile_node_count() as u32,
            bricks as u32,
        ))
    }

    /// Reads the grid back (blocking: job threads only). Bricks and nodes keep their GPU order.
    ///
    /// # Errors
    /// [`ComputeError::Shape`] when `T::ENCODING` is not this grid's encoding;
    /// [`ComputeError::Cancelled`] when `ctx` is cancelled; [`ComputeError::Gpu`] when the
    /// device is lost.
    pub fn download<T: GridValue>(&self, gpu: &Gpu, ctx: &JobCtx) -> ComputeResult<BrickGrid<T>> {
        if T::ENCODING != self.encoding {
            return Err(ComputeError::Shape(format!(
                "downloading a {:?} grid as {:?}",
                self.encoding,
                T::ENCODING
            )));
        }
        ctx.checkpoint()?;
        let read = |v: &GpuVec<u32>, len: u64| -> ComputeResult<Vec<u32>> {
            if len == 0 {
                return Ok(Vec::new());
            }
            Ok(v.read_range(gpu, 0..len).wait()?)
        };
        let root = read(&self.root, self.root.len())?;
        let tiles = read(
            &self.tiles,
            u64::from(self.node_count) * u64::from(TILE_SLOTS),
        )?;
        ctx.checkpoint()?;
        let stride = self.encoding.brick_words(self.layout) as usize;
        let words = read(&self.data, (self.brick_count as usize * stride) as u64)?;
        let spb = self.layout.samples();
        let bytes: &[u8] = bytemuck::cast_slice(&words);
        let mut data: Vec<T> = Vec::with_capacity(self.brick_count as usize * spb);
        for brick in bytes.chunks_exact((stride * 4).max(1)) {
            // Bricks start at whole words, so the slice is aligned for f16, f32 and u8.
            data.extend_from_slice(bytemuck::cast_slice::<u8, T>(
                &brick[..spb * size_of::<T>()],
            ));
        }
        Ok(BrickGrid::from_parts(
            self.layout,
            self.fill.map(T::from_f32),
            self.tile_box,
            root,
            tiles,
            data,
        ))
    }

    /// GPU-side build: `mask[i] != 0` marks candidate brick `i` of `cell_box` (index
    /// [`TileBox::candidate_index`]: tile in root order, then Morton order inside the tile;
    /// `mask.len()` = `TileBox::candidate_count`). Allocates tile nodes and bricks by scan
    /// (deterministic, the same order as `BrickGrid::canonicalize`): a tile gets a node iff one
    /// of its candidates is marked. Payloads are zero until the caller's kernel writes them
    /// (`bg_store`); unmarked slots and tiles get `fill_of_unmarked[tile]` (per root entry:
    /// 0 = Fill0, otherwise Fill1).
    ///
    /// Submits `cx`, waits for the two allocation counts (blocking: job threads only), records
    /// the index writes and submits again; kernels recorded into `cx` afterwards see the index.
    ///
    /// # Errors
    /// [`ComputeError::Shape`] for a bad box or apron, wrong `mask` / `fill_of_unmarked`
    /// lengths, or more candidates than `u32` indexes; [`ComputeError::TooLarge`] when a buffer
    /// exceeds the binding limit; [`ComputeError::Gpu`] when the device is lost.
    #[allow(clippy::too_many_arguments, reason = "the S15 interface")]
    pub fn allocate(
        cx: &mut ComputeCtx,
        prim: &Primitives,
        layout: BrickLayout,
        enc: GpuEncoding,
        cell_box: ([i32; 3], [i32; 3]),
        mask: &GpuVec<u32>,
        fill: [f32; 2],
        fill_of_unmarked: &GpuVec<u32>,
    ) -> ComputeResult<Self> {
        layout.check()?;
        let tb = TileBox::covering(cell_box)?;
        let n_tiles = tb.tile_count();
        let n_cand = u32_len("brick grid candidates", tb.candidate_count())?;
        if mask.len() != u64::from(n_cand) {
            return Err(ComputeError::Shape(format!(
                "allocate: mask of {} entries for {n_cand} candidate bricks",
                mask.len()
            )));
        }
        if fill_of_unmarked.len() != u64::from(n_tiles) {
            return Err(ComputeError::Shape(format!(
                "allocate: fill_of_unmarked of {} entries for {n_tiles} tiles",
                fill_of_unmarked.len()
            )));
        }
        let gpu = cx.gpu_arc();
        let params = [n_tiles, n_cand, 0, 0, 0, 0, 0, 0];
        let flags = GpuVec::<u32>::zeroed(&gpu, "brick grid tile flags", u64::from(n_tiles))?;
        let node_of = GpuVec::<u32>::zeroed(&gpu, "brick grid node of tile", u64::from(n_tiles))?;
        let brick_of =
            GpuVec::<u32>::zeroed(&gpu, "brick grid brick of candidate", u64::from(n_cand))?;
        let totals = GpuVec::<u32>::zeroed(&gpu, "brick grid totals", 2)?;

        let k = Kernel::new(
            &gpu,
            "brick grid tile flags",
            &wgsl::tile_flags(),
            "main",
            &[],
        )?;
        launch(
            cx,
            &k,
            &params,
            &[mask.binding(), flags.binding()],
            u64::from(n_tiles),
        );
        prim.scan(
            cx,
            ScanKind::Exclusive,
            &flags,
            &node_of,
            Some((&totals, 0)),
        )?;
        prim.scan_flags(cx, ScanKind::Exclusive, mask, &brick_of, Some((&totals, 1)))?;
        cx.submit();
        let counts = totals.read(&gpu).wait()?;
        let (nodes, bricks) = (counts[0], counts[1]);
        if bricks >= FILL1_CODE {
            return Err(ComputeError::Shape(format!("allocate: {bricks} bricks")));
        }

        let stride = u64::from(enc.brick_words(layout));
        let root = GpuVec::<u32>::zeroed(&gpu, "brick grid root", u64::from(n_tiles))?;
        let tiles = GpuVec::<u32>::zeroed(
            &gpu,
            "brick grid tiles",
            u64::from(nodes) * u64::from(TILE_SLOTS),
        )?;
        let data = GpuVec::<u32>::zeroed(&gpu, "brick grid data", u64::from(bricks) * stride)?;
        let k = Kernel::new(
            &gpu,
            "brick grid write root",
            &wgsl::write_root(),
            "main",
            &[],
        )?;
        launch(
            cx,
            &k,
            &params,
            &[
                flags.binding(),
                node_of.binding(),
                fill_of_unmarked.binding(),
                root.binding(),
            ],
            u64::from(n_tiles),
        );
        if nodes > 0 {
            let k = Kernel::new(
                &gpu,
                "brick grid write tiles",
                &wgsl::write_tiles(),
                "main",
                &[],
            )?;
            launch(
                cx,
                &k,
                &params,
                &[
                    mask.binding(),
                    flags.binding(),
                    node_of.binding(),
                    brick_of.binding(),
                    fill_of_unmarked.binding(),
                    tiles.binding(),
                ],
                u64::from(n_cand),
            );
        }
        // Submitted, so a later download (its own queue submission) sees the index.
        cx.submit();
        Ok(Self::assemble(
            &gpu, root, tiles, data, layout, enc, tb, fill, nodes, bricks,
        ))
    }

    /// Number of bricks (known after `allocate`'s readback).
    pub fn brick_count(&self) -> u32 {
        self.brick_count
    }

    /// Number of tile nodes.
    pub fn tile_node_count(&self) -> u32 {
        self.node_count
    }

    pub fn layout(&self) -> BrickLayout {
        self.layout
    }

    pub fn encoding(&self) -> GpuEncoding {
        self.encoding
    }

    /// The tiles the grid covers.
    pub fn tile_box(&self) -> TileBox {
        self.tile_box
    }

    /// Bytes of the root and tile buffers (the index).
    pub fn index_bytes(&self) -> u64 {
        (self.root.len() + self.tiles.len()) * 4
    }

    /// Bytes of the brick payload buffer.
    pub fn data_bytes(&self) -> u64 {
        self.data.len() * 4
    }

    /// Meta, root, tiles, data: bind at `first_binding..first_binding + 3` of the group given
    /// to `brick_grid_wgsl`.
    pub fn bindings(&self) -> [wgpu::BindingResource<'_>; 4] {
        [
            self.meta.as_entire_binding(),
            self.root.binding(),
            self.tiles.binding(),
            self.data.binding(),
        ]
    }

    /// Read-write variant of `data` for the producing kernel (`brick_grid_wgsl(.., true)`).
    pub fn data_mut_binding(&self) -> wgpu::BindingResource<'_> {
        self.data.binding()
    }

    /// The raw `root`, `tiles` and `data` buffers (determinism checks, custom kernels).
    pub fn buffers(&self) -> [&GpuVec<u32>; 3] {
        [&self.root, &self.tiles, &self.data]
    }
}

/// One thread per element, parameters at binding 0, `resources` at 1...
fn launch(
    cx: &mut ComputeCtx,
    k: &Kernel,
    params: &[u32; 8],
    resources: &[wgpu::BindingResource<'_>],
    n: u64,
) {
    let slot = cx.uniform(params);
    let mut entries = vec![cx.uniform_binding(slot)];
    entries.extend(resources.iter().cloned());
    let bind = k.bind_group(cx.gpu(), 0, &entries);
    cx.dispatch_1d(k, &[&bind], n);
}
