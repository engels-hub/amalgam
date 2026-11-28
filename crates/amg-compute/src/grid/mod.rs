//! Voxel and brick grids (CMP-02, design note §4).
//!
//! - [`DenseGrid`]: a plain dense box of values, x fastest.
//! - [`BrickGrid`]: a sparse grid in index space with a fixed two-level tree. A dense *root*
//!   covers the grid's [`TileBox`] with one entry per tile of 8³ bricks (64³ cells); an
//!   allocated tile has a 512-slot *tile node*; a slot holds a brick index or one of two fill
//!   codes ([`Fill`]). Bricks are 8³ cells, stored as `(8 + apron)³` samples ([`BrickLayout`]).
//! - [`GpuBrickGrid`] (feature `gpu`): the GPU mirror with the same root/tile/brick layout, the
//!   WGSL accessors from [`brick_grid_wgsl`] and a deterministic GPU-side
//!   [`GpuBrickGrid::allocate`].
//!
//! **Canonical order.** Tile nodes are kept in root order (x fastest) and bricks in the order of
//! their [`TileBox::candidate_index`]: root index of their tile, then the 9-bit Morton code of
//! the brick inside the tile. This is the order of [`BrickGrid::canonicalize`] and of
//! `GpuBrickGrid::allocate`, so a CPU and a GPU build give the same buffers. (A global Morton
//! index over a non-cubic box is sparse; tile-major order keeps the candidate list dense.)
//!
//! **Slot codes** (CPU and GPU): a value below [`FILL1_CODE`] is a brick index;
//! [`FILL0_CODE`] = `0xFFFF_FFFF` and [`FILL1_CODE`] = `0xFFFF_FFFE` mark empty space. In a
//! node, slot `x + 8 y + 64 z` belongs to the brick at local position `(x, y, z)` of the tile.

mod brick;
mod dense;

#[cfg(feature = "gpu")]
mod gpu;
#[cfg(feature = "gpu")]
mod wgsl;

pub use brick::BrickGrid;
pub use dense::DenseGrid;

#[cfg(feature = "gpu")]
pub use gpu::GpuBrickGrid;
#[cfg(feature = "gpu")]
pub use wgsl::brick_grid_wgsl;

use crate::{ComputeError, ComputeResult, f16};

/// Cells per brick edge (fixed).
pub const BRICK_EDGE: u32 = 8;
/// Bricks per tile edge (fixed): a tile is 64³ cells.
pub const TILE_EDGE: u32 = 8;
/// Brick slots per tile node.
pub const TILE_SLOTS: u32 = TILE_EDGE * TILE_EDGE * TILE_EDGE;
/// Largest number of root entries (tiles) a grid may have: 2²⁴ (256³ tiles, 16 384³ cells).
pub const MAX_ROOT_TILES: u64 = 1 << 24;
/// Slot code of a [`Fill::Fill0`] region.
pub const FILL0_CODE: u32 = 0xFFFF_FFFF;
/// Slot code of a [`Fill::Fill1`] region. Brick indices are below this value.
pub const FILL1_CODE: u32 = 0xFFFF_FFFE;

/// Cells per tile edge.
const TILE_CELLS: i64 = (BRICK_EDGE * TILE_EDGE) as i64;

/// How a grid's samples are stored on the GPU.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GpuEncoding {
    /// Two f16 per `u32` (even sample in the low half), read with `unpack2x16float`.
    F16Packed,
    /// One f32 per `u32`.
    F32,
    /// Four u8 per `u32` (sample `4k` in the lowest byte); read as `f32(byte)`.
    U8Packed,
}

impl GpuEncoding {
    /// Samples per `u32` word.
    pub fn samples_per_word(self) -> u32 {
        match self {
            GpuEncoding::F16Packed => 2,
            GpuEncoding::F32 => 1,
            GpuEncoding::U8Packed => 4,
        }
    }

    /// `u32` words per brick of `layout` (the last word of a brick may hold padding).
    pub fn brick_words(self, layout: BrickLayout) -> u32 {
        (layout.samples() as u32).div_ceil(self.samples_per_word())
    }
}

/// Sample values a grid can hold: f16, f32, u8.
pub trait GridValue: bytemuck::Pod + Send + Sync + crate::sealed::Sealed {
    /// How this type is stored on the GPU.
    const ENCODING: GpuEncoding;
    /// The value as f32 (exact for all three types; u8 gives `0.0..=255.0`).
    fn to_f32(self) -> f32;
    /// Nearest value (u8: rounded half to even and clamped to `0..=255`, like WGSL `round`).
    fn from_f32(v: f32) -> Self;
}

impl GridValue for f16 {
    const ENCODING: GpuEncoding = GpuEncoding::F16Packed;
    fn to_f32(self) -> f32 {
        f16::to_f32(self)
    }
    fn from_f32(v: f32) -> Self {
        f16::from_f32(v)
    }
}

impl GridValue for f32 {
    const ENCODING: GpuEncoding = GpuEncoding::F32;
    fn to_f32(self) -> f32 {
        self
    }
    fn from_f32(v: f32) -> Self {
        v
    }
}

impl GridValue for u8 {
    const ENCODING: GpuEncoding = GpuEncoding::U8Packed;
    fn to_f32(self) -> f32 {
        f32::from(self)
    }
    fn from_f32(v: f32) -> Self {
        // Clamped first; `as` maps NaN to 0.
        v.round_ties_even().clamp(0.0, 255.0) as u8
    }
}

/// Brick coordinate (cell coordinate / 8, floor).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct BrickCoord(pub [i32; 3]);

impl BrickCoord {
    /// The brick containing cell (or sample) `ijk`.
    pub fn of_cell(ijk: [i32; 3]) -> BrickCoord {
        BrickCoord(ijk.map(|c| c.div_euclid(BRICK_EDGE as i32)))
    }

    /// Morton code (bit `3k` = bit `k` of x, `3k + 1` of y, `3k + 2` of z) of the low 21 bits of
    /// each component. Morton order is not translation invariant, so pass coordinates relative
    /// to the grid's tile origin (`b - TileBox::origin_brick()`), which are non-negative.
    pub fn morton(self) -> u64 {
        let [x, y, z] = self.0.map(|c| spread3(u64::from(c as u32 & 0x1F_FFFF)));
        x | (y << 1) | (z << 2)
    }

    /// The cell (or corner sample) at the brick's low corner.
    pub fn min_cell(self) -> [i32; 3] {
        self.0.map(|c| c * BRICK_EDGE as i32)
    }
}

/// Spreads the low 21 bits of `v` to every third bit.
fn spread3(v: u64) -> u64 {
    let mut x = v & 0x1F_FFFF;
    x = (x | (x << 32)) & 0x001F_0000_0000_FFFF;
    x = (x | (x << 16)) & 0x001F_0000_FF00_00FF;
    x = (x | (x << 8)) & 0x100F_00F0_0F00_F00F;
    x = (x | (x << 4)) & 0x10C3_0C30_C30C_30C3;
    x = (x | (x << 2)) & 0x1249_2492_4924_9249;
    x
}

/// Morton code of a local brick position inside a tile (each component `0..8`): 9 bits.
pub(crate) fn local_morton(l: [u32; 3]) -> u32 {
    let mut m = 0;
    for bit in 0..3 {
        m |= ((l[0] >> bit) & 1) << (3 * bit);
        m |= ((l[1] >> bit) & 1) << (3 * bit + 1);
        m |= ((l[2] >> bit) & 1) << (3 * bit + 2);
    }
    m
}

/// Inverse of [`local_morton`].
pub(crate) fn local_from_morton(m: u32) -> [u32; 3] {
    let mut l = [0; 3];
    for bit in 0..3 {
        for (axis, c) in l.iter_mut().enumerate() {
            *c |= ((m >> (3 * bit + axis as u32)) & 1) << bit;
        }
    }
    l
}

/// Samples stored per brick: `(BRICK_EDGE + apron)³`. Apron 1 = 9³ corner samples (S08).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BrickLayout {
    /// 0 or 1.
    pub apron: u32,
}

impl BrickLayout {
    /// 512 cell values per brick (sample `ijk` is the centre of cell `ijk`).
    pub const CELLS: BrickLayout = BrickLayout { apron: 0 };
    /// 729 corner samples per brick (sample `ijk` is the low corner of cell `ijk`); the samples
    /// with a local coordinate of 8 duplicate the neighbour's first layer (the apron), so a
    /// trilinear lookup reads one brick.
    pub const CORNERS: BrickLayout = BrickLayout { apron: 1 };

    /// `BRICK_EDGE + apron`.
    pub fn samples_per_axis(self) -> u32 {
        BRICK_EDGE + self.apron
    }

    /// `samples_per_axis()³`.
    pub fn samples(self) -> usize {
        (self.samples_per_axis() as usize).pow(3)
    }

    pub(crate) fn check(self) -> ComputeResult<()> {
        if self.apron > 1 {
            return Err(ComputeError::Shape(format!(
                "brick apron {} (must be 0 or 1)",
                self.apron
            )));
        }
        Ok(())
    }
}

/// Constant value of an empty region (S08: Fill0 = outside, Fill1 = inside).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Fill {
    /// Slot code [`FILL0_CODE`]; also the value outside the grid's box.
    Fill0,
    /// Slot code [`FILL1_CODE`].
    Fill1,
}

impl Fill {
    /// The slot code of this fill.
    pub fn code(self) -> u32 {
        match self {
            Fill::Fill0 => FILL0_CODE,
            Fill::Fill1 => FILL1_CODE,
        }
    }

    /// The fill of a slot code, `None` for a brick index.
    pub fn of_code(code: u32) -> Option<Fill> {
        match code {
            FILL0_CODE => Some(Fill::Fill0),
            FILL1_CODE => Some(Fill::Fill1),
            _ => None,
        }
    }

    fn index(self) -> usize {
        match self {
            Fill::Fill0 => 0,
            Fill::Fill1 => 1,
        }
    }
}

/// Index of a brick in a grid's brick pool (changes on `remove` and `canonicalize`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BrickId(pub u32);

/// What a brick slot holds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Slot<'a, T> {
    /// A stored brick and its samples (`layout.samples()`, x fastest).
    Brick(BrickId, &'a [T]),
    /// Empty space.
    Fill(Fill),
}

/// The box of tiles a grid covers: `dims` tiles from tile `origin` (tile = 64³ cells).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileBox {
    /// Tile coordinate (cell / 64, floor) of the first tile.
    pub origin: [i32; 3],
    /// Tiles per axis (each ≥ 1); the root has `dims[0] * dims[1] * dims[2]` entries.
    pub dims: [u32; 3],
}

impl TileBox {
    /// The tiles covering `cell_box` (inclusive-exclusive cell range).
    ///
    /// # Errors
    /// [`ComputeError::Shape`] for an empty box or one that needs more than 2²⁴ tiles.
    pub fn covering(cell_box: ([i32; 3], [i32; 3])) -> ComputeResult<TileBox> {
        let (lo, hi) = cell_box;
        let mut origin = [0; 3];
        let mut dims = [0; 3];
        let mut count: u64 = 1;
        for a in 0..3 {
            if hi[a] <= lo[a] {
                return Err(ComputeError::Shape(format!(
                    "empty cell box {lo:?}..{hi:?} on axis {a}"
                )));
            }
            let t0 = i64::from(lo[a]).div_euclid(TILE_CELLS);
            let t1 = (i64::from(hi[a]) - 1).div_euclid(TILE_CELLS) + 1;
            origin[a] = t0 as i32;
            let d = (t1 - t0) as u64;
            count = count.saturating_mul(d);
            dims[a] = d as u32;
        }
        if count > MAX_ROOT_TILES {
            return Err(ComputeError::Shape(format!(
                "cell box {lo:?}..{hi:?} needs {count} root tiles (limit 2^24 = {MAX_ROOT_TILES})"
            )));
        }
        Ok(TileBox { origin, dims })
    }

    /// Number of tiles (root entries).
    pub fn tile_count(&self) -> u32 {
        self.dims[0] * self.dims[1] * self.dims[2]
    }

    /// Number of candidate bricks: `tile_count() * 512`.
    pub fn candidate_count(&self) -> u64 {
        u64::from(self.tile_count()) * u64::from(TILE_SLOTS)
    }

    /// The brick coordinate of the box's first brick.
    pub fn origin_brick(&self) -> BrickCoord {
        BrickCoord(self.origin.map(|t| t * TILE_EDGE as i32))
    }

    /// The cell range the tiles cover (inclusive-exclusive).
    pub fn cell_box(&self) -> ([i32; 3], [i32; 3]) {
        let lo = self.origin.map(|t| t * TILE_CELLS as i32);
        let hi = [0, 1, 2].map(|a| lo[a] + (self.dims[a] as i32) * TILE_CELLS as i32);
        (lo, hi)
    }

    /// Root index of absolute tile coordinate `tile`, `None` outside the box.
    pub fn root_index(&self, tile: [i32; 3]) -> Option<u32> {
        let mut r = [0u32; 3];
        for a in 0..3 {
            let t = i64::from(tile[a]) - i64::from(self.origin[a]);
            if t < 0 || t >= i64::from(self.dims[a]) {
                return None;
            }
            r[a] = t as u32;
        }
        Some((r[2] * self.dims[1] + r[1]) * self.dims[0] + r[0])
    }

    /// Absolute tile coordinate of root index `r`.
    pub fn tile_of_root(&self, r: u32) -> [i32; 3] {
        let [dx, dy, _] = self.dims;
        let local = [r % dx, (r / dx) % dy, r / (dx * dy)];
        [0, 1, 2].map(|a| self.origin[a] + local[a] as i32)
    }

    /// `(root index, slot = x + 8 y + 64 z, 9-bit local Morton code)` of brick `b`.
    pub(crate) fn locate(&self, b: BrickCoord) -> Option<(u32, u32, u32)> {
        let e = TILE_EDGE as i32;
        let r = self.root_index(b.0.map(|c| c.div_euclid(e)))?;
        let l = b.0.map(|c| c.rem_euclid(e) as u32);
        Some((r, l[0] + 8 * l[1] + 64 * l[2], local_morton(l)))
    }

    /// Index of brick `b` in the candidate list of [`GpuBrickGrid::allocate`] (and the
    /// canonical sort key): `root index * 512 + local Morton code`. `None` outside the box.
    pub fn candidate_index(&self, b: BrickCoord) -> Option<u64> {
        let (r, _, m) = self.locate(b)?;
        Some(u64::from(r) * u64::from(TILE_SLOTS) + u64::from(m))
    }

    /// The brick of candidate index `i` (inverse of [`TileBox::candidate_index`]).
    pub fn candidate_brick(&self, i: u64) -> BrickCoord {
        let tile = self.tile_of_root((i / u64::from(TILE_SLOTS)) as u32);
        let l = local_from_morton((i % u64::from(TILE_SLOTS)) as u32);
        BrickCoord([0, 1, 2].map(|a| tile[a] * TILE_EDGE as i32 + l[a] as i32))
    }

    /// The brick of slot `slot` (`x + 8 y + 64 z`) in tile node at root index `r`.
    #[cfg(feature = "gpu")]
    pub(crate) fn slot_brick(&self, r: u32, slot: u32) -> BrickCoord {
        let tile = self.tile_of_root(r);
        let l = [slot % 8, (slot / 8) % 8, slot / 64];
        BrickCoord([0, 1, 2].map(|a| tile[a] * TILE_EDGE as i32 + l[a] as i32))
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "exact values are the point of these tests"
)]
mod tests {
    use super::*;

    #[test]
    fn morton_interleaves_bits() {
        assert_eq!(BrickCoord([1, 0, 0]).morton(), 0b001);
        assert_eq!(BrickCoord([0, 1, 0]).morton(), 0b010);
        assert_eq!(BrickCoord([0, 0, 1]).morton(), 0b100);
        assert_eq!(BrickCoord([3, 0, 0]).morton(), 0b1001);
        assert_eq!(
            BrickCoord([0x1F_FFFF, 0x1F_FFFF, 0x1F_FFFF]).morton(),
            (1 << 63) - 1
        );
        for m in 0..512 {
            let l = local_from_morton(m);
            assert_eq!(local_morton(l), m);
            assert_eq!(
                u64::from(m),
                BrickCoord(l.map(|c| c as i32)).morton(),
                "{l:?}"
            );
        }
    }

    #[test]
    fn of_cell_floors() {
        assert_eq!(BrickCoord::of_cell([0, 7, 8]).0, [0, 0, 1]);
        assert_eq!(BrickCoord::of_cell([-1, -8, -9]).0, [-1, -1, -2]);
    }

    #[test]
    fn tile_box_covers_and_indexes() {
        let tb = TileBox::covering(([-1, 0, 0], [64, 65, 1])).unwrap();
        assert_eq!(tb.origin, [-1, 0, 0]);
        assert_eq!(tb.dims, [2, 2, 1]);
        assert_eq!(tb.tile_count(), 4);
        assert_eq!(tb.cell_box(), ([-64, 0, 0], [64, 128, 64]));
        for i in 0..tb.candidate_count() {
            let b = tb.candidate_brick(i);
            assert_eq!(tb.candidate_index(b), Some(i));
        }
        assert_eq!(tb.candidate_index(BrickCoord([8, 0, 0])), None);
        assert!(TileBox::covering(([0, 0, 0], [0, 1, 1])).is_err());
    }

    #[test]
    fn root_limit_is_2_pow_24() {
        let edge = 256 * 64;
        assert!(TileBox::covering(([0; 3], [edge; 3])).is_ok());
        let err = TileBox::covering(([0; 3], [edge + 1, edge, edge])).unwrap_err();
        assert!(matches!(err, ComputeError::Shape(_)), "{err}");
        // Extremes do not overflow.
        assert!(TileBox::covering(([i32::MIN; 3], [i32::MAX; 3])).is_err());
    }

    #[test]
    fn u8_from_f32_rounds_half_even_and_clamps() {
        assert_eq!(u8::from_f32(2.5), 2);
        assert_eq!(u8::from_f32(3.5), 4);
        assert_eq!(u8::from_f32(-3.0), 0);
        assert_eq!(u8::from_f32(300.0), 255);
        assert_eq!(u8::from_f32(f32::NAN), 0);
    }

    #[test]
    fn brick_words() {
        assert_eq!(
            GpuEncoding::F16Packed.brick_words(BrickLayout::CORNERS),
            365
        );
        assert_eq!(GpuEncoding::F32.brick_words(BrickLayout::CORNERS), 729);
        assert_eq!(GpuEncoding::U8Packed.brick_words(BrickLayout::CORNERS), 183);
        assert_eq!(GpuEncoding::F16Packed.brick_words(BrickLayout::CELLS), 256);
    }
}
