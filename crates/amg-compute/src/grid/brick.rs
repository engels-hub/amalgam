//! `BrickGrid<T>`: the CPU sparse brick grid.

use std::fmt;

use rayon::prelude::*;

use super::{
    BrickCoord, BrickId, BrickLayout, FILL0_CODE, FILL1_CODE, Fill, GridValue, Slot, TILE_SLOTS,
    TileBox,
};
use crate::{ComputeError, ComputeResult};

/// Slots per node as `usize`.
const SLOTS: usize = TILE_SLOTS as usize;

/// Sparse grid in index space: dense root over tiles, 512-slot tile nodes, brick pool.
/// World mapping (origin, cell size) belongs to the caller.
///
/// Sample `ijk` of brick `b` is stored at `x + s (y + s z)` with `(x, y, z) = ijk - 8 b` and
/// `s = layout.samples_per_axis()`. With [`BrickLayout::CORNERS`], sample `ijk` is the corner
/// at the low end of cell `ijk`; with [`BrickLayout::CELLS`], the centre of cell `ijk`.
///
/// Bricks may only be stored inside the grid's [`TileBox`] (the cell box rounded out to whole
/// 64³ tiles); everything outside it reads as [`Fill::Fill0`].
///
/// Two grids compare equal (`==`) when their buffers are equal, which after
/// [`BrickGrid::canonicalize`] means they hold the same content.
#[derive(Clone)]
pub struct BrickGrid<T: GridValue> {
    pub(crate) layout: BrickLayout,
    pub(crate) fill: [T; 2],
    pub(crate) tile_box: TileBox,
    /// Per tile: a node index or a fill code.
    pub(crate) root: Vec<u32>,
    /// 512 slots per node: a brick index or a fill code.
    pub(crate) tiles: Vec<u32>,
    /// Root index of each node.
    pub(crate) tile_root: Vec<u32>,
    /// Coordinate of each brick.
    pub(crate) coords: Vec<BrickCoord>,
    /// `layout.samples()` values per brick.
    pub(crate) data: Vec<T>,
}

impl<T: GridValue> fmt::Debug for BrickGrid<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BrickGrid")
            .field("layout", &self.layout)
            .field("fill", &self.fill.map(T::to_f32))
            .field("tile_box", &self.tile_box)
            .field("tile_nodes", &self.tile_root.len())
            .field("bricks", &self.coords.len())
            .finish_non_exhaustive()
    }
}

impl<T: GridValue> PartialEq for BrickGrid<T> {
    fn eq(&self, other: &Self) -> bool {
        let bytes = |v: &[T]| bytemuck::cast_slice::<T, u8>(v).to_vec();
        self.layout == other.layout
            && bytes(&self.fill) == bytes(&other.fill)
            && self.tile_box == other.tile_box
            && self.root == other.root
            && self.tiles == other.tiles
            && self.tile_root == other.tile_root
            && self.coords == other.coords
            && bytemuck::cast_slice::<T, u8>(&self.data)
                == bytemuck::cast_slice::<T, u8>(&other.data)
    }
}

impl<T: GridValue> BrickGrid<T> {
    /// An empty grid (everything [`Fill::Fill0`]). `cell_box` = inclusive-exclusive cell range
    /// the grid may cover; it is rounded out to whole tiles.
    ///
    /// # Errors
    /// [`ComputeError::Shape`] for an empty box, a box whose root would exceed 2²⁴ tiles, or
    /// an apron other than 0 or 1.
    pub fn new(
        layout: BrickLayout,
        fill: [T; 2],
        cell_box: ([i32; 3], [i32; 3]),
    ) -> ComputeResult<Self> {
        layout.check()?;
        let tile_box = TileBox::covering(cell_box)?;
        Ok(BrickGrid {
            layout,
            fill,
            tile_box,
            root: vec![FILL0_CODE; tile_box.tile_count() as usize],
            tiles: Vec::new(),
            tile_root: Vec::new(),
            coords: Vec::new(),
            data: Vec::new(),
        })
    }

    pub fn layout(&self) -> BrickLayout {
        self.layout
    }

    pub fn fill_value(&self, f: Fill) -> T {
        self.fill[f.index()]
    }

    /// The tiles the grid covers.
    pub fn tile_box(&self) -> TileBox {
        self.tile_box
    }

    pub fn brick_count(&self) -> usize {
        self.coords.len()
    }

    /// Number of allocated tile nodes.
    pub fn tile_node_count(&self) -> usize {
        self.tile_root.len()
    }

    fn spb(&self) -> usize {
        self.layout.samples()
    }

    fn brick_data(&self, id: u32) -> &[T] {
        let n = self.spb();
        &self.data[id as usize * n..(id as usize + 1) * n]
    }

    /// The slot code of brick `b` (`FILL0_CODE` outside the box).
    pub(crate) fn code(&self, b: BrickCoord) -> u32 {
        let Some((r, slot, _)) = self.tile_box.locate(b) else {
            return FILL0_CODE;
        };
        let node = self.root[r as usize];
        if node >= FILL1_CODE {
            return node;
        }
        self.tiles[node as usize * SLOTS + slot as usize]
    }

    fn locate(&self, b: BrickCoord) -> ComputeResult<(u32, u32)> {
        self.tile_box
            .locate(b)
            .map(|(r, slot, _)| (r, slot))
            .ok_or_else(|| {
                ComputeError::Shape(format!(
                    "brick {:?} is outside the grid's tiles {:?}",
                    b.0, self.tile_box
                ))
            })
    }

    /// The node of root entry `r`, created (all slots = the tile's fill) if needed.
    fn ensure_node(&mut self, r: u32) -> u32 {
        let entry = self.root[r as usize];
        if entry < FILL1_CODE {
            return entry;
        }
        let node = self.tile_root.len() as u32;
        self.tiles.extend(std::iter::repeat_n(entry, SLOTS));
        self.tile_root.push(r);
        self.root[r as usize] = node;
        node
    }

    /// Stores `data` (`layout.samples()` values, x fastest) as brick `b`. Replaces an existing
    /// brick in place (same id).
    ///
    /// # Errors
    /// [`ComputeError::Shape`] when `data` has the wrong length, `b` is outside the grid's tiles,
    /// or the pool is full (2³² − 2 bricks).
    pub fn insert(&mut self, b: BrickCoord, data: &[T]) -> ComputeResult<BrickId> {
        let n = self.spb();
        if data.len() != n {
            return Err(ComputeError::Shape(format!(
                "brick data of {} samples (layout needs {n})",
                data.len()
            )));
        }
        let (r, slot) = self.locate(b)?;
        let node = self.ensure_node(r);
        let at = node as usize * SLOTS + slot as usize;
        let code = self.tiles[at];
        if code < FILL1_CODE {
            self.data[code as usize * n..(code as usize + 1) * n].copy_from_slice(data);
            return Ok(BrickId(code));
        }
        let id = self.coords.len();
        if id >= FILL1_CODE as usize {
            return Err(ComputeError::Shape("brick pool is full".into()));
        }
        self.coords.push(b);
        self.data.extend_from_slice(data);
        self.tiles[at] = id as u32;
        Ok(BrickId(id as u32))
    }

    /// Marks slot `b` as empty with fill `f`, removing a brick stored there.
    ///
    /// # Errors
    /// [`ComputeError::Shape`] when `b` is outside the grid's tiles.
    pub fn set_fill(&mut self, b: BrickCoord, f: Fill) -> ComputeResult<()> {
        let (r, slot) = self.locate(b)?;
        if self.root[r as usize] == f.code() {
            return Ok(());
        }
        let node = self.ensure_node(r);
        let at = node as usize * SLOTS + slot as usize;
        let code = self.tiles[at];
        if code < FILL1_CODE {
            self.remove_brick(code);
        }
        self.tiles[at] = f.code();
        Ok(())
    }

    /// Marks a whole tile (absolute tile coordinate, cell / 64) as `f`, removing its bricks and
    /// its node.
    ///
    /// # Errors
    /// [`ComputeError::Shape`] when the tile is outside the grid.
    pub fn set_tile_fill(&mut self, tile: [i32; 3], f: Fill) -> ComputeResult<()> {
        let r = self.tile_box.root_index(tile).ok_or_else(|| {
            ComputeError::Shape(format!(
                "tile {tile:?} is outside the grid's tiles {:?}",
                self.tile_box
            ))
        })?;
        let node = self.root[r as usize];
        if node < FILL1_CODE {
            let mut ids: Vec<u32> = self.tiles[node as usize * SLOTS..(node as usize + 1) * SLOTS]
                .iter()
                .copied()
                .filter(|&c| c < FILL1_CODE)
                .collect();
            // Descending, so the brick that `remove_brick` moves into a hole is never one that
            // is still to be removed.
            ids.sort_unstable_by(|a, b| b.cmp(a));
            for id in ids {
                self.remove_brick(id);
            }
            self.remove_node(node);
        }
        self.root[r as usize] = f.code();
        Ok(())
    }

    /// Like [`BrickGrid::set_fill`]; a brick outside the grid is ignored.
    pub fn remove(&mut self, b: BrickCoord, f: Fill) {
        // Outside the grid there is nothing to remove.
        let _ = self.set_fill(b, f);
    }

    /// Removes brick `id` from the pool (the last brick moves into its place). The caller
    /// rewrites the slot that pointed to `id`.
    #[allow(
        clippy::expect_used,
        reason = "every stored brick lies in the tile box"
    )]
    fn remove_brick(&mut self, id: u32) {
        let n = self.spb();
        let last = self.coords.len() as u32 - 1;
        if id != last {
            let (head, tail) = self.data.split_at_mut(last as usize * n);
            head[id as usize * n..(id as usize + 1) * n].copy_from_slice(&tail[..n]);
            let moved = self.coords[last as usize];
            self.coords[id as usize] = moved;
            let (r, slot, _) = self
                .tile_box
                .locate(moved)
                .expect("stored bricks are in the box");
            let node = self.root[r as usize];
            self.tiles[node as usize * SLOTS + slot as usize] = id;
        }
        self.coords.pop();
        self.data.truncate(last as usize * n);
    }

    /// Removes node `node` (the last node moves into its place). The caller rewrites the root
    /// entry that pointed to `node`.
    fn remove_node(&mut self, node: u32) {
        let last = self.tile_root.len() as u32 - 1;
        if node != last {
            let (head, tail) = self.tiles.split_at_mut(last as usize * SLOTS);
            head[node as usize * SLOTS..(node as usize + 1) * SLOTS]
                .copy_from_slice(&tail[..SLOTS]);
            let r = self.tile_root[last as usize];
            self.tile_root[node as usize] = r;
            self.root[r as usize] = node;
        }
        self.tile_root.pop();
        self.tiles.truncate(last as usize * SLOTS);
    }

    /// What slot `b` holds; outside the grid, `Fill(Fill0)`.
    pub fn lookup(&self, b: BrickCoord) -> Slot<'_, T> {
        let code = self.code(b);
        match Fill::of_code(code) {
            Some(f) => Slot::Fill(f),
            None => Slot::Brick(BrickId(code), self.brick_data(code)),
        }
    }

    pub fn brick_mut(&mut self, b: BrickCoord) -> Option<&mut [T]> {
        let code = self.code(b);
        if code >= FILL1_CODE {
            return None;
        }
        let n = self.spb();
        Some(&mut self.data[code as usize * n..(code as usize + 1) * n])
    }

    /// Value at a sample point (cell corner with apron 1, cell centre with apron 0). The apron
    /// layer is never read: sample `ijk` comes from the brick containing it.
    pub fn sample(&self, ijk: [i32; 3]) -> T {
        let b = BrickCoord::of_cell(ijk);
        let code = self.code(b);
        if let Some(f) = Fill::of_code(code) {
            return self.fill_value(f);
        }
        let s = self.layout.samples_per_axis() as usize;
        let l = ijk.map(|c| c.rem_euclid(8) as usize);
        self.brick_data(code)[l[0] + s * (l[1] + s * l[2])]
    }

    /// Trilinear interpolation at `p` in sample units (sample `ijk` at `p = ijk`).
    ///
    /// With apron 1 the lookup reads one brick: the one containing `floor(p)`, whose apron
    /// supplies the upper samples (fill the aprons first, [`BrickGrid::fill_aprons`]); a point
    /// in an empty slot gets the slot's fill value. With apron 0 the eight samples are read
    /// one by one with [`BrickGrid::sample`]. The GPU's `bg_trilinear` does the same in f32.
    pub fn trilinear(&self, p: [f64; 3]) -> f64 {
        let f = p.map(f64::floor);
        let t = [0, 1, 2].map(|a| p[a] - f[a]);
        let i = f.map(|c| c as i32);
        let mut c = [0.0f64; 8];
        if self.layout.apron == 0 {
            for (k, ck) in c.iter_mut().enumerate() {
                let o = [k & 1, (k >> 1) & 1, k >> 2].map(|v| v as i32);
                *ck = f64::from(
                    self.sample([i[0] + o[0], i[1] + o[1], i[2] + o[2]])
                        .to_f32(),
                );
            }
        } else {
            let code = self.code(BrickCoord::of_cell(i));
            if let Some(fl) = Fill::of_code(code) {
                return f64::from(self.fill_value(fl).to_f32());
            }
            let s = self.layout.samples_per_axis() as usize;
            let l = i.map(|v| v.rem_euclid(8) as usize);
            let base = l[0] + s * (l[1] + s * l[2]);
            let d = self.brick_data(code);
            for (k, ck) in c.iter_mut().enumerate() {
                let at = base + (k & 1) + s * ((k >> 1) & 1) + s * s * (k >> 2);
                *ck = f64::from(d[at].to_f32());
            }
        }
        lerp8(c, t)
    }

    /// Copies apron samples (local coordinate 8 on some axis) from the neighbouring bricks'
    /// first layer, or the neighbour slot's fill value. Apron 0: nothing to do.
    pub fn fill_aprons(&mut self) {
        if self.layout.apron == 0 {
            return;
        }
        let s = self.layout.samples_per_axis() as usize;
        let apron: Vec<(usize, [i32; 3])> = (0..self.spb())
            .map(|k| [k % s, (k / s) % s, k / (s * s)])
            .filter(|l| l.contains(&8))
            .map(|l| (l[0] + s * (l[1] + s * l[2]), l.map(|v| v as i32)))
            .collect();
        let this = &*self;
        let apron_ref = &apron;
        let values: Vec<T> = this
            .coords
            .par_iter()
            .flat_map_iter(|b| {
                let o = b.min_cell();
                apron_ref
                    .iter()
                    .map(move |&(_, l)| this.sample([o[0] + l[0], o[1] + l[1], o[2] + l[2]]))
            })
            .collect();
        let n = self.spb();
        self.data
            .par_chunks_mut(n)
            .zip(values.par_chunks(apron.len()))
            .for_each(|(brick, vals)| {
                for (&(at, _), &v) in apron.iter().zip(vals) {
                    brick[at] = v;
                }
            });
    }

    /// Reorders the grid into canonical form: tile nodes in root order, bricks by
    /// [`TileBox::candidate_index`] (tile, then Morton code inside the tile), and nodes without
    /// bricks whose slots all hold the same fill collapsed into their root entry. Grids with
    /// the same content compare equal afterwards, whatever order they were built in.
    #[allow(
        clippy::expect_used,
        reason = "every stored brick lies in the tile box"
    )]
    pub fn canonicalize(&mut self) {
        // Collapse brick-less uniform nodes (highest node first: `remove_node` moves the last).
        for node in (0..self.tile_root.len() as u32).rev() {
            let slots = &self.tiles[node as usize * SLOTS..(node as usize + 1) * SLOTS];
            let first = slots[0];
            if first >= FILL1_CODE && slots.iter().all(|&c| c == first) {
                let r = self.tile_root[node as usize];
                self.remove_node(node);
                self.root[r as usize] = first;
            }
        }
        // Nodes in root order.
        let mut order: Vec<u32> = (0..self.tile_root.len() as u32).collect();
        order.sort_unstable_by_key(|&n| self.tile_root[n as usize]);
        let mut tiles = Vec::with_capacity(self.tiles.len());
        let mut tile_root = Vec::with_capacity(self.tile_root.len());
        for (new, &old) in order.iter().enumerate() {
            tiles.extend_from_slice(&self.tiles[old as usize * SLOTS..(old as usize + 1) * SLOTS]);
            let r = self.tile_root[old as usize];
            tile_root.push(r);
            self.root[r as usize] = new as u32;
        }
        self.tiles = tiles;
        self.tile_root = tile_root;
        // Bricks in candidate order.
        let tb = self.tile_box;
        let mut keyed: Vec<(u64, u32)> = self
            .coords
            .iter()
            .enumerate()
            .map(|(id, &b)| (tb.candidate_index(b).expect("in box"), id as u32))
            .collect();
        keyed.sort_unstable();
        let mut new_of = vec![0u32; keyed.len()];
        let mut data = Vec::with_capacity(self.data.len());
        let mut coords = Vec::with_capacity(self.coords.len());
        for (new, &(_, old)) in keyed.iter().enumerate() {
            new_of[old as usize] = new as u32;
            data.extend_from_slice(self.brick_data(old));
            coords.push(self.coords[old as usize]);
        }
        self.data = data;
        self.coords = coords;
        for code in &mut self.tiles {
            if *code < FILL1_CODE {
                *code = new_of[*code as usize];
            }
        }
    }

    /// The stored bricks in pool order (canonical order after `canonicalize`).
    pub fn bricks(&self) -> impl Iterator<Item = (BrickCoord, &[T])> + '_ {
        self.coords
            .iter()
            .copied()
            .zip(self.data.chunks_exact(self.spb()))
    }

    /// The same structure with every sample mapped through `f` and new fill values.
    pub fn map<U: GridValue>(&self, f: impl Fn(T) -> U + Sync, fill: [U; 2]) -> BrickGrid<U> {
        BrickGrid {
            layout: self.layout,
            fill,
            tile_box: self.tile_box,
            root: self.root.clone(),
            tiles: self.tiles.clone(),
            tile_root: self.tile_root.clone(),
            coords: self.coords.clone(),
            data: self.data.par_iter().map(|&v| f(v)).collect(),
        }
    }

    /// Bytes of the index: root and tile nodes (`u32` each), as on the GPU.
    pub fn index_bytes(&self) -> usize {
        (self.root.len() + self.tiles.len()) * 4
    }

    /// Bytes of the brick payloads.
    pub fn brick_bytes(&self) -> usize {
        self.data.len() * size_of::<T>()
    }

    /// All heap bytes: index, payloads and the CPU-only node and brick back-references.
    pub fn bytes(&self) -> usize {
        self.index_bytes()
            + self.brick_bytes()
            + self.tile_root.len() * 4
            + self.coords.len() * size_of::<BrickCoord>()
    }

    /// blake3 over layout, value type, fill values, tile box, and the content in canonical
    /// order: per tile in root order its fill, or per slot (Morton order) its fill or brick
    /// samples. It does not depend on the storage order, so it is the same before and after
    /// `canonicalize`.
    pub fn content_hash(&self) -> [u8; 32] {
        let mut h = blake3::Hasher::new();
        h.update(b"amg-compute BrickGrid v1");
        h.update(&self.layout.apron.to_le_bytes());
        h.update(&[T::ENCODING as u8]);
        h.update(bytemuck::cast_slice(&self.fill));
        for v in self.tile_box.origin {
            h.update(&v.to_le_bytes());
        }
        for v in self.tile_box.dims {
            h.update(&v.to_le_bytes());
        }
        let slot_of_morton: Vec<usize> = (0..TILE_SLOTS)
            .map(|m| {
                let l = super::local_from_morton(m);
                (l[0] + 8 * l[1] + 64 * l[2]) as usize
            })
            .collect();
        for &entry in &self.root {
            if entry >= FILL1_CODE {
                h.update(&[0]);
                h.update(&entry.to_le_bytes());
                continue;
            }
            let slots = &self.tiles[entry as usize * SLOTS..(entry as usize + 1) * SLOTS];
            let first = slots[0];
            if first >= FILL1_CODE && slots.iter().all(|&c| c == first) {
                h.update(&[0]);
                h.update(&first.to_le_bytes());
                continue;
            }
            h.update(&[1]);
            for &slot in &slot_of_morton {
                let code = slots[slot];
                if code >= FILL1_CODE {
                    h.update(&code.to_le_bytes());
                } else {
                    h.update(&0u32.to_le_bytes());
                    h.update(bytemuck::cast_slice(self.brick_data(code)));
                }
            }
        }
        *h.finalize().as_bytes()
    }

    /// Assembles a grid from its buffers (GPU download). Callers guarantee consistency.
    #[cfg(feature = "gpu")]
    pub(crate) fn from_parts(
        layout: BrickLayout,
        fill: [T; 2],
        tile_box: TileBox,
        root: Vec<u32>,
        tiles: Vec<u32>,
        data: Vec<T>,
    ) -> Self {
        let nodes = tiles.len() / SLOTS;
        let mut tile_root = vec![0u32; nodes];
        for (r, &e) in root.iter().enumerate() {
            if e < FILL1_CODE {
                tile_root[e as usize] = r as u32;
            }
        }
        let bricks = data.len() / layout.samples();
        let mut coords = vec![BrickCoord([0; 3]); bricks];
        for (node, slots) in tiles.chunks_exact(SLOTS).enumerate() {
            for (slot, &code) in slots.iter().enumerate() {
                if code < FILL1_CODE {
                    coords[code as usize] = tile_box.slot_brick(tile_root[node], slot as u32);
                }
            }
        }
        BrickGrid {
            layout,
            fill,
            tile_box,
            root,
            tiles,
            tile_root,
            coords,
            data,
        }
    }
}

/// Trilinear blend of corners `c[x + 2 y + 4 z]`: along x, then y, then z (as `bg_trilinear`).
pub(crate) fn lerp8(c: [f64; 8], t: [f64; 3]) -> f64 {
    let mix = |a: f64, b: f64, t: f64| a * (1.0 - t) + b * t;
    let x00 = mix(c[0], c[1], t[0]);
    let x10 = mix(c[2], c[3], t[0]);
    let x01 = mix(c[4], c[5], t[0]);
    let x11 = mix(c[6], c[7], t[0]);
    mix(mix(x00, x10, t[1]), mix(x01, x11, t[1]), t[2])
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "exact values are the point of these tests"
)]
mod tests {
    use super::*;
    use crate::f16;

    fn grid() -> BrickGrid<f32> {
        BrickGrid::new(
            BrickLayout::CORNERS,
            [1.0, -1.0],
            ([-64, 0, 0], [64, 64, 64]),
        )
        .unwrap()
    }

    fn brick(v: f32) -> Vec<f32> {
        vec![v; BrickLayout::CORNERS.samples()]
    }

    #[test]
    fn insert_lookup_replace_remove() {
        let mut g = grid();
        assert_eq!(g.lookup(BrickCoord([0, 0, 0])), Slot::Fill(Fill::Fill0));
        let a = g.insert(BrickCoord([0, 0, 0]), &brick(2.0)).unwrap();
        let b = g.insert(BrickCoord([-1, 3, 7]), &brick(3.0)).unwrap();
        assert_eq!((a, b), (BrickId(0), BrickId(1)));
        assert_eq!(g.insert(BrickCoord([0, 0, 0]), &brick(4.0)).unwrap(), a);
        assert_eq!(g.brick_count(), 2);
        assert_eq!(g.tile_node_count(), 2);
        assert_eq!(g.sample([0, 0, 0]), 4.0);
        assert_eq!(g.sample([-8, 24, 56]), 3.0);
        assert_eq!(g.sample([8, 0, 0]), 1.0);
        g.remove(BrickCoord([0, 0, 0]), Fill::Fill1);
        assert_eq!(g.brick_count(), 1);
        assert_eq!(g.lookup(BrickCoord([0, 0, 0])), Slot::Fill(Fill::Fill1));
        assert_eq!(
            g.sample([-8, 24, 56]),
            3.0,
            "the moved brick is still found"
        );
        assert!(g.insert(BrickCoord([8, 0, 0]), &brick(0.0)).is_err());
        assert!(g.insert(BrickCoord([0, 0, 0]), &[0.0; 3]).is_err());
        g.remove(BrickCoord([100, 0, 0]), Fill::Fill0); // outside: ignored
    }

    #[test]
    fn tile_fill_drops_bricks_and_node() {
        let mut g = grid();
        for x in 0..8 {
            g.insert(BrickCoord([x - 8, 0, 0]), &brick(x as f32))
                .unwrap();
            g.insert(BrickCoord([x, 0, 0]), &brick(10.0 + x as f32))
                .unwrap();
        }
        g.set_tile_fill([-1, 0, 0], Fill::Fill1).unwrap();
        assert_eq!(g.brick_count(), 8);
        assert_eq!(g.tile_node_count(), 1);
        for x in 0..8 {
            assert_eq!(g.sample([8 * x, 0, 0]), 10.0 + x as f32);
            assert_eq!(g.sample([8 * x - 64, 0, 0]), -1.0);
        }
        assert!(g.set_tile_fill([5, 0, 0], Fill::Fill0).is_err());
    }

    #[test]
    fn trilinear_reproduces_a_linear_field() {
        // f = x + 2y - z is reproduced exactly by trilinear interpolation of corner samples.
        let f = |p: [i32; 3]| (p[0] + 2 * p[1] - p[2]) as f32;
        let mut g = grid();
        for b in [[0, 0, 0], [1, 0, 0], [-1, 2, 3]] {
            let b = BrickCoord(b);
            let o = b.min_cell();
            let data: Vec<f32> = (0..729)
                .map(|k| f([o[0] + k % 9, o[1] + (k / 9) % 9, o[2] + k / 81]))
                .collect();
            g.insert(b, &data).unwrap();
        }
        // Apron between the two neighbours already holds the right values; break it, then
        // fill it back from the neighbour.
        g.brick_mut(BrickCoord([0, 0, 0])).unwrap()[8] = 99.0;
        g.fill_aprons();
        for p in [
            [7.5, 3.25, 1.0],
            [0.0, 0.0, 0.0],
            [12.9, 6.99, 6.5],
            [-4.5, 20.0, 30.1],
        ] {
            let want = p[0] + 2.0 * p[1] - p[2];
            assert!((g.trilinear(p) - want).abs() < 1e-9, "{p:?}");
        }
        // The apron of the brick next to empty space holds the fill value.
        assert_eq!(g.brick_mut(BrickCoord([1, 0, 0])).unwrap()[8], 1.0);
        assert_eq!(g.trilinear([20.0, 0.0, 0.0]), 1.0);
    }

    #[test]
    fn cell_layout_trilinear_reads_neighbours() {
        let mut g: BrickGrid<f32> =
            BrickGrid::new(BrickLayout::CELLS, [0.0, 0.0], ([0; 3], [64; 3])).unwrap();
        for b in [[0, 0, 0], [1, 0, 0]] {
            let b = BrickCoord(b);
            let o = b.min_cell();
            let data: Vec<f32> = (0..512).map(|k| (o[0] + k % 8) as f32).collect();
            g.insert(b, &data).unwrap();
        }
        assert!((g.trilinear([7.25, 1.0, 1.0]) - 7.25).abs() < 1e-12);
    }

    #[test]
    fn canonical_form_is_order_independent() {
        let coords: Vec<BrickCoord> = (0..200)
            .map(|i| BrickCoord([i % 16 - 8, (i / 16) % 8, i / 128]))
            .collect();
        let build = |order: &mut dyn Iterator<Item = &BrickCoord>| {
            let mut g: BrickGrid<f16> = BrickGrid::new(
                BrickLayout::CORNERS,
                [f16::from_f32(4.0), f16::from_f32(-4.0)],
                ([-64, 0, 0], [64, 64, 64]),
            )
            .unwrap();
            g.set_tile_fill([0, 0, 0], Fill::Fill1).unwrap();
            for b in order {
                let v = f16::from_f32((b.0[0] * 100 + b.0[1] * 10 + b.0[2]) as f32);
                g.insert(*b, &vec![v; 729]).unwrap();
            }
            g
        };
        let mut a = build(&mut coords.iter());
        let mut b = build(&mut coords.iter().rev());
        assert_eq!(a.content_hash(), b.content_hash());
        assert_ne!(a, b);
        a.canonicalize();
        b.canonicalize();
        assert_eq!(a, b);
        let h = a.content_hash();
        assert_eq!(h, b.content_hash());
        // A different sample changes the hash.
        b.brick_mut(coords[0]).unwrap()[3] = f16::from_f32(0.5);
        assert_ne!(h, b.content_hash());
    }

    #[test]
    fn canonicalize_collapses_uniform_nodes() {
        let mut g = grid();
        g.set_fill(BrickCoord([0, 0, 0]), Fill::Fill1).unwrap();
        g.insert(BrickCoord([-1, 0, 0]), &brick(5.0)).unwrap();
        g.remove(BrickCoord([-1, 0, 0]), Fill::Fill0);
        assert_eq!(g.tile_node_count(), 2);
        let h = g.content_hash();
        g.canonicalize();
        assert_eq!(g.tile_node_count(), 1);
        assert_eq!(g.content_hash(), h);
        assert_eq!(g, {
            let mut e = grid();
            e.set_fill(BrickCoord([0, 0, 0]), Fill::Fill1).unwrap();
            e
        });
    }

    #[test]
    fn map_and_bytes() {
        let mut g = grid();
        g.insert(BrickCoord([0, 0, 0]), &brick(2.5)).unwrap();
        let h = g.map(f16::from_f32, [f16::ONE, f16::NEG_ONE]);
        assert_eq!(h.sample([1, 1, 1]), f16::from_f32(2.5));
        assert_eq!(h.brick_bytes(), 729 * 2);
        assert_eq!(h.index_bytes(), (2 + 512) * 4);
        assert!(h.bytes() > h.index_bytes() + h.brick_bytes());
    }
}
