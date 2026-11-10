//! `SubShapeIndex`: adjacency tables over the 0-based `TopExp::MapShapes` indices (S02 §3).
//!
//! Plain Rust data, so it needs no OCCT at query time and can be tested without the backend.

use crate::shape::{EdgeId, FaceId, VertexId};

/// Raw tables from the backend. Same layout as `amg_occt_sys::ffi::OcTopology`.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct RawTopology {
    pub(crate) n_faces: u32,
    pub(crate) n_edges: u32,
    pub(crate) n_vertices: u32,
    pub(crate) face_wire_starts: Vec<u32>,
    pub(crate) wire_edge_starts: Vec<u32>,
    pub(crate) wire_edges: Vec<u32>,
    pub(crate) edge_vertices: Vec<u32>,
    pub(crate) vertex_edge_starts: Vec<u32>,
    pub(crate) vertex_edges: Vec<u32>,
}

/// Compressed adjacency: the items of slot `i` are `items[starts[i]..starts[i + 1]]`.
#[derive(Clone, Debug, PartialEq)]
struct Table<T> {
    starts: Vec<u32>,
    items: Vec<T>,
}

impl<T> Table<T> {
    fn get(&self, i: usize) -> &[T] {
        &self.items[self.starts[i] as usize..self.starts[i + 1] as usize]
    }

    fn slots(&self) -> usize {
        self.starts.len().saturating_sub(1)
    }
}

/// The sub-shape index of one shape.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct SubShapeIndex {
    n_faces: u32,
    n_edges: u32,
    n_vertices: u32,
    /// Per face: its wires, outer first, as ranges of `wires`.
    face_wires: Table<u32>,
    /// Per wire: its edges in traversal order. A seam edge appears twice.
    wires: Table<EdgeId>,
    /// Per face: distinct edges in wire order.
    face_edges: Table<EdgeId>,
    /// Per edge: distinct faces, ascending.
    edge_faces: Table<FaceId>,
    edge_vertices: Vec<[VertexId; 2]>,
    vertex_edges: Table<EdgeId>,
}

impl SubShapeIndex {
    pub(crate) fn from_raw(raw: RawTopology) -> Self {
        let wires = Table {
            starts: raw.wire_edge_starts,
            items: raw.wire_edges.into_iter().map(EdgeId).collect(),
        };
        // Offsets of each face's wires, stored as wire numbers.
        let n_wires = wires.slots() as u32;
        let wire_numbers: Vec<u32> = (0..n_wires).collect();
        let face_wires = Table {
            starts: raw.face_wire_starts,
            items: wire_numbers,
        };

        let face_edges = dedup_face_edges(raw.n_faces, raw.n_edges, &face_wires, &wires);
        let edge_faces = invert(&face_edges, raw.n_edges);
        let edge_vertices = raw
            .edge_vertices
            .chunks_exact(2)
            .map(|pair| [VertexId(pair[0]), VertexId(pair[1])])
            .collect();
        let vertex_edges = Table {
            starts: raw.vertex_edge_starts,
            items: raw.vertex_edges.into_iter().map(EdgeId).collect(),
        };

        Self {
            n_faces: raw.n_faces,
            n_edges: raw.n_edges,
            n_vertices: raw.n_vertices,
            face_wires,
            wires,
            face_edges,
            edge_faces,
            edge_vertices,
            vertex_edges,
        }
    }

    pub(crate) fn n_faces(&self) -> u32 {
        self.n_faces
    }

    pub(crate) fn n_edges(&self) -> u32 {
        self.n_edges
    }

    pub(crate) fn n_vertices(&self) -> u32 {
        self.n_vertices
    }

    pub(crate) fn face_edges(&self, f: FaceId) -> &[EdgeId] {
        self.face_edges.get(f.0 as usize)
    }

    pub(crate) fn face_wires(&self, f: FaceId) -> Vec<Vec<EdgeId>> {
        self.face_wires
            .get(f.0 as usize)
            .iter()
            .map(|&wire| self.wires.get(wire as usize).to_vec())
            .collect()
    }

    pub(crate) fn edge_faces(&self, e: EdgeId) -> &[FaceId] {
        self.edge_faces.get(e.0 as usize)
    }

    pub(crate) fn edge_vertices(&self, e: EdgeId) -> [VertexId; 2] {
        self.edge_vertices[e.0 as usize]
    }

    pub(crate) fn vertex_edges(&self, v: VertexId) -> &[EdgeId] {
        self.vertex_edges.get(v.0 as usize)
    }

    /// Heap bytes of the tables, for `Shape::estimated_bytes`.
    pub(crate) fn heap_bytes(&self) -> u64 {
        fn table<T>(t: &Table<T>) -> usize {
            t.starts.capacity() * size_of::<u32>() + t.items.capacity() * size_of::<T>()
        }
        let total = table(&self.face_wires)
            + table(&self.wires)
            + table(&self.face_edges)
            + table(&self.edge_faces)
            + table(&self.vertex_edges)
            + self.edge_vertices.capacity() * size_of::<[VertexId; 2]>();
        total as u64
    }
}

/// Flattens each face's wires, keeping the first occurrence of every edge.
fn dedup_face_edges(
    n_faces: u32,
    n_edges: u32,
    face_wires: &Table<u32>,
    wires: &Table<EdgeId>,
) -> Table<EdgeId> {
    // `seen[e]` is the last face that listed edge `e`, so each face needs no clearing pass.
    let mut seen = vec![u32::MAX; n_edges as usize];
    let mut table = Table {
        starts: vec![0],
        items: Vec::with_capacity(wires.items.len()),
    };
    for f in 0..n_faces {
        for &wire in face_wires.get(f as usize) {
            for &edge in wires.get(wire as usize) {
                if seen[edge.0 as usize] != f {
                    seen[edge.0 as usize] = f;
                    table.items.push(edge);
                }
            }
        }
        table.starts.push(table.items.len() as u32);
    }
    table
}

/// Builds edge → faces from face → distinct edges. Faces come out ascending.
fn invert(face_edges: &Table<EdgeId>, n_edges: u32) -> Table<FaceId> {
    let mut counts = vec![0u32; n_edges as usize];
    for edge in &face_edges.items {
        counts[edge.0 as usize] += 1;
    }
    let mut starts = Vec::with_capacity(n_edges as usize + 1);
    let mut total = 0u32;
    starts.push(0);
    for count in &counts {
        total += count;
        starts.push(total);
    }
    let mut cursors: Vec<u32> = starts[..n_edges as usize].to_vec();
    let mut items = vec![FaceId(0); total as usize];
    for f in 0..face_edges.slots() {
        for edge in face_edges.get(f) {
            let cursor = &mut cursors[edge.0 as usize];
            items[*cursor as usize] = FaceId(f as u32);
            *cursor += 1;
        }
    }
    Table { starts, items }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two faces sharing edge 1; face 1 has a hole wire, and edge 3 is a seam listed twice.
    fn raw() -> RawTopology {
        RawTopology {
            n_faces: 2,
            n_edges: 5,
            n_vertices: 3,
            face_wire_starts: vec![0, 1, 3],
            wire_edge_starts: vec![0, 2, 6, 7],
            wire_edges: vec![0, 1, 1, 3, 2, 3, 4],
            edge_vertices: vec![0, 1, 1, 2, 2, 0, 0, 0, 1, u32::MAX],
            vertex_edge_starts: vec![0, 3, 6, 8],
            vertex_edges: vec![0, 2, 3, 0, 1, 4, 1, 2],
        }
    }

    #[test]
    fn face_edges_are_distinct_and_in_wire_order() {
        let index = SubShapeIndex::from_raw(raw());
        assert_eq!(index.face_edges(FaceId(0)), [EdgeId(0), EdgeId(1)]);
        assert_eq!(
            index.face_edges(FaceId(1)),
            [EdgeId(1), EdgeId(3), EdgeId(2), EdgeId(4)]
        );
    }

    #[test]
    fn face_wires_keep_seam_duplicates_and_wire_split() {
        let index = SubShapeIndex::from_raw(raw());
        assert_eq!(
            index.face_wires(FaceId(1)),
            vec![
                vec![EdgeId(1), EdgeId(3), EdgeId(2), EdgeId(3)],
                vec![EdgeId(4)]
            ]
        );
    }

    #[test]
    fn edge_faces_are_the_inverse_without_duplicates() {
        let index = SubShapeIndex::from_raw(raw());
        assert_eq!(index.edge_faces(EdgeId(1)), [FaceId(0), FaceId(1)]);
        assert_eq!(index.edge_faces(EdgeId(3)), [FaceId(1)]);
        assert_eq!(index.edge_faces(EdgeId(0)), [FaceId(0)]);
    }

    #[test]
    fn free_edge_has_no_faces() {
        let mut raw = raw();
        raw.n_edges = 6;
        raw.edge_vertices.extend([0, 1]);
        let index = SubShapeIndex::from_raw(raw);
        assert!(index.edge_faces(EdgeId(5)).is_empty());
    }

    #[test]
    fn edge_and_vertex_tables_pass_through() {
        let index = SubShapeIndex::from_raw(raw());
        assert_eq!(index.edge_vertices(EdgeId(1)), [VertexId(1), VertexId(2)]);
        assert_eq!(
            index.edge_vertices(EdgeId(4)),
            [VertexId(1), VertexId(u32::MAX)]
        );
        assert_eq!(index.vertex_edges(VertexId(2)), [EdgeId(1), EdgeId(2)]);
    }
}
