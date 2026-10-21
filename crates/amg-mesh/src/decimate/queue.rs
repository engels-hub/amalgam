//! Indexed binary heap over vertices, keyed by each vertex's cheapest collapse candidate.
//!
//! Every vertex stores its best incident edge `(cost, lo, hi)`; the heap orders vertices by
//! `(cost, lo, hi, vertex)`, a strict total order, so the pop sequence is deterministic. A
//! collapse updates a handful of vertices in place instead of pushing duplicate entries.

/// Cheapest collapse candidate of one vertex. `lo == u32::MAX` means none.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Best {
    /// `f32` bits of the non-negative cost (orders like the value).
    pub(super) cost: u32,
    pub(super) lo: u32,
    pub(super) hi: u32,
}

impl Best {
    pub(super) const NONE: Best = Best {
        cost: u32::MAX,
        lo: u32::MAX,
        hi: u32::MAX,
    };

    pub(super) fn new(cost: u32, a: u32, b: u32) -> Best {
        Best {
            cost,
            lo: a.min(b),
            hi: a.max(b),
        }
    }

    pub(super) fn is_none(&self) -> bool {
        self.lo == u32::MAX
    }

    pub(super) fn sort_key(&self) -> (u32, u32, u32) {
        (self.cost, self.lo, self.hi)
    }
}

const ABSENT: u32 = u32::MAX;

pub(super) struct VertexQueue {
    best: Vec<Best>,
    heap: Vec<u32>,
    pos: Vec<u32>,
}

impl VertexQueue {
    pub(super) fn build(best: Vec<Best>) -> VertexQueue {
        let mut q = VertexQueue {
            pos: vec![ABSENT; best.len()],
            heap: Vec::with_capacity(best.len()),
            best,
        };
        for v in 0..q.best.len() {
            if !q.best[v].is_none() {
                q.pos[v] = q.heap.len() as u32;
                q.heap.push(v as u32);
            }
        }
        for i in (0..q.heap.len() / 2).rev() {
            q.sift_down(i);
        }
        q
    }

    pub(super) fn best(&self, v: u32) -> Best {
        self.best[v as usize]
    }

    /// The vertex with the cheapest candidate.
    pub(super) fn peek(&self) -> Option<(u32, Best)> {
        self.heap.first().map(|&v| (v, self.best[v as usize]))
    }

    fn less(&self, x: u32, y: u32) -> bool {
        (self.best[x as usize].sort_key(), x) < (self.best[y as usize].sort_key(), y)
    }

    fn sift_up(&mut self, mut i: usize) {
        let v = self.heap[i];
        while i > 0 {
            let p = (i - 1) / 2;
            let pv = self.heap[p];
            if !self.less(v, pv) {
                break;
            }
            self.heap[i] = pv;
            self.pos[pv as usize] = i as u32;
            i = p;
        }
        self.heap[i] = v;
        self.pos[v as usize] = i as u32;
    }

    fn sift_down(&mut self, mut i: usize) {
        let n = self.heap.len();
        let v = self.heap[i];
        loop {
            let l = 2 * i + 1;
            if l >= n {
                break;
            }
            let r = l + 1;
            let c = if r < n && self.less(self.heap[r], self.heap[l]) {
                r
            } else {
                l
            };
            let cv = self.heap[c];
            if !self.less(cv, v) {
                break;
            }
            self.heap[i] = cv;
            self.pos[cv as usize] = i as u32;
            i = c;
        }
        self.heap[i] = v;
        self.pos[v as usize] = i as u32;
    }

    /// Replaces the candidate of `v`, inserting or removing it as needed.
    pub(super) fn set(&mut self, v: u32, b: Best) {
        if b.is_none() {
            self.remove(v);
            return;
        }
        self.best[v as usize] = b;
        let p = self.pos[v as usize];
        if p == ABSENT {
            self.pos[v as usize] = self.heap.len() as u32;
            self.heap.push(v);
            self.sift_up(self.heap.len() - 1);
        } else {
            self.sift_up(p as usize);
            let p = self.pos[v as usize] as usize;
            self.sift_down(p);
        }
    }

    pub(super) fn remove(&mut self, v: u32) {
        self.best[v as usize] = Best::NONE;
        let p = self.pos[v as usize];
        if p == ABSENT {
            return;
        }
        self.pos[v as usize] = ABSENT;
        let last = self.heap.pop();
        if let Some(last) = last
            && last != v
        {
            self.heap[p as usize] = last;
            self.pos[last as usize] = p;
            self.sift_up(p as usize);
            let p = self.pos[last as usize] as usize;
            self.sift_down(p);
        }
    }
}
