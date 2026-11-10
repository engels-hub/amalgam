//! The dispatch context: one encoder per batch, a uniform arena and per-dispatch timestamps.

use std::cell::Cell;
use std::collections::VecDeque;
use std::fmt;
use std::marker::PhantomData;
use std::num::NonZeroU64;
use std::sync::Arc;
use std::time::Duration;

use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuError, Readback, wgpu};

use super::kernel::Kernel;
use super::vec::GpuVec;
use crate::ComputeResult;

/// Uniform slots are this many bytes (the largest `min_uniform_buffer_offset_alignment` allowed).
const SLOT: u64 = 256;
/// Bytes per arena buffer: 256 slots. Chaining buffers keeps earlier bind groups valid when the
/// arena grows.
const CHUNK: u64 = 256 * SLOT;
/// A batch is submitted once it holds this many dispatches. Two timestamps per dispatch must
/// fit one query set (`wgpu::QUERY_SET_MAX_QUERIES` = 4096).
const MAX_BATCH_DISPATCHES: u32 = 1024;
/// `max_compute_workgroups_per_dimension` of WebGPU and DX12.
const MAX_WORKGROUPS_PER_DIM: u64 = 65_535;
/// Timing batches kept for `take_timings`. They are diagnostics, so a solver that never asks
/// for them must not accumulate buffers without bound; the oldest are dropped.
const MAX_PENDING_TIMING_BATCHES: usize = 256;
/// Longest single wait in `finish`: bounds how late cancellation and device loss are noticed.
const FINISH_POLL: Duration = Duration::from_millis(50);

/// A sub-allocation of the uniform arena, from [`ComputeCtx::uniform`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UniformSlot {
    /// Byte offset in the arena (across its chained buffers).
    pub offset: u64,
    /// Size of the parameters in bytes.
    pub size: u64,
}

/// `CPU bytes -> GPU chunks`. Bytes are uploaded when a batch is submitted, which orders them
/// before that batch on the queue, so a chunk can be reused after `submit` while older batches
/// are still in flight.
#[derive(Debug, Default)]
struct UniformArena {
    chunks: Vec<wgpu::Buffer>,
    /// Everything since the last reset, in slots of `SLOT` bytes.
    cpu: Vec<u8>,
    /// The prefix of `cpu` that has been written to the GPU.
    flushed: usize,
}

impl UniformArena {
    fn alloc(&mut self, device: &wgpu::Device, bytes: &[u8]) -> UniformSlot {
        let size = bytes.len() as u64;
        assert!(
            size > 0 && size <= CHUNK,
            "uniform parameters must be 1..={CHUNK} bytes, got {size}"
        );
        let rounded = size.next_multiple_of(SLOT);
        let mut offset = self.cpu.len() as u64;
        // A binding must lie inside one chunk.
        let room = CHUNK - offset % CHUNK;
        if rounded > room {
            offset += room;
        }
        self.cpu.resize((offset + rounded) as usize, 0);
        self.cpu[offset as usize..(offset + size) as usize].copy_from_slice(bytes);
        let chunk = (offset / CHUNK) as usize;
        while self.chunks.len() <= chunk {
            self.chunks
                .push(device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("amg compute uniforms"),
                    size: CHUNK,
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                }));
        }
        UniformSlot { offset, size }
    }

    fn binding(&self, slot: UniformSlot) -> wgpu::BindingResource<'_> {
        wgpu::BindingResource::Buffer(wgpu::BufferBinding {
            buffer: &self.chunks[(slot.offset / CHUNK) as usize],
            offset: slot.offset % CHUNK,
            size: NonZeroU64::new(slot.size),
        })
    }

    /// Uploads the bytes not written yet; they stay allocated until `reset`.
    fn upload(&mut self, queue: &wgpu::Queue) {
        let chunk_bytes = CHUNK as usize;
        let mut start = self.flushed;
        while start < self.cpu.len() {
            let index = start / chunk_bytes;
            let stop = ((index + 1) * chunk_bytes).min(self.cpu.len());
            queue.write_buffer(
                &self.chunks[index],
                (start % chunk_bytes) as u64,
                &self.cpu[start..stop],
            );
            start = stop;
        }
        self.flushed = self.cpu.len();
    }

    fn reset(&mut self) {
        self.cpu.clear();
        self.flushed = 0;
    }
}

#[derive(Debug)]
struct PendingTimings {
    labels: Vec<Arc<str>>,
    ticks: Readback<u64>,
}

/// Per-dispatch timestamps; exists only on a device with `TIMESTAMP_QUERY`.
#[derive(Debug)]
struct Timing {
    query_set: wgpu::QuerySet,
    resolve: wgpu::Buffer,
    /// The dispatches of the current batch; dispatch `i` wrote queries `2i` and `2i + 1`.
    labels: Vec<Arc<str>>,
    pending: VecDeque<PendingTimings>,
    nanos_per_tick: f64,
}

impl Timing {
    fn new(gpu: &Gpu) -> Timing {
        let queries = 2 * MAX_BATCH_DISPATCHES;
        Timing {
            query_set: gpu.device().create_query_set(&wgpu::QuerySetDescriptor {
                label: Some("amg compute timestamps"),
                ty: wgpu::QueryType::Timestamp,
                count: queries,
            }),
            resolve: gpu.device().create_buffer(&wgpu::BufferDescriptor {
                label: Some("amg compute timestamps"),
                size: u64::from(queries) * u64::from(wgpu::QUERY_SIZE),
                usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            }),
            labels: Vec::new(),
            pending: VecDeque::new(),
            nanos_per_tick: f64::from(gpu.queue().get_timestamp_period()),
        }
    }
}

/// Records dispatches into one encoder; submits in batches. Not Sync; one per job.
pub struct ComputeCtx {
    gpu: Arc<Gpu>,
    label: String,
    encoder: Option<wgpu::CommandEncoder>,
    batch_dispatches: u32,
    arena: UniformArena,
    timing: Option<Timing>,
    last_submission: Option<wgpu::SubmissionIndex>,
    _not_sync: PhantomData<Cell<()>>,
}

impl fmt::Debug for ComputeCtx {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ComputeCtx")
            .field("label", &self.label)
            .field("batch_dispatches", &self.batch_dispatches)
            .field("uniform_bytes", &self.arena.cpu.len())
            .field("timestamps", &self.timing.is_some())
            .finish_non_exhaustive()
    }
}

/// Workgroup counts that launch at least `n` threads of a kernel with `wg` threads along x, in
/// the fewest columns of 65 535 workgroups; `None` for `n == 0`.
///
/// # Panics
/// If `n` does not fit the `u32` index of `amg_linear_id`, or the launch needs more than 65 535
/// rows (only possible with a workgroup of very few threads).
fn workgroups_1d(n: u64, wg: u32) -> Option<[u32; 3]> {
    if n == 0 {
        return None;
    }
    assert!(
        n <= u64::from(u32::MAX),
        "dispatch_1d of {n} elements: amg_linear_id indexes with u32"
    );
    let groups = n.div_ceil(u64::from(wg));
    if groups <= MAX_WORKGROUPS_PER_DIM {
        return Some([groups as u32, 1, 1]);
    }
    let rows = groups.div_ceil(MAX_WORKGROUPS_PER_DIM);
    assert!(
        rows <= MAX_WORKGROUPS_PER_DIM,
        "dispatch_1d of {n} elements with {wg} threads per workgroup needs {rows} rows"
    );
    Some([MAX_WORKGROUPS_PER_DIM as u32, rows as u32, 1])
}

impl ComputeCtx {
    /// An empty context on `gpu`; `label` names its command encoders.
    pub fn new(gpu: Arc<Gpu>, label: &str) -> Self {
        let timing = gpu
            .has(wgpu::Features::TIMESTAMP_QUERY)
            .then(|| Timing::new(&gpu));
        ComputeCtx {
            gpu,
            label: label.to_owned(),
            encoder: None,
            batch_dispatches: 0,
            arena: UniformArena::default(),
            timing,
            last_submission: None,
            _not_sync: PhantomData,
        }
    }

    /// The device this context records for.
    pub fn gpu(&self) -> &Gpu {
        &self.gpu
    }

    /// A shared handle to the device this context records for.
    pub fn gpu_arc(&self) -> Arc<Gpu> {
        Arc::clone(&self.gpu)
    }

    /// Copies `params` into the uniform arena (256-byte slots); valid until `submit`.
    ///
    /// # Panics
    /// If `P` is empty or larger than 64 KiB.
    pub fn uniform<P: bytemuck::Pod>(&mut self, params: &P) -> UniformSlot {
        self.arena
            .alloc(self.gpu.device(), bytemuck::bytes_of(params))
    }

    /// The bind-group resource of `slot`.
    ///
    /// # Panics
    /// If `slot` did not come from this context.
    pub fn uniform_binding(&self, slot: UniformSlot) -> wgpu::BindingResource<'_> {
        self.arena.binding(slot)
    }

    /// Records one dispatch. Each dispatch is its own compute pass, so timestamps can be taken
    /// per dispatch. A dispatch with a zero count records nothing.
    pub fn dispatch(&mut self, k: &Kernel, groups: &[&wgpu::BindGroup], workgroups: [u32; 3]) {
        if workgroups.contains(&0) {
            return;
        }
        let label = &self.label;
        let device = self.gpu.device();
        let encoder = self.encoder.get_or_insert_with(|| {
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some(label) })
        });
        let timestamp_writes = self.timing.as_ref().map(|t| {
            let first = 2 * t.labels.len() as u32;
            wgpu::ComputePassTimestampWrites {
                query_set: &t.query_set,
                beginning_of_pass_write_index: Some(first),
                end_of_pass_write_index: Some(first + 1),
            }
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some(&k.label),
                timestamp_writes,
            });
            pass.set_pipeline(&k.pipeline);
            for (i, group) in groups.iter().enumerate() {
                pass.set_bind_group(i as u32, *group, &[]);
            }
            pass.dispatch_workgroups(workgroups[0], workgroups[1], workgroups[2]);
        }
        if let Some(t) = &mut self.timing {
            t.labels.push(Arc::clone(&k.label));
        }
        self.batch_dispatches += 1;
        if self.batch_dispatches >= MAX_BATCH_DISPATCHES {
            self.flush();
        }
    }

    /// One thread per element for `n` elements; folds into 2-D past 65 535 workgroups. The
    /// launch rounds up to whole workgroups, so the kernel must ignore indices `>= n`.
    /// `n == 0` records nothing.
    ///
    /// # Panics
    /// If `n` exceeds `u32::MAX` (the index of `amg_linear_id`).
    pub fn dispatch_1d(&mut self, k: &Kernel, groups: &[&wgpu::BindGroup], n: u64) {
        if let Some(workgroups) = workgroups_1d(n, k.workgroup()[0]) {
            self.dispatch(k, groups, workgroups);
        }
    }

    /// Records a copy of all of `src` to the start of `dst`.
    ///
    /// # Panics
    /// If `src` is longer than `dst`.
    pub fn copy<T: bytemuck::Pod>(&mut self, src: &GpuVec<T>, dst: &GpuVec<T>) {
        assert!(
            src.len() <= dst.len(),
            "copy of {} elements into a vector of {}",
            src.len(),
            dst.len()
        );
        if src.is_empty() {
            return;
        }
        let label = &self.label;
        let device = self.gpu.device();
        let encoder = self.encoder.get_or_insert_with(|| {
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some(label) })
        });
        let bytes = src.len() * size_of::<T>() as u64;
        encoder.copy_buffer_to_buffer(src.buffer(), 0, dst.buffer(), 0, bytes);
    }

    /// Submits the recorded work and releases the uniform slots.
    pub fn submit(&mut self) {
        self.flush();
        self.arena.reset();
    }

    /// Submits the batch without releasing uniform slots, so an automatic submission between a
    /// caller's `uniform` and its `dispatch` is invisible to the caller.
    fn flush(&mut self) {
        self.batch_dispatches = 0;
        let encoder = self.encoder.take();
        let labels = self
            .timing
            .as_mut()
            .map(|t| std::mem::take(&mut t.labels))
            .unwrap_or_default();
        if self.gpu.is_lost() {
            return; // `finish` reports it; the commands can never run.
        }
        self.arena.upload(self.gpu.queue());
        let Some(mut encoder) = encoder else {
            return;
        };
        let queries = 2 * labels.len() as u32;
        if let (Some(t), true) = (&self.timing, queries > 0) {
            encoder.resolve_query_set(&t.query_set, 0..queries, &t.resolve, 0);
        }
        self.last_submission = Some(self.gpu.queue().submit([encoder.finish()]));
        if let (Some(t), true) = (&mut self.timing, queries > 0) {
            let bytes = u64::from(queries) * u64::from(wgpu::QUERY_SIZE);
            if t.pending.len() == MAX_PENDING_TIMING_BATCHES {
                t.pending.pop_front();
            }
            // `Device::destroy` invalidates the device before the lost flag is set (on a later
            // poll). In that window `map_async` raises a validation error, which would panic in
            // the default handler; the scope (popped on drop) discards it. The readback then
            // reports the loss when it is waited on.
            let _scope = self
                .gpu
                .device()
                .push_error_scope(wgpu::ErrorFilter::Validation);
            t.pending.push_back(PendingTimings {
                labels,
                ticks: self.gpu.readback(&t.resolve, 0..bytes),
            });
        }
    }

    /// Submits and blocks until the GPU has finished this context's work. Job threads only
    /// (never the UI thread).
    ///
    /// # Errors
    /// [`ComputeError::Cancelled`](crate::ComputeError::Cancelled) when the job is cancelled
    /// (the submitted work still runs), and
    /// [`ComputeError::Gpu`](crate::ComputeError::Gpu) with `DeviceLost` or `Poll`.
    pub fn finish(&mut self, ctx: &JobCtx) -> ComputeResult<()> {
        debug_assert!(
            !amg_jobs::is_ui_thread(),
            "ComputeCtx::finish blocks; call it on a job thread"
        );
        self.submit();
        loop {
            ctx.checkpoint()?;
            self.gpu.poll()?;
            // A bounded wait, so cancellation and device loss are noticed while work runs.
            match self.gpu.device().poll(wgpu::PollType::Wait {
                submission_index: self.last_submission.clone(),
                timeout: Some(FINISH_POLL),
            }) {
                Ok(_) => break,
                Err(wgpu::PollError::Timeout) => {}
                Err(e) => return Err(GpuError::Poll(e.to_string()).into()),
            }
        }
        // The device-lost callback runs during a poll, so check once more after the wait.
        self.gpu.poll()?;
        Ok(())
    }

    /// GPU time per labelled dispatch since the last call, in submission order; empty without
    /// `TIMESTAMP_QUERY`. Only submitted dispatches are reported, from at most the last
    /// 256 batches. Waits for the timestamps to arrive, so call it on a job thread.
    pub fn take_timings(&mut self) -> Vec<(String, Duration)> {
        let Some(t) = &mut self.timing else {
            return Vec::new();
        };
        let mut timings = Vec::new();
        for batch in t.pending.drain(..) {
            // Without the device (lost) the timings of this batch simply do not exist.
            let Ok(ticks) = batch.ticks.wait() else {
                continue;
            };
            for (label, pair) in batch.labels.iter().zip(ticks.chunks_exact(2)) {
                let nanos = pair[1].saturating_sub(pair[0]) as f64 * t.nanos_per_tick;
                timings.push((label.to_string(), Duration::from_nanos(nanos as u64)));
            }
        }
        timings
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_elements_launch_nothing() {
        assert_eq!(workgroups_1d(0, 256), None);
    }

    #[test]
    fn small_launches_stay_one_dimensional() {
        assert_eq!(workgroups_1d(1, 256), Some([1, 1, 1]));
        assert_eq!(workgroups_1d(255, 256), Some([1, 1, 1]));
        assert_eq!(workgroups_1d(256, 256), Some([1, 1, 1]));
        assert_eq!(workgroups_1d(257, 256), Some([2, 1, 1]));
        assert_eq!(workgroups_1d(65_535 * 256, 256), Some([65_535, 1, 1]));
    }

    #[test]
    fn large_launches_fold_into_two_dimensions() {
        assert_eq!(workgroups_1d(65_535 * 256 + 1, 256), Some([65_535, 2, 1]));
        let [x, y, z] = workgroups_1d((1 << 26) + 3, 256).unwrap();
        assert_eq!((x, y, z), (65_535, 5, 1));
        assert!(u64::from(x) * u64::from(y) * 256 >= (1 << 26) + 3);
        assert_eq!(
            workgroups_1d(u64::from(u32::MAX), 256),
            Some([65_535, 257, 1])
        );
    }

    #[test]
    #[should_panic(expected = "amg_linear_id indexes with u32")]
    fn more_than_u32_elements_panics() {
        workgroups_1d(1 << 32, 256);
    }
}
