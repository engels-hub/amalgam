//! Non-blocking GPU -> CPU copies.

use std::fmt;
use std::marker::PhantomData;
use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::Duration;

use super::{Gpu, GpuError, LostState, poll_device};

const PENDING: u8 = 0;
const MAPPED: u8 = 1;
const FAILED: u8 = 2;

/// In-flight GPU -> CPU copy, from [`Gpu::readback`].
///
/// Poll it with [`Readback::try_get`] (never blocks; fine on the UI thread) or block on
/// [`Readback::wait`] (job threads only). Dropping it before completion is fine.
pub struct Readback<T> {
    device: wgpu::Device,
    lost: Arc<LostState>,
    /// `None` when the device was already lost at creation (no wgpu call was made).
    staging: Option<wgpu::Buffer>,
    submission: Option<wgpu::SubmissionIndex>,
    status: Arc<AtomicU8>,
    taken: bool,
    _marker: PhantomData<fn() -> T>,
}

impl<T> fmt::Debug for Readback<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Readback")
            .field("bytes", &self.staging.as_ref().map(wgpu::Buffer::size))
            .field("status", &self.status.load(Ordering::Acquire))
            .field("taken", &self.taken)
            .finish_non_exhaustive()
    }
}

impl<T: bytemuck::Pod> Readback<T> {
    pub(super) fn start(gpu: &Gpu, src: &wgpu::Buffer, range: std::ops::Range<u64>) -> Self {
        let elem = size_of::<T>() as u64;
        assert!(
            range.start < range.end,
            "readback range {range:?} is empty or reversed"
        );
        let len = range.end - range.start;
        assert!(
            range.start.is_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT)
                && len.is_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT),
            "readback range {range:?} must start and end on 4-byte boundaries"
        );
        assert!(
            elem != 0 && len.is_multiple_of(elem),
            "readback length {len} is not a multiple of the element size {elem}"
        );

        let device = gpu.device().clone();
        if gpu.lost_state().is_lost() {
            return Readback {
                device,
                lost: Arc::clone(gpu.lost_state()),
                staging: None,
                submission: None,
                status: Arc::new(AtomicU8::new(FAILED)),
                taken: false,
                _marker: PhantomData,
            };
        }
        let staging = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("amg readback"),
            size: len,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("amg readback"),
        });
        enc.copy_buffer_to_buffer(src, range.start, &staging, 0, len);
        let submission = gpu.queue().submit([enc.finish()]);

        let status = Arc::new(AtomicU8::new(PENDING));
        let done = Arc::clone(&status);
        staging.map_async(wgpu::MapMode::Read, .., move |r| {
            done.store(if r.is_ok() { MAPPED } else { FAILED }, Ordering::Release);
        });

        Readback {
            device,
            lost: Arc::clone(gpu.lost_state()),
            staging: Some(staging),
            submission: Some(submission),
            status,
            taken: false,
            _marker: PhantomData,
        }
    }

    /// Non-blocking: polls the device once. `Some` when the copy has mapped (or failed);
    /// `None` while it is in flight, and on every call after the result was returned.
    pub fn try_get(&mut self) -> Option<Result<Vec<T>, GpuError>> {
        if self.taken {
            return None;
        }
        if self.status.load(Ordering::Acquire) == PENDING
            && let Err(e) = poll_device(&self.device, &self.lost)
        {
            self.taken = true;
            return Some(Err(e));
        }
        match self.status.load(Ordering::Acquire) {
            PENDING => None,
            _ => {
                self.taken = true;
                Some(self.take())
            }
        }
    }

    /// Blocks until the copy has mapped. `debug_assert!(!is_ui_thread())`: job threads only.
    ///
    /// # Errors
    /// [`GpuError::DeviceLost`], [`GpuError::Poll`] or [`GpuError::Map`].
    pub fn wait(mut self) -> Result<Vec<T>, GpuError> {
        debug_assert!(
            !crate::is_ui_thread(),
            "Readback::wait blocks; use try_get on the UI thread"
        );
        while self.status.load(Ordering::Acquire) == PENDING {
            self.lost.check()?;
            // A bounded wait, so device loss is noticed even if the submission never finishes.
            match self.device.poll(wgpu::PollType::Wait {
                submission_index: self.submission.clone(),
                timeout: Some(Duration::from_millis(100)),
            }) {
                Ok(_) | Err(wgpu::PollError::Timeout) => {}
                Err(e) => return Err(GpuError::Poll(e.to_string())),
            }
            if self.status.load(Ordering::Acquire) == PENDING {
                // The submission finished but the map callback runs on the next poll.
                poll_device(&self.device, &self.lost)?;
            }
        }
        self.taken = true;
        self.take()
    }

    fn take(&self) -> Result<Vec<T>, GpuError> {
        if self.status.load(Ordering::Acquire) != MAPPED {
            self.lost.check()?;
            return Err(GpuError::Map(wgpu::BufferAsyncError));
        }
        let staging = self
            .staging
            .as_ref()
            .ok_or_else(|| GpuError::DeviceLost(String::new()))?;
        let out = {
            let view = staging
                .get_mapped_range(..)
                .map_err(|_| GpuError::Map(wgpu::BufferAsyncError))?;
            // `pod_collect_to_vec` copies, so the staging bytes need no particular alignment.
            bytemuck::pod_collect_to_vec::<u8, T>(&view)
        };
        staging.unmap();
        Ok(out)
    }
}
