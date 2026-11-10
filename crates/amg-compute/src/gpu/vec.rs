//! Typed storage buffers.

use std::fmt;
use std::marker::PhantomData;

use amg_jobs::gpu::{Gpu, Readback, wgpu};

use crate::{ComputeError, ComputeResult};

const USAGE: wgpu::BufferUsages = wgpu::BufferUsages::STORAGE
    .union(wgpu::BufferUsages::COPY_SRC)
    .union(wgpu::BufferUsages::COPY_DST);

/// Typed storage buffer. Usage always includes `STORAGE | COPY_SRC | COPY_DST`.
///
/// The buffer is at least 4 bytes (and at least one element, so a binding of an `array<T>` is
/// never smaller than one `T`) and a multiple of 4 bytes, as storage bindings require, so an
/// empty vector can still be bound. `arrayLength` in a kernel can therefore exceed [`len`] for
/// an empty vector or an element smaller than 4 bytes; such kernels take the count from a
/// uniform.
///
/// [`len`]: GpuVec::len
pub struct GpuVec<T: bytemuck::Pod> {
    buffer: wgpu::Buffer,
    len: u64,
    _marker: PhantomData<fn() -> T>,
}

impl<T: bytemuck::Pod> fmt::Debug for GpuVec<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GpuVec")
            .field("type", &std::any::type_name::<T>())
            .field("len", &self.len)
            .finish_non_exhaustive()
    }
}

/// Bytes of a `len`-element buffer, rounded up to a multiple of 4, if one binding can hold it.
fn checked_bytes(label: &str, len: u64, elem: u64, limit: u64) -> ComputeResult<u64> {
    let bytes = len
        .checked_mul(elem)
        .and_then(|b| b.checked_next_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT));
    match bytes {
        Some(b) if b <= limit => Ok(b),
        _ => Err(ComputeError::TooLarge {
            label: label.to_owned(),
            bytes: bytes.unwrap_or(u64::MAX),
            limit,
        }),
    }
}

/// The largest buffer a single storage binding of `gpu` can cover.
fn binding_limit(gpu: &Gpu) -> u64 {
    let limits = gpu.device().limits();
    limits
        .max_storage_buffer_binding_size
        .min(limits.max_buffer_size)
}

impl<T: bytemuck::Pod> GpuVec<T> {
    /// A zero-filled vector of `len` elements.
    ///
    /// # Errors
    /// [`ComputeError::TooLarge`] when `len * size_of::<T>()` exceeds the binding limit.
    pub fn zeroed(gpu: &Gpu, label: &str, len: u64) -> ComputeResult<Self> {
        let bytes = checked_bytes(label, len, size_of::<T>() as u64, binding_limit(gpu))?;
        let buffer = gpu.device().create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: bytes.max((size_of::<T>() as u64).max(wgpu::COPY_BUFFER_ALIGNMENT)),
            usage: USAGE,
            mapped_at_creation: false,
        });
        Ok(GpuVec {
            buffer,
            len,
            _marker: PhantomData,
        })
    }

    /// A vector holding a copy of `data`.
    ///
    /// # Errors
    /// [`ComputeError::TooLarge`] when `data` exceeds the binding limit.
    pub fn from_slice(gpu: &Gpu, label: &str, data: &[T]) -> ComputeResult<Self> {
        if data.is_empty() {
            return Self::zeroed(gpu, label, 0);
        }
        let len = data.len() as u64;
        checked_bytes(label, len, size_of::<T>() as u64, binding_limit(gpu))?;
        Ok(GpuVec {
            buffer: gpu.upload(label, data, USAGE),
            len,
            _marker: PhantomData,
        })
    }

    /// Number of elements.
    pub fn len(&self) -> u64 {
        self.len
    }

    /// True when the vector has no elements.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The underlying buffer.
    pub fn buffer(&self) -> &wgpu::Buffer {
        &self.buffer
    }

    /// The whole buffer as a bind-group resource.
    pub fn binding(&self) -> wgpu::BindingResource<'_> {
        self.buffer.as_entire_binding()
    }

    /// Writes `data` at element `offset` (`queue.write_buffer`; takes effect before the next
    /// submission).
    ///
    /// # Panics
    /// If the write runs past the end of the vector, or its start or length in bytes is not a
    /// multiple of 4 (only possible for elements smaller than 4 bytes).
    pub fn write(&self, gpu: &Gpu, offset: u64, data: &[T]) {
        if data.is_empty() {
            return;
        }
        let bytes = self.byte_range(offset..offset + data.len() as u64);
        gpu.queue()
            .write_buffer(&self.buffer, bytes.start, bytemuck::cast_slice(data));
    }

    /// Reads the whole vector. Drive the result with `try_get` or `wait`.
    ///
    /// # Panics
    /// As [`GpuVec::read_range`]; in particular if the vector is empty.
    pub fn read(&self, gpu: &Gpu) -> Readback<T> {
        self.read_range(gpu, 0..self.len)
    }

    /// Reads the elements in `range`. Include any pending `ComputeCtx` work first (`submit`).
    ///
    /// # Panics
    /// If `range` is empty, runs past the end of the vector, or its start or length in bytes is
    /// not a multiple of 4 (`Gpu::readback` has no zero-length form).
    pub fn read_range(&self, gpu: &Gpu, range: std::ops::Range<u64>) -> Readback<T> {
        let bytes = self.byte_range(range);
        gpu.readback(&self.buffer, bytes)
    }

    /// Byte range of the element range `r`.
    fn byte_range(&self, r: std::ops::Range<u64>) -> std::ops::Range<u64> {
        assert!(
            r.start <= r.end && r.end <= self.len,
            "element range {r:?} is outside a vector of {} elements",
            self.len
        );
        let elem = size_of::<T>() as u64;
        let (start, end) = (r.start * elem, r.end * elem);
        assert!(
            start.is_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT)
                && (end - start).is_multiple_of(wgpu::COPY_BUFFER_ALIGNMENT),
            "byte range {start}..{end} must start and end on 4-byte boundaries"
        );
        start..end
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checked_bytes_rounds_up_to_4() {
        assert_eq!(checked_bytes("v", 0, 4, 100).unwrap(), 0);
        assert_eq!(checked_bytes("v", 5, 1, 100).unwrap(), 8);
        assert_eq!(checked_bytes("v", 3, 2, 100).unwrap(), 8);
        assert_eq!(checked_bytes("v", 25, 4, 100).unwrap(), 100);
    }

    #[test]
    fn checked_bytes_past_the_limit_is_too_large() {
        let e = checked_bytes("v", 26, 4, 100).unwrap_err();
        let ComputeError::TooLarge {
            label,
            bytes,
            limit,
        } = &e
        else {
            panic!("expected TooLarge, got {e:?}");
        };
        assert_eq!((label.as_str(), *bytes, *limit), ("v", 104, 100));
        // A byte that rounds up past the limit is rejected too.
        assert!(checked_bytes("v", 101, 1, 103).is_err());
    }

    #[test]
    fn checked_bytes_overflow_is_too_large() {
        let e = checked_bytes("v", u64::MAX, 8, u64::MAX).unwrap_err();
        assert!(
            matches!(
                e,
                ComputeError::TooLarge {
                    bytes: u64::MAX,
                    ..
                }
            ),
            "{e:?}"
        );
        let e = checked_bytes("v", u64::MAX, 1, u64::MAX).unwrap_err();
        assert!(
            matches!(
                e,
                ComputeError::TooLarge {
                    bytes: u64::MAX,
                    ..
                }
            ),
            "{e:?}"
        );
    }
}
