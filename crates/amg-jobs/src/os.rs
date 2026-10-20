//! OS thread priority for the Background pool.
//!
//! On Windows this is a minimal FFI to `kernel32` (`GetCurrentThread`, `SetThreadPriority`,
//! `GetThreadPriority`), which std already links. Other platforms are a no-op for now
//! (Amalgam targets Windows x64; see S00 §1).

/// Lowers the calling thread's OS priority to below-normal. Returns `false` if the OS refused
/// or the platform is unsupported; the pool still works, just without the priority hint.
pub(crate) fn lower_current_thread_priority() -> bool {
    imp::lower_current_thread_priority()
}

/// The calling thread's OS priority (Windows `THREAD_PRIORITY_*` value), if known.
#[cfg(test)]
pub(crate) fn current_thread_priority() -> Option<i32> {
    imp::current_thread_priority()
}

/// Windows `THREAD_PRIORITY_BELOW_NORMAL`.
#[cfg(test)]
pub(crate) const BELOW_NORMAL: i32 = -1;

#[cfg(windows)]
#[allow(unsafe_code)] // Documented FFI boundary (AGENTS.md conventions).
mod imp {
    use std::ffi::c_void;

    /// `THREAD_PRIORITY_BELOW_NORMAL` from `processthreadsapi.h`.
    const THREAD_PRIORITY_BELOW_NORMAL: i32 = -1;
    /// `THREAD_PRIORITY_ERROR_RETURN` (`MAXLONG`).
    #[cfg(test)]
    const THREAD_PRIORITY_ERROR_RETURN: i32 = i32::MAX;

    #[link(name = "kernel32")]
    unsafe extern "system" {
        /// Returns a pseudo-handle for the calling thread; never fails, needs no closing.
        fn GetCurrentThread() -> *mut c_void;
        /// Returns non-zero on success.
        fn SetThreadPriority(thread: *mut c_void, priority: i32) -> i32;
        /// Returns `THREAD_PRIORITY_ERROR_RETURN` on failure.
        #[cfg(test)]
        fn GetThreadPriority(thread: *mut c_void) -> i32;
    }

    pub(super) fn lower_current_thread_priority() -> bool {
        // SAFETY: `GetCurrentThread` takes no arguments and returns a pseudo-handle that is
        // always valid for the calling thread. `SetThreadPriority` is called with that handle
        // and a documented priority constant; it has no memory-safety preconditions and only
        // reports failure through its return value.
        unsafe { SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL) != 0 }
    }

    #[cfg(test)]
    pub(super) fn current_thread_priority() -> Option<i32> {
        // SAFETY: as above; `GetThreadPriority` only reads the priority of the thread named by
        // the always-valid pseudo-handle.
        let p = unsafe { GetThreadPriority(GetCurrentThread()) };
        (p != THREAD_PRIORITY_ERROR_RETURN).then_some(p)
    }
}

#[cfg(not(windows))]
mod imp {
    pub(super) fn lower_current_thread_priority() -> bool {
        false
    }

    #[cfg(test)]
    pub(super) fn current_thread_priority() -> Option<i32> {
        None
    }
}
