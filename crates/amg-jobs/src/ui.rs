//! UI-thread guard: blocking APIs `debug_assert!` they are not called on the UI thread.

use std::cell::Cell;

thread_local! {
    static IS_UI_THREAD: Cell<bool> = const { Cell::new(false) };
}

/// Marks the current thread as the UI thread (called once by `amg-app`).
///
/// Afterwards the blocking APIs ([`crate::JobHandle::wait_blocking`], [`crate::JobCtx::wait`]
/// outside a pool, [`crate::JobSystem::install`]) panic in debug builds when called here.
pub fn mark_ui_thread() {
    IS_UI_THREAD.with(|c| c.set(true));
}

/// True if [`mark_ui_thread`] was called on the current thread.
pub fn is_ui_thread() -> bool {
    IS_UI_THREAD.with(Cell::get)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn marking_is_per_thread() {
        std::thread::spawn(|| {
            assert!(!is_ui_thread());
            mark_ui_thread();
            assert!(is_ui_thread());
        })
        .join()
        .unwrap();
        assert!(!is_ui_thread());
    }
}
