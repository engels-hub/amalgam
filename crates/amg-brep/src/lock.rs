//! The global I/O lock: category G of S02 §2.
//!
//! OCCT code that touches process-wide state, or is not thread-safe even on separate inputs,
//! runs under this one mutex: IGES read/write (BREP-13), and the process init (message printer,
//! thread pool). On OCCT 8.0.0 text `.brep` and STEP reads needed it too (`Standard_Strtod`
//! race); the 8.0.1 pin fixed that, so they run unlocked.

use std::sync::{Mutex, MutexGuard, PoisonError};

static GLOBAL_IO: Mutex<()> = Mutex::new(());

/// Takes the global lock. A panic while it was held does not poison it: the guarded OCCT
/// calls leave no Rust state behind.
pub(crate) fn global_io_lock() -> MutexGuard<'static, ()> {
    GLOBAL_IO.lock().unwrap_or_else(PoisonError::into_inner)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn lock_serialises_holders() {
        static INSIDE: AtomicUsize = AtomicUsize::new(0);
        static MAX_INSIDE: AtomicUsize = AtomicUsize::new(0);
        std::thread::scope(|scope| {
            for _ in 0..8 {
                scope.spawn(|| {
                    for _ in 0..200 {
                        let _guard = global_io_lock();
                        let now = INSIDE.fetch_add(1, Ordering::SeqCst) + 1;
                        MAX_INSIDE.fetch_max(now, Ordering::SeqCst);
                        std::hint::spin_loop();
                        INSIDE.fetch_sub(1, Ordering::SeqCst);
                    }
                });
            }
        });
        assert_eq!(MAX_INSIDE.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn a_panic_does_not_poison_the_lock() {
        let result = std::thread::spawn(|| {
            let _guard = global_io_lock();
            panic!("panic while holding the global I/O lock");
        })
        .join();
        assert!(result.is_err());
        drop(global_io_lock());
    }
}
