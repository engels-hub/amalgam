//! Hierarchical cancellation tokens.

use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

/// Cooperative cancellation flag. Cloning shares the flag; [`CancelToken::child`] creates a
/// token that is also cancelled when any ancestor is. Cheap to clone; `Send + Sync`.
#[derive(Clone, Debug, Default)]
pub struct CancelToken {
    inner: Arc<TokenInner>,
}

#[derive(Debug, Default)]
struct TokenInner {
    flag: AtomicBool,
    parent: Option<CancelToken>,
}

impl CancelToken {
    /// A fresh, uncancelled root token.
    pub fn new() -> CancelToken {
        CancelToken::default()
    }

    /// A token that is cancelled when `self` (or any of its ancestors) is cancelled, and that
    /// can also be cancelled on its own without affecting `self`.
    pub fn child(&self) -> CancelToken {
        CancelToken {
            inner: Arc::new(TokenInner {
                flag: AtomicBool::new(false),
                parent: Some(self.clone()),
            }),
        }
    }

    /// Requests cancellation of this token and all its descendants. Idempotent.
    pub fn cancel(&self) {
        self.inner.flag.store(true, Ordering::Release);
    }

    /// True once this token or any ancestor has been cancelled.
    pub fn is_cancelled(&self) -> bool {
        let mut t = self;
        loop {
            if t.inner.flag.load(Ordering::Acquire) {
                return true;
            }
            match &t.inner.parent {
                Some(p) => t = p,
                None => return false,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_token_is_not_cancelled() {
        assert!(!CancelToken::new().is_cancelled());
    }

    #[test]
    fn clones_share_the_flag() {
        let a = CancelToken::new();
        let b = a.clone();
        b.cancel();
        assert!(a.is_cancelled());
    }

    #[test]
    fn parent_cancels_descendants_but_not_vice_versa() {
        let root = CancelToken::new();
        let child = root.child();
        let grandchild = child.child();
        grandchild.cancel();
        assert!(grandchild.is_cancelled());
        assert!(!child.is_cancelled());
        assert!(!root.is_cancelled());
        let other = root.child().child();
        root.cancel();
        assert!(child.is_cancelled());
        assert!(other.is_cancelled());
    }
}
