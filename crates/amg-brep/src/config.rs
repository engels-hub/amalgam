//! `KernelConfig` and the validation policy (S02 §7, §9).

use std::num::NonZeroUsize;

use amg_core::Tolerance;

/// Environment variable read by [`KernelConfig::from_env`]: `off`, `debug` or `always`.
pub const ENV_VALIDATE: &str = "AMG_KERNEL_VALIDATE";
/// Environment variable read by [`KernelConfig::from_env`]: OCCT pool threads, at least 1.
pub const ENV_OCCT_THREADS: &str = "AMG_OCCT_THREADS";

/// When op results are checked with `BRepCheck` (S02 §9).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum Validate {
    /// Never.
    Off,
    /// In builds with debug assertions.
    #[default]
    DebugOnly,
    /// Always; an invalid result becomes `KernelError::InvalidResult`.
    Always,
}

impl Validate {
    /// Whether this policy checks results in the current build.
    pub fn is_enabled(self) -> bool {
        match self {
            Validate::Off => false,
            Validate::DebugOnly => cfg!(debug_assertions),
            Validate::Always => true,
        }
    }

    /// Parses `off`, `debug` (or `debug-only`, `debugonly`) and `always`, any case.
    pub fn parse(text: &str) -> Option<Validate> {
        match text.trim().to_ascii_lowercase().as_str() {
            "off" | "0" | "false" | "never" => Some(Validate::Off),
            "debug" | "debug-only" | "debugonly" | "debug_only" => Some(Validate::DebugOnly),
            "always" | "on" | "1" | "true" => Some(Validate::Always),
            _ => None,
        }
    }
}

/// Process and per-kernel settings of the OCCT backend.
#[derive(Clone, Debug, PartialEq)]
pub struct KernelConfig {
    /// Result validation policy.
    pub validate: Validate,
    /// Size of OCCT's own thread pool (`OSD_ThreadPool`); `None` = `max(1, cores / 2)` (S02
    /// §7). Process-wide: the first [`crate::init`] call sets it.
    pub occt_threads: Option<NonZeroUsize>,
    /// Default of `BoolOpts::parallel`. Off until BREP-04's determinism test sets it (S02 §7).
    pub bool_parallel: bool,
    /// Model tolerance.
    pub tolerance: Tolerance,
}

impl Default for KernelConfig {
    fn default() -> Self {
        KernelConfig {
            validate: Validate::default(),
            occt_threads: None,
            bool_parallel: false,
            tolerance: Tolerance::DEFAULT,
        }
    }
}

impl KernelConfig {
    /// The default, with [`ENV_VALIDATE`] and [`ENV_OCCT_THREADS`] applied. Unparsable values
    /// are ignored with a warning.
    pub fn from_env() -> Self {
        let mut cfg = KernelConfig::default();
        cfg.apply_env(
            std::env::var(ENV_VALIDATE).ok().as_deref(),
            std::env::var(ENV_OCCT_THREADS).ok().as_deref(),
        );
        cfg
    }

    fn apply_env(&mut self, validate: Option<&str>, threads: Option<&str>) {
        if let Some(text) = validate {
            match Validate::parse(text) {
                Some(policy) => self.validate = policy,
                None => {
                    tracing::warn!("ignoring {ENV_VALIDATE}={text:?}: use off, debug or always")
                }
            }
        }
        if let Some(text) = threads {
            match text.trim().parse::<NonZeroUsize>() {
                Ok(n) => self.occt_threads = Some(n),
                Err(_) => tracing::warn!("ignoring {ENV_OCCT_THREADS}={text:?}: not a count >= 1"),
            }
        }
    }

    /// `occt_threads`, or `max(1, cores / 2)` when unset.
    pub fn effective_occt_threads(&self) -> NonZeroUsize {
        self.occt_threads.unwrap_or_else(|| {
            let cores = std::thread::available_parallelism().map_or(1, NonZeroUsize::get);
            NonZeroUsize::new(cores / 2).unwrap_or(NonZeroUsize::MIN)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validate_parses_and_matches_build() {
        assert_eq!(Validate::parse(" Always "), Some(Validate::Always));
        assert_eq!(Validate::parse("OFF"), Some(Validate::Off));
        assert_eq!(Validate::parse("debug"), Some(Validate::DebugOnly));
        assert_eq!(Validate::parse("sometimes"), None);
        assert!(Validate::Always.is_enabled());
        assert!(!Validate::Off.is_enabled());
        assert_eq!(Validate::DebugOnly.is_enabled(), cfg!(debug_assertions));
        assert_eq!(Validate::default(), Validate::DebugOnly);
    }

    #[test]
    fn env_values_override_or_are_ignored() {
        let mut cfg = KernelConfig::default();
        cfg.apply_env(Some("always"), Some("3"));
        assert_eq!(cfg.validate, Validate::Always);
        assert_eq!(cfg.occt_threads, NonZeroUsize::new(3));
        assert_eq!(cfg.effective_occt_threads().get(), 3);

        let mut cfg = KernelConfig::default();
        cfg.apply_env(Some("bogus"), Some("0"));
        assert_eq!(cfg, KernelConfig::default());
        cfg.apply_env(None, Some("-2"));
        assert_eq!(cfg, KernelConfig::default());
    }

    #[test]
    fn default_threads_are_half_the_cores() {
        let cores = std::thread::available_parallelism().map_or(1, NonZeroUsize::get);
        let threads = KernelConfig::default().effective_occt_threads().get();
        assert_eq!(threads, (cores / 2).max(1));
    }
}
