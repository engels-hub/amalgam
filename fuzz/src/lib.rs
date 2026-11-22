//! IO-16 fuzz harness. The targets in `fuzz_targets/` are one-line wrappers around
//! [`harness`], which `crates/amg-io/tests/fuzz_replay.rs` also compiles (by path) to replay
//! the committed artifacts on stable.

pub mod harness;

/// Runs one target body on one input. Off Windows this is just `target(data)`: libfuzzer-sys's
/// panic hook aborts, libFuzzer catches `SIGABRT` and writes the crash file.
///
/// On Windows `std::process::abort` is `__fastfail`, which no exception handler sees, so a
/// panic would end the process without a crash file. There the panic is caught (after the
/// message is printed) and turned into an illegal-instruction exception, which libFuzzer's
/// handler reports as a crash and saves.
pub fn run(target: harness::Target, data: &[u8]) {
    #[cfg(windows)]
    {
        static HOOK: std::sync::Once = std::sync::Once::new();
        HOOK.call_once(|| {
            std::panic::set_hook(Box::new(|info| {
                eprintln!("{info}");
                // Only with RUST_BACKTRACE: symbolising takes seconds, past `-timeout=2`.
                eprintln!("{}", std::backtrace::Backtrace::capture());
            }));
        });
        if std::panic::catch_unwind(|| target(data)).is_err() {
            // SAFETY: `ud2` only raises EXCEPTION_ILLEGAL_INSTRUCTION; libFuzzer's handler
            // writes the input and exits the process.
            #[allow(unsafe_code)]
            unsafe {
                core::arch::asm!("ud2", options(noreturn));
            }
        }
    }
    #[cfg(not(windows))]
    target(data);
}

// Windows MSVC without a sanitizer (`cargo fuzz build -s none`): LLVM puts the
// SanitizerCoverage counters and PC table in the COFF sections `.SCOV$CM` / `.SCOVP$M` and
// references start/stop symbols that, on MSVC, only the ASan runtime defines
// (compiler-rt `sanitizer_coverage_win_sections.cpp`). These are the same markers: the
// linker sorts `$A` < `$M` < `$Z` within a section group, so they bracket the instrumented
// data. Zero padding between them is skipped by libFuzzer (zero PCs, zero counters). The
// runtime's own copies are `selectany`, so an ASan build on Windows would still link.
#[cfg(all(windows, target_env = "msvc", fuzzing))]
// Assembly, not Rust statics: the coverage pass renames its own references when the module
// already has IR globals of these names, and then nothing resolves them.
core::arch::global_asm!(
    ".section .SCOV$CA,\"dw\"",
    ".globl __start___sancov_cntrs",
    "__start___sancov_cntrs:",
    ".quad 0",
    ".section .SCOV$CZ,\"dw\"",
    ".globl __stop___sancov_cntrs",
    "__stop___sancov_cntrs:",
    ".quad 0",
    ".section .SCOVP$A,\"dr\"",
    ".globl __start___sancov_pcs",
    "__start___sancov_pcs:",
    ".quad 0",
    ".section .SCOVP$Z,\"dr\"",
    ".globl __stop___sancov_pcs",
    "__stop___sancov_pcs:",
    ".quad 0",
    ".text",
);
