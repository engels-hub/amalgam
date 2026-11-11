//! Embeds the git commit SHA as `AMG_GIT_SHA` ("unknown" outside a git checkout).

use std::process::Command;

fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").args(args).output().ok()?;
    if !out.status.success() {
        return None;
    }
    let s = String::from_utf8(out.stdout).ok()?.trim().to_owned();
    (!s.is_empty()).then_some(s)
}

fn main() {
    let sha = git(&["rev-parse", "--short=12", "HEAD"]).unwrap_or_else(|| "unknown".to_owned());
    println!("cargo::rustc-env=AMG_GIT_SHA={sha}");

    // Re-run when HEAD moves: HEAD itself (branch switch) and the ref it points to (commit).
    if let Some(head) = git(&["rev-parse", "--path-format=absolute", "--git-path", "HEAD"]) {
        println!("cargo::rerun-if-changed={head}");
    }
    if let Some(r) = git(&["symbolic-ref", "-q", "HEAD"])
        && let Some(path) = git(&["rev-parse", "--path-format=absolute", "--git-path", &r])
    {
        println!("cargo::rerun-if-changed={path}");
    }
}
