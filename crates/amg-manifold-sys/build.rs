//! Links the vcpkg-built Manifold (`manifoldc` + `manifold`), Clipper2 and oneTBB static
//! libraries.
//!
//! Only active with the `native` feature. No git, CMake or downloads happen here (S07 decision
//! 5): the libraries come from `vcpkg install` in manifest mode (repo `vcpkg.json`, overlay port
//! `ports/manifold`, triplet `x64-windows-amg`).
//!
//! Lookup of the vcpkg install tree (the directory that holds `<triplet>/lib`):
//! 1. `AMG_VCPKG_INSTALLED` (for `vcpkg install --x-install-root=<dir>`);
//! 2. `<workspace>/vcpkg_installed` (the manifest-mode default, used by CI).
//!
//! The triplet is `AMG_VCPKG_TRIPLET`, else `VCPKG_DEFAULT_TRIPLET`, else `x64-windows-amg` on
//! Windows and `x64-linux` elsewhere (Linux is best-effort).
//!
//! The pinned `manifoldc.h` is scanned for every function `src/lib.rs` declares, so a Manifold
//! update that renames or drops one fails here instead of at link time.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-env-changed=AMG_VCPKG_INSTALLED");
    println!("cargo:rerun-if-env-changed=AMG_VCPKG_TRIPLET");
    println!("cargo:rerun-if-env-changed=VCPKG_DEFAULT_TRIPLET");
    if env::var_os("CARGO_FEATURE_NATIVE").is_none() {
        return;
    }
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap_or_default());
    let installed = env::var_os("AMG_VCPKG_INSTALLED")
        .map(PathBuf::from)
        .unwrap_or_else(|| manifest.join("../../vcpkg_installed"));
    let windows = env::var("CARGO_CFG_TARGET_OS").is_ok_and(|os| os == "windows");
    let triplet = env::var("AMG_VCPKG_TRIPLET")
        .or_else(|_| env::var("VCPKG_DEFAULT_TRIPLET"))
        .unwrap_or_else(|_| {
            if windows {
                "x64-windows-amg".to_owned()
            } else {
                "x64-linux".to_owned()
            }
        });
    let root = installed.join(&triplet);
    let lib = root.join("lib");
    let header = root.join("include/manifold/manifoldc.h");
    if !header.is_file() {
        fail(&format!(
            "Manifold not found: {} is missing.\n\
             The `native` feature of amg-manifold-sys (enabled by `amg-mesh/manifold`) needs the \
             vcpkg-built Manifold. From the repository root run\n  \
             vcpkg install --triplet {triplet}\n\
             (or `--x-install-root=<dir>` and set AMG_VCPKG_INSTALLED=<dir>), see docs/dev-setup.md.",
            header.display()
        ));
    }
    println!("cargo:rerun-if-changed={}", header.display());
    check_header(&header, &manifest.join("src/lib.rs"));

    println!("cargo:rustc-link-search=native={}", lib.display());
    println!("cargo:rustc-link-lib=static=manifoldc");
    println!("cargo:rustc-link-lib=static=manifold");
    // Clipper2: the C binding needs CrossSection (MANIFOLD_CBIND requires it in 3.5.4).
    println!("cargo:rustc-link-lib=static=Clipper2");
    // oneTBB: `tbb12` on Windows (the vcpkg port removes the duplicate `tbb.lib`), `tbb` elsewhere.
    let tbb = if windows { "tbb12" } else { "tbb" };
    if !lib.join(lib_file(tbb, windows)).is_file() {
        fail(&format!(
            "oneTBB library {tbb} not found in {}",
            lib.display()
        ));
    }
    println!("cargo:rustc-link-lib=static={tbb}");
    if !windows {
        println!("cargo:rustc-link-lib=dylib=stdc++");
        println!("cargo:rustc-link-lib=dylib=pthread");
    }
    println!("cargo:include={}", root.join("include").display());
}

fn lib_file(name: &str, windows: bool) -> String {
    if windows {
        format!("{name}.lib")
    } else {
        format!("lib{name}.a")
    }
}

/// Every `pub fn manifold_*` declared in `lib.rs` must appear in the pinned header.
fn check_header(header: &Path, lib_rs: &Path) {
    let h = fs::read_to_string(header).unwrap_or_default();
    let src = fs::read_to_string(lib_rs).unwrap_or_default();
    let missing: Vec<&str> = src
        .lines()
        .filter_map(|l| l.trim().strip_prefix("pub fn "))
        .filter_map(|l| l.split('(').next())
        .filter(|name| name.starts_with("manifold_"))
        .filter(|name| !h.contains(&format!("{name}(")))
        .collect();
    if !missing.is_empty() {
        fail(&format!(
            "{} does not declare {} (Manifold version drift? Update src/lib.rs.)",
            header.display(),
            missing.join(", ")
        ));
    }
}

fn fail(msg: &str) -> ! {
    eprintln!("error: {msg}");
    std::process::exit(1);
}
