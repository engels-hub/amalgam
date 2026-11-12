//! Compiles the C++ shim and links the vcpkg-built OCCT static toolkits (S02 §1, BREP-01).
//!
//! Only active with the `occt` feature. Never runs vcpkg or CMake: the install tree comes from
//! `vcpkg install` in manifest mode (repo `vcpkg.json`, triplet `x64-windows-amg`).
//! `AMG_OCCT_DIR` names it; `.cargo/config.toml [env]` defaults it to
//! `<workspace>/vcpkg_installed/x64-windows-amg`.

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=AMG_OCCT_DIR");
    #[cfg(feature = "occt")]
    occt::build();
}

#[cfg(feature = "occt")]
mod occt {
    use std::env;
    use std::path::{Path, PathBuf};

    /// Static toolkits, dependents first (occt-integration.md §2). A name missing from the
    /// install fails in `link_toolkits`.
    const TOOLKITS: &[&str] = &[
        "TKXCAF",
        "TKDESTEP",
        "TKDEIGES",
        "TKDE",
        "TKXSBase",
        "TKBin",
        "TKBinL",
        "TKCAF",
        "TKLCAF",
        "TKCDF",
        "TKFeat",
        "TKOffset",
        "TKFillet",
        "TKBool",
        "TKBO",
        "TKHelix",
        "TKMesh",
        "TKShHealing",
        "TKPrim",
        "TKTopAlgo",
        "TKGeomAlgo",
        "TKBRep",
        "TKGeomBase",
        "TKG3d",
        "TKG2d",
        "TKMath",
        "TKernel",
    ];

    /// Windows system libraries the linked OCCT objects reference.
    const SYSTEM_LIBS: &[&str] = &["advapi32", "user32", "ws2_32"];

    pub(super) fn build() {
        let msvc = env::var("CARGO_CFG_TARGET_ENV").is_ok_and(|e| e == "msvc");
        let root = occt_dir();
        let include = root.join("include/opencascade");
        let version_header = include.join("Standard_Version.hxx");
        if !version_header.is_file() {
            fail(&format!(
                "OCCT not found: {} is missing.\n\
                 The `occt` feature of amg-occt-sys needs the vcpkg-built OCCT. From the \
                 repository root run\n  \
                 vcpkg install --triplet x64-windows-amg\n\
                 or point AMG_OCCT_DIR at an install prefix (the directory that holds \
                 include/opencascade and lib). See docs/notes/occt-integration.md §2.",
                version_header.display()
            ));
        }
        println!("cargo:rerun-if-changed={}", version_header.display());
        println!("cargo:rerun-if-changed=src/bridge.rs");
        println!("cargo:rerun-if-changed=cpp/occt_shim.hxx");
        println!("cargo:rerun-if-changed=cpp/occt_shim.cpp");
        println!("cargo:rerun-if-changed=cpp/occt_tess.cpp");
        println!("cargo:rerun-if-changed=cpp/occt_build.cpp");
        println!("cargo:rerun-if-changed=cpp/occt_extract.cpp");

        let fixtures = cfg!(feature = "fixture-gen");
        let mut bridges = vec!["src/bridge.rs"];
        if fixtures {
            println!("cargo:rerun-if-changed=src/fixtures.rs");
            println!("cargo:rerun-if-changed=cpp/occt_fixtures.hxx");
            println!("cargo:rerun-if-changed=cpp/occt_fixtures.cpp");
            bridges.push("src/fixtures.rs");
        }
        let mut shim = cxx_build::bridges(bridges);
        shim.file("cpp/occt_shim.cpp")
            .file("cpp/occt_tess.cpp")
            .file("cpp/occt_build.cpp")
            .file("cpp/occt_extract.cpp")
            .include(&include)
            .std("c++17");
        if fixtures {
            shim.file("cpp/occt_fixtures.cpp");
        }
        if msvc {
            for flag in ["/EHsc", "/MD", "/utf-8", "/bigobj", "/permissive-"] {
                shim.flag(flag);
            }
        }
        shim.compile("amg_occt_shim");

        link_toolkits(&root.join("lib"), msvc);
    }

    fn occt_dir() -> PathBuf {
        match env::var_os("AMG_OCCT_DIR") {
            Some(dir) => PathBuf::from(dir),
            None => fail(
                "AMG_OCCT_DIR is not set. `.cargo/config.toml` sets it to the vcpkg manifest \
                 install tree; set it by hand if you build from outside the workspace.",
            ),
        }
    }

    fn link_toolkits(lib: &Path, msvc: bool) {
        println!("cargo:rustc-link-search=native={}", lib.display());
        for toolkit in TOOLKITS {
            let file = if msvc {
                format!("{toolkit}.lib")
            } else {
                format!("lib{toolkit}.a")
            };
            if lib.join(&file).is_file() {
                // `-bundle`: OCCT's libraries are large, so rustc must not copy them into the
                // amg-occt-sys rlib. They are only read at the final link.
                println!("cargo:rustc-link-lib=static:-bundle={toolkit}");
            } else if msvc {
                fail(&format!(
                    "OCCT toolkit {file} not found in {}",
                    lib.display()
                ));
            } else {
                // Linux best effort: a system OCCT may ship shared libraries only.
                println!("cargo:rustc-link-lib=dylib={toolkit}");
            }
        }
        if msvc {
            for name in SYSTEM_LIBS {
                println!("cargo:rustc-link-lib=dylib={name}");
            }
        }
    }

    fn fail(msg: &str) -> ! {
        eprintln!("error: {msg}");
        std::process::exit(1);
    }
}
