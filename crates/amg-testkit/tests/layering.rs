//! Layering check: every `amg-*` -> `amg-*` dependency edge in the workspace must be allowed by
//! docs/01-architecture.md §Crates. Changing `allowed()` needs an ADR, since crate boundaries
//! are architecture.

#![allow(
    clippy::expect_used,
    reason = "test helpers; a failure is a test failure"
)]

use std::collections::BTreeSet;
use std::process::Command;

const FOUNDATION: &[&str] = &["amg-core", "amg-jobs", "amg-compute"];
const DOMAINS: &[&str] = &[
    "amg-nurbs",
    "amg-brep",
    "amg-sketch",
    "amg-mesh",
    "amg-implicit",
    "amg-subd",
    "amg-materials",
    "amg-io",
    "amg-fem",
    "amg-topopt",
    "amg-am",
];

/// The allow-list, copied from architecture §Crates. "Besides core/jobs" there means every
/// crate above the foundation layer may also use the whole foundation layer.
fn allowed(krate: &str) -> Option<BTreeSet<&'static str>> {
    let extra: Vec<&'static str> = match krate {
        "amg-core" | "amg-occt-sys" => return Some(BTreeSet::new()),
        // S00 §Interfaces: `TriSoup` borrows `amg_core::Point3`. Never a domain crate, since
        // domain crates dev-depend on the testkit.
        "amg-testkit" => return Some(["amg-core"].into()),
        "amg-jobs" => return Some(["amg-core"].into()),
        "amg-compute" => return Some(["amg-core", "amg-jobs"].into()),
        // render is fed through `RenderScene`, never amg-doc.
        "amg-render" => return Some(["amg-core", "amg-jobs"].into()),
        "amg-nurbs" | "amg-sketch" | "amg-materials" => vec![],
        // amg-manifold-sys is created by MESH-05 (ADR-003).
        "amg-mesh" => vec!["amg-manifold-sys"],
        "amg-brep" => vec!["amg-occt-sys", "amg-nurbs"],
        "amg-implicit" => vec!["amg-mesh"],
        "amg-subd" => vec!["amg-brep", "amg-mesh"],
        "amg-io" => vec!["amg-brep", "amg-mesh", "amg-sketch", "amg-materials"],
        "amg-fem" => vec!["amg-mesh", "amg-materials"],
        "amg-topopt" => vec!["amg-fem", "amg-implicit", "amg-mesh"],
        "amg-am" => vec!["amg-mesh", "amg-implicit", "amg-io"],
        "amg-doc" => DOMAINS.to_vec(),
        "amg-features" => [DOMAINS, &["amg-doc"]].concat(),
        "amg-ui" => [DOMAINS, &["amg-doc", "amg-features", "amg-render"]].concat(),
        "amg-script" => vec!["amg-doc", "amg-features"],
        "amg-app" | "amg-cli" => [
            DOMAINS,
            &[
                "amg-doc",
                "amg-features",
                "amg-render",
                "amg-ui",
                "amg-script",
            ],
        ]
        .concat(),
        _ => return None,
    };
    Some(FOUNDATION.iter().copied().chain(extra).collect())
}

/// One dependency edge between two workspace crates.
#[derive(Debug)]
struct Edge {
    from: String,
    to: String,
    dev: bool,
}

/// Returns a description of every edge the allow-list rejects.
fn violations(edges: &[Edge]) -> Vec<String> {
    let mut out = Vec::new();
    for e in edges {
        // Every crate may use amg-testkit for its tests.
        if e.dev && e.to == "amg-testkit" {
            continue;
        }
        match allowed(&e.from) {
            None => out.push(format!("{} is not in the allow-list", e.from)),
            Some(set) if !set.contains(e.to.as_str()) => {
                let kind = if e.dev { " (dev)" } else { "" };
                out.push(format!("{} -> {}{kind} is not allowed", e.from, e.to));
            }
            Some(_) => {}
        }
    }
    out
}

/// Workspace package names and their `amg-*` edges, from `cargo metadata --no-deps`.
fn workspace() -> (BTreeSet<String>, Vec<Edge>) {
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_owned());
    let out = Command::new(cargo)
        .args([
            "metadata",
            "--format-version",
            "1",
            "--no-deps",
            "--offline",
        ])
        .current_dir(env!("CARGO_MANIFEST_DIR"))
        .output()
        .expect("run cargo metadata");
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let meta: serde_json::Value = serde_json::from_slice(&out.stdout).expect("parse metadata");

    let mut names = BTreeSet::new();
    let mut edges = Vec::new();
    for pkg in meta["packages"].as_array().expect("packages") {
        let from = pkg["name"].as_str().expect("name").to_owned();
        for dep in pkg["dependencies"].as_array().expect("dependencies") {
            let to = dep["name"].as_str().expect("dep name");
            if to.starts_with("amg-") {
                let dev = dep["kind"].as_str() == Some("dev");
                edges.push(Edge {
                    from: from.clone(),
                    to: to.to_owned(),
                    dev,
                });
            }
        }
        names.insert(from);
    }
    (names, edges)
}

#[test]
fn workspace_has_the_23_architecture_crates() {
    let (names, _) = workspace();
    let amg: Vec<_> = names.iter().filter(|n| n.starts_with("amg-")).collect();
    assert_eq!(amg.len(), 23, "amg-* packages: {amg:?}");
    for n in amg {
        assert!(allowed(n).is_some(), "{n} has no allow-list entry");
    }
}

#[test]
fn every_edge_is_allowed_by_the_architecture() {
    let (_, edges) = workspace();
    assert!(!edges.is_empty());
    let bad = violations(&edges);
    assert!(bad.is_empty(), "layering violations:\n{}", bad.join("\n"));
}

#[test]
fn forbidden_edges_are_rejected() {
    let edge = |from: &str, to: &str, dev| Edge {
        from: from.into(),
        to: to.into(),
        dev,
    };
    let bad = violations(&[
        edge("amg-render", "amg-doc", false),
        edge("amg-mesh", "amg-brep", false),
        edge("amg-core", "amg-jobs", false),
        edge("amg-doc", "amg-ui", true),
        edge("amg-unknown", "amg-core", false),
    ]);
    assert_eq!(bad.len(), 5, "{bad:#?}");
    assert!(violations(&[edge("amg-mesh", "amg-testkit", true)]).is_empty());
}
