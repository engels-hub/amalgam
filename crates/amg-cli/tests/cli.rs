//! `amg-cli` end-to-end tests.

use std::{
    process::Command,
    time::{SystemTime, UNIX_EPOCH},
};

#[test]
fn info_prints_and_writes_to_the_log() {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!("amg-cli-it-{}-{nanos}", std::process::id()));
    let out = Command::new(env!("CARGO_BIN_EXE_amg-cli"))
        .arg("info")
        .env("AMG_LOG_DIR", &dir)
        .env_remove("AMG_LOG")
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(out.status.success(), "{stdout}\n{stderr}");
    assert!(stdout.contains("amg-cli "), "{stdout}");
    assert!(stdout.contains(&dir.display().to_string()), "{stdout}");
    // Without AMG_LOG nothing is logged to stderr.
    assert!(stderr.is_empty(), "{stderr}");

    let log: String = std::fs::read_dir(&dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| {
            let n = p.file_name().unwrap().to_string_lossy();
            n.starts_with("amalgam-cli") && n.ends_with(".log")
        })
        .map(|p| std::fs::read_to_string(p).unwrap())
        .collect();
    assert!(log.contains("amg-cli info"), "{log}");
    assert!(log.contains(env!("CARGO_PKG_VERSION")), "{log}");
    let _ = std::fs::remove_dir_all(&dir);
}

/// S20 KH-02: `kernel harness` runs a seed list in worker processes (`kernel harness-worker`)
/// and writes the report JSON. Without OCCT every case is a setup error, but the run completes.
#[test]
fn kernel_harness_runs_a_seed_list_and_writes_a_report() {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!("amg-cli-harness-{}-{nanos}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let seeds = dir.join("seeds.toml");
    std::fs::write(
        &seeds,
        "schema = 1\n[[generator]]\nname = \"prism_vertical_edges\"\ncount = 2\n\
         [[generator]]\nname = \"radius_too_large\"\ncount = 1\n",
    )
    .unwrap();
    let report = dir.join("report.json");
    let baseline = dir.join("baseline.toml");
    let out = Command::new(env!("CARGO_BIN_EXE_amg-cli"))
        .args(["kernel", "harness", "--jobs", "1", "--seeds"])
        .arg(&seeds)
        .arg("--report")
        .arg(&report)
        .arg("--write-baseline")
        .arg(&baseline)
        .env("AMG_LOG_DIR", &dir)
        .env_remove("AMG_LOG")
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(out.status.success(), "{stdout}\n{stderr}");
    let want = if cfg!(feature = "occt") {
        "| all, raw | 3 | 2 / 2 (100.0 %) | 1 / 1 |"
    } else {
        "| all, raw | 3 | 0 / 2 (0.0 %) | 0 / 1 |"
    };
    assert!(stdout.contains(want), "{stdout}");
    let json = std::fs::read_to_string(&report).unwrap();
    assert!(json.contains("\"schema\": 1"), "{json}");
    assert!(
        json.contains("fillet/synthetic/prism_vertical_edges-0001"),
        "{json}"
    );
    // The run's own baseline: comparing against it finds no regression.
    let again = Command::new(env!("CARGO_BIN_EXE_amg-cli"))
        .args(["kernel", "harness", "--jobs", "1", "--seeds"])
        .arg(&seeds)
        .arg("--baseline")
        .arg(&baseline)
        .env("AMG_LOG_DIR", &dir)
        .output()
        .unwrap();
    assert!(
        again.status.success(),
        "{}",
        String::from_utf8_lossy(&again.stderr)
    );
    let _ = std::fs::remove_dir_all(&dir);
}
