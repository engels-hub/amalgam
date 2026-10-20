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
