//! Slicer hand-off (S16 "Hand-off", AM-06): find the installed slicers, write the plate as a
//! 3MF or STL in slicer coordinates, and start the slicer on it.
//!
//! - [`detect`]: [`detect_slicers`], [`parse_registry_exe`] (registry reads only).
//! - [`export`]: [`export_plate`], [`handoff_path`], [`prune_handoff_dir`].
//! - [`launch`]: [`launch()`] (detached) and [`SendToSlicerJob`], the `amg-jobs` job that does
//!   export, prune and launch, so the UI thread never waits for any of it.

mod detect;
mod export;
mod launch;

pub use detect::{
    DetectOptions, DetectSource, SlicerInstall, classify_exe, classify_name, detect_slicers,
    detect_slicers_with, parse_registry_exe, parse_registry_exe_with,
};
pub use export::{
    HandoffFormat, HandoffItem, HandoffPlate, HandoffReport, default_handoff_dir, export_plate,
    handoff_path, prune_handoff_dir,
};
pub use launch::{SendReport, SendToSlicerJob, SlicerConfig, launch, send_to_slicer};
