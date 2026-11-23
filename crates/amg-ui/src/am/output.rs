//! Send to Slicer and Export Plate 3MF: AM-06's hand-off run as `amg-jobs` jobs.
//!
//! The UI thread only collects the plate (names, `Arc` meshes, placements); the effective
//! transforms, the 3MF/STL write, pruning the hand-off folder and starting the slicer all run in
//! the job. Failures (no slicer, write errors) come back as toasts through
//! [`AppCx::spawn_tracked`]; the result is kept in the layer ([`AmLayer::last_output`]).

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::mpsc::Receiver;

use amg_am::AmError;
use amg_am::handoff::{
    HandoffFormat, HandoffItem, HandoffPlate, SlicerConfig, SlicerInstall, detect_slicers,
    export_plate, send_to_slicer as am_send,
};
use amg_am::machine::{Machine, SlicerKind};
use amg_am::place::{Placement, effective_transform};
use amg_features::am::{ItemGeom, ItemId, UnresolvedReason, resolve_plate};
use amg_jobs::{JobCtx, SpawnOptions};
use amg_mesh::{IndexedMesh, Rgba8};

use super::layer::{AmLayer, unresolved_message};
use super::settings::{self, slicer_name};
use super::{FnJob, current_setup};
use crate::app::AppCx;
use crate::file::{FileDialogKind, FileDialogRequest, FileFilter};
use crate::notify::Notification;

/// What the last Send to Slicer or Export did.
#[derive(Clone, Debug, PartialEq)]
pub struct LastOutput {
    /// The file written.
    pub path: PathBuf,
    /// Objects in it.
    pub objects: usize,
    /// Triangles in it.
    pub triangles: usize,
    /// The started slicer's pid (Send to Slicer only).
    pub pid: Option<u32>,
    /// The slicer started (Send to Slicer only).
    pub slicer: Option<SlicerKind>,
}

struct ItemInput {
    name: String,
    mesh: Arc<IndexedMesh>,
    placement: Placement,
    filament: Option<u8>,
    color: Option<Rgba8>,
}

/// A plate captured on the UI thread for a job.
pub(crate) struct PlateInput {
    design_name: String,
    plate_name: String,
    machine: Machine,
    items: Vec<ItemInput>,
    skipped: Vec<(ItemId, UnresolvedReason)>,
}

impl PlateInput {
    /// Effective transforms (one vertex pass per item); job side.
    fn items(&self, ctx: &JobCtx) -> Result<Vec<HandoffItem>, AmError> {
        let mut out = Vec::with_capacity(self.items.len());
        for it in &self.items {
            ctx.checkpoint()?;
            out.push(HandoffItem {
                name: it.name.clone(),
                transform: effective_transform(&it.mesh, &it.placement),
                mesh: Arc::clone(&it.mesh),
                filament: it.filament,
                color: it.color,
            });
        }
        Ok(out)
    }
}

/// A Save dialog answer waited for by the layer.
pub(crate) struct PendingExport {
    rx: Receiver<Option<PathBuf>>,
    plate: PlateInput,
}

/// The active plate of the active setup, or a toast saying why there is nothing to output.
fn collect(cx: &mut AppCx<'_>, what: &str) -> Option<PlateInput> {
    let Some((_, s)) = current_setup(cx) else {
        cx.notify(Notification::info(
            what,
            "Create a print setup first (New Setup).",
        ));
        return None;
    };
    let env = cx.env();
    let plate_name = s
        .plates
        .iter()
        .find(|p| p.id == s.active_plate)
        .map(|p| p.name.clone())
        .unwrap_or_default();
    let mut items = Vec::new();
    let mut skipped = Vec::new();
    for r in resolve_plate(&s, s.active_plate, env.design, env.evaluated) {
        match r.geom {
            ItemGeom::Mesh(mesh) => items.push(ItemInput {
                name: r.name,
                mesh,
                placement: r.item.placement,
                filament: r.item.filament,
                color: r.color,
            }),
            ItemGeom::Unresolved(why) => skipped.push((r.item.id, why)),
        }
    }
    let design_name = env.design.meta().name.clone();
    let input = PlateInput {
        design_name,
        plate_name,
        machine: s.machine.clone(),
        items,
        skipped,
    };
    if !input.skipped.is_empty() {
        let why: Vec<String> = input
            .skipped
            .iter()
            .map(|(id, r)| format!("item {}: {}", id.0, unresolved_message(r)))
            .collect();
        cx.notify(Notification::warning(
            format!("{what}: {} item(s) left out", input.skipped.len()),
            why.join("; "),
        ));
    }
    if input.items.is_empty() {
        cx.notify(Notification::info(
            what,
            "The plate has no printable items.",
        ));
        return None;
    }
    Some(input)
}

/// The detected install to use for a machine: its preferred slicer, else Chitubox for resin
/// and OrcaSlicer otherwise, else the first one found.
pub fn pick_install(found: &[SlicerInstall], machine: &Machine) -> Option<SlicerInstall> {
    let fallback = if machine.technology.is_resin() {
        SlicerKind::Chitubox
    } else {
        SlicerKind::OrcaSlicer
    };
    let want = [machine.slicer, Some(fallback)];
    want.iter()
        .flatten()
        .find_map(|k| found.iter().find(|i| i.kind == *k))
        .or_else(|| found.first())
        .cloned()
}

/// Send to Slicer on the active plate (see the module docs).
pub(crate) fn send_to_slicer(cx: &mut AppCx<'_>) {
    let Some(input) = collect(cx, "Send to Slicer") else {
        return;
    };
    let s = settings::load();
    let configured = s.slicer.clone();
    let dir = s.handoff_dir();
    let job = FnJob::new("Send to slicer", move |ctx: &JobCtx| {
        let cfg = match configured {
            Some(c) => c,
            None => {
                let found = detect_slicers();
                let i = pick_install(&found, &input.machine).ok_or_else(|| {
                    AmError::SlicerNotFound(PathBuf::from("(no slicer detected)"))
                })?;
                SlicerConfig::new(i.kind, i.exe)
            }
        };
        let items = input.items(&ctx.sub(0.0, 0.1))?;
        let plate = HandoffPlate {
            design_name: input.design_name.clone(),
            plate_name: input.plate_name.clone(),
            machine: &input.machine,
            items,
            thumbnail_png: None,
        };
        let r = am_send(&plate, &cfg, &dir, &ctx.sub(0.1, 1.0))?;
        Ok((r, cfg.kind))
    });
    cx.spawn_tracked(job, SpawnOptions::default(), |res, cx| {
        let Ok((r, kind)) = res else { return };
        cx.notify(Notification::info(
            format!("Sent to {}", slicer_name(kind)),
            format!(
                "{} object(s), {} triangles: {}",
                r.handoff.objects,
                r.handoff.triangles,
                r.handoff.path.display()
            ),
        ));
        if let Some(l) = cx.doc_layer::<AmLayer>() {
            l.last_output = Some(LastOutput {
                path: r.handoff.path.clone(),
                objects: r.handoff.objects,
                triangles: r.handoff.triangles,
                pid: Some(r.pid),
                slicer: Some(kind),
            });
        }
    });
}

/// Export Plate 3MF: asks for a path, then exports in a job (polled by the layer).
pub(crate) fn export_3mf(cx: &mut AppCx<'_>) {
    let Some(input) = collect(cx, "Export Plate 3MF") else {
        return;
    };
    let name = format!("{}-{}.3mf", input.design_name, input.plate_name)
        .replace(['\\', '/', ':', '*', '?', '"', '<', '>', '|'], "_");
    let rx = cx.pick_file(FileDialogRequest {
        kind: FileDialogKind::Save,
        title: "Export Plate 3MF".to_owned(),
        filters: vec![FileFilter {
            name: "3MF".to_owned(),
            extensions: vec!["3mf".to_owned()],
        }],
        directory: None,
        file_name: Some(name),
    });
    match cx.doc_layer::<AmLayer>() {
        Some(l) => l.export = Some(PendingExport { rx, plate: input }),
        None => tracing::warn!("am: no plate layer for the export"),
    }
}

/// Polls the Save dialog of a pending export; spawns the export job once it has a path.
pub(crate) fn poll_export(layer: &mut AmLayer, cx: &mut AppCx<'_>) {
    let Some(p) = &layer.export else { return };
    let answer = match p.rx.try_recv() {
        Ok(a) => a,
        Err(std::sync::mpsc::TryRecvError::Empty) => return,
        Err(std::sync::mpsc::TryRecvError::Disconnected) => None,
    };
    let Some(PendingExport { plate, .. }) = layer.export.take() else {
        return;
    };
    let Some(mut path) = answer else { return };
    if !path
        .extension()
        .is_some_and(|e| e.eq_ignore_ascii_case("3mf"))
    {
        path.set_extension("3mf");
    }
    let flavor = plate.machine.slicer.unwrap_or(SlicerKind::OrcaSlicer);
    let job = FnJob::new("Export plate 3MF", move |ctx: &JobCtx| {
        let items = plate.items(&ctx.sub(0.0, 0.1))?;
        let hp = HandoffPlate {
            design_name: plate.design_name.clone(),
            plate_name: plate.plate_name.clone(),
            machine: &plate.machine,
            items,
            thumbnail_png: None,
        };
        export_plate(
            &hp,
            HandoffFormat::ThreeMf,
            flavor,
            &path,
            &ctx.sub(0.1, 1.0),
        )
    });
    cx.spawn_tracked(job, SpawnOptions::default(), |res, cx| {
        let Ok(r) = res else { return };
        cx.notify(Notification::info(
            "Plate exported",
            format!("{} object(s): {}", r.objects, r.path.display()),
        ));
        if let Some(l) = cx.doc_layer::<AmLayer>() {
            l.last_output = Some(LastOutput {
                path: r.path.clone(),
                objects: r.objects,
                triangles: r.triangles,
                pid: None,
                slicer: None,
            });
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_am::handoff::DetectSource;
    use amg_am::machine::MachineLibrary;

    fn install(kind: SlicerKind) -> SlicerInstall {
        SlicerInstall {
            kind,
            exe: PathBuf::from(format!("C:/{kind:?}.exe")),
            version: None,
            source: DetectSource::Path,
        }
    }

    #[test]
    fn the_machine_picks_its_slicer() {
        let lib = MachineLibrary::builtin();
        let fff = lib
            .iter()
            .find(|m| !m.technology.is_resin())
            .cloned()
            .unwrap_or_else(|| unreachable!("built-in FFF machine"));
        let resin = lib
            .iter()
            .find(|m| m.technology.is_resin())
            .cloned()
            .unwrap_or_else(|| unreachable!("built-in resin machine"));
        let all = [
            install(SlicerKind::PrusaSlicer),
            install(SlicerKind::Chitubox),
            install(SlicerKind::OrcaSlicer),
        ];
        assert_eq!(
            pick_install(&all, &fff).map(|i| i.kind),
            Some(SlicerKind::OrcaSlicer)
        );
        assert_eq!(
            pick_install(&all, &resin).map(|i| i.kind),
            Some(SlicerKind::Chitubox)
        );
        assert_eq!(
            pick_install(&all[..1], &fff).map(|i| i.kind),
            Some(SlicerKind::PrusaSlicer)
        );
        assert_eq!(pick_install(&[], &fff), None);
    }
}
