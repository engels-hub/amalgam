//! The Manufacture dialogs: New Setup (machine picker), Add Bodies and Move/Rotate.
//!
//! They are module dialogs ([`AppDialog`]) hand-built on the framework API; UI-05's generic
//! renderer replaces them in Phase 2. Each OK submits one `CommitBatch` (one undo entry).

use std::borrow::Cow;

use amg_am::machine::{
    BedShape, LoadWarning, Machine, MachineKey, MachineLibrary, ProcessParams, Technology,
};
use amg_am::place::{Placement, ZMode};
use amg_core::glam::{DQuat, DVec2};
use amg_doc::{BodyId, ExtKey};
use amg_features::am::{ItemId, PrintSetup, next_setup_key, setups};

use super::layer::AmLayer;
use super::{
    active_items, add_bodies, current_setup, initial_placement, mesh_of, selected_bodies, settings,
    submit_setup, targeted_items,
};
use crate::app::{AppCx, CommitBatch};
use crate::dialog::AppDialog;
use crate::widgets::{DialogAction, DialogFrame};

/// The machine the New Setup dialog selects first: the owner's Ender-3 V3 KE KCM.
pub const DEFAULT_MACHINE: &str = "builtin:creality/ender-3-v3-ke-kcm";

/// One-line description of a machine ("220 × 220 × 240 mm, FFF, 4 filament slots").
pub fn machine_summary(m: &Machine) -> String {
    let v = &m.volume;
    let bed = match &v.bed {
        BedShape::Rect { size } => format!("{} × {}", fmt_mm(size.x), fmt_mm(size.y)),
        BedShape::Circle { diameter } => format!("Ø {}", fmt_mm(*diameter)),
        BedShape::Polygon { .. } => {
            let (lo, hi) = v.bed_rect();
            format!("{} × {} (shaped)", fmt_mm(hi.x - lo.x), fmt_mm(hi.y - lo.y))
        }
    };
    let tech = match m.technology {
        Technology::Fff => "FFF",
        Technology::Msla => "MSLA",
        Technology::Sla => "SLA",
        Technology::Sls => "SLS",
        Technology::Mpbf => "metal PBF",
    };
    let extra = match &m.process {
        ProcessParams::Fff(f) if f.filament_slots > 1 => {
            format!(", {} filament slots", f.filament_slots)
        }
        ProcessParams::Resin(r) => format!(", {} × {} px", r.resolution[0], r.resolution[1]),
        _ => String::new(),
    };
    format!("{bed} × {} mm, {tech}{extra}", fmt_mm(v.height))
}

fn fmt_mm(x: f64) -> String {
    let s = format!("{x:.2}");
    s.trim_end_matches('0').trim_end_matches('.').to_owned()
}

/// The machine library: built-ins plus the user machine folder, with load warnings.
pub fn load_library() -> (MachineLibrary, Vec<LoadWarning>) {
    let mut lib = MachineLibrary::builtin();
    let warnings = match settings::load().machines_dir() {
        Some(dir) if dir.is_dir() => lib.load_user_dir(&dir),
        _ => Vec::new(),
    };
    (lib, warnings)
}

// ---- New Setup ---------------------------------------------------------------------------

/// New Setup: pick a machine from the library (built-in + user folder) and a name. Selected
/// mesh bodies are added to the new setup's plate.
pub struct NewSetupDialog {
    library: MachineLibrary,
    warnings: Vec<LoadWarning>,
    /// The selected machine.
    pub machine: MachineKey,
    /// Setup name.
    pub name: String,
    /// Bodies to add (the selection's mesh bodies when the dialog opened).
    pub bodies: Vec<BodyId>,
}

impl std::fmt::Debug for NewSetupDialog {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NewSetupDialog")
            .field("machine", &self.machine)
            .field("name", &self.name)
            .field("bodies", &self.bodies)
            .finish_non_exhaustive()
    }
}

impl NewSetupDialog {
    /// The dialog for the document behind `cx`.
    pub fn new(cx: &AppCx<'_>) -> NewSetupDialog {
        let (library, warnings) = load_library();
        let machine = library
            .get(&MachineKey(DEFAULT_MACHINE.to_owned()))
            .or_else(|| library.iter().next())
            .map(|m| m.key.clone())
            .unwrap_or_else(|| MachineKey(DEFAULT_MACHINE.to_owned()));
        let env = cx.env();
        let n = setups(env.design).len() + 1;
        let bodies = selected_bodies(cx)
            .into_iter()
            .filter(|b| mesh_of(env.evaluated, *b).is_some())
            .collect();
        NewSetupDialog {
            library,
            warnings,
            machine,
            name: format!("Setup {n}"),
            bodies,
        }
    }

    /// The machines offered, in library order.
    pub fn machines(&self) -> impl Iterator<Item = &Machine> + '_ {
        self.library.iter()
    }

    /// Problems with files of the user machine folder.
    pub fn warnings(&self) -> &[LoadWarning] {
        &self.warnings
    }

    /// Creates the setup (one undo entry) and makes it active. Returns its key.
    pub fn create(&self, cx: &mut AppCx<'_>) -> Option<ExtKey> {
        let m = self.library.get(&self.machine)?.clone();
        let name = if self.name.trim().is_empty() {
            "Setup"
        } else {
            self.name.trim()
        };
        let mut s = PrintSetup::new(name, m);
        let place = initial_placement(&s);
        let plate = s.active_plate;
        for b in &self.bodies {
            s.add_body(plate, *b, place);
        }
        let key = next_setup_key(cx.env().design);
        cx.submit(CommitBatch {
            label: "New Print Setup".to_owned(),
            commands: vec![s.add_command()],
        });
        if let Some(l) = cx.doc_layer::<AmLayer>() {
            l.set_active(Some(key.clone()));
        }
        Some(key)
    }

    fn form(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Name");
            ui.text_edit_singleline(&mut self.name);
        });
        ui.label(egui::RichText::new("Machine").strong());
        egui::ScrollArea::vertical()
            .max_height(220.0)
            .show(ui, |ui| {
                for m in self.library.iter() {
                    let source = match m.key.0.split(':').next() {
                        Some("builtin") => "built-in",
                        Some("user") => "user",
                        _ => "imported",
                    };
                    ui.radio_value(
                        &mut self.machine,
                        m.key.clone(),
                        format!("{} ({source})", m.name),
                    );
                }
            });
        if let Some(m) = self.library.get(&self.machine) {
            ui.weak(machine_summary(m));
        }
        for w in &self.warnings {
            ui.colored_label(
                ui.visuals().warn_fg_color,
                format!("{}: {}", w.file.display(), w.message),
            );
        }
        if !self.bodies.is_empty() {
            ui.weak(format!(
                "{} selected mesh bod{} will be added to the plate.",
                self.bodies.len(),
                if self.bodies.len() == 1 { "y" } else { "ies" }
            ));
        }
    }
}

impl AppDialog for NewSetupDialog {
    fn title(&self) -> Cow<'static, str> {
        Cow::Borrowed("New Print Setup")
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        let action = DialogFrame::new("New Print Setup").show(ctx, |ui| self.form(ui));
        match action {
            DialogAction::Ok => {
                self.create(cx);
                false
            }
            DialogAction::Cancel => false,
            DialogAction::None => true,
        }
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

// ---- Add Bodies --------------------------------------------------------------------------

/// Add Bodies without a selection: tick the mesh bodies to put on the active plate.
#[derive(Debug)]
pub struct AddBodiesDialog {
    key: ExtKey,
    /// Candidate bodies (mesh bodies not on the plate) and whether each is ticked.
    pub bodies: Vec<(BodyId, String, bool)>,
}

impl AddBodiesDialog {
    /// The dialog for setup `key`.
    pub fn new(cx: &AppCx<'_>, key: ExtKey, s: &PrintSetup) -> AddBodiesDialog {
        let env = cx.env();
        let on_plate: Vec<BodyId> = active_items(s).iter().map(|i| i.body).collect();
        let bodies = env
            .evaluated
            .bodies()
            .filter(|(id, _)| !on_plate.contains(id) && mesh_of(env.evaluated, *id).is_some())
            .map(|(id, _)| {
                let name = env
                    .design
                    .body_props(id)
                    .name
                    .clone()
                    .unwrap_or_else(|| format!("Body {}.{}", id.origin.0, id.slot));
                (id, name, false)
            })
            .collect();
        AddBodiesDialog { key, bodies }
    }
}

impl AppDialog for AddBodiesDialog {
    fn title(&self) -> Cow<'static, str> {
        Cow::Borrowed("Add Bodies")
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        let any = self.bodies.iter().any(|b| b.2);
        let action = DialogFrame::new("Add Bodies")
            .ok_enabled(if any {
                crate::command::Enablement::Enabled
            } else {
                crate::command::Enablement::disabled("Tick at least one body")
            })
            .show(ctx, |ui| {
                if self.bodies.is_empty() {
                    ui.weak("Every mesh body is already on the plate.");
                }
                for (_, name, on) in &mut self.bodies {
                    ui.checkbox(on, name.as_str());
                }
            });
        match action {
            DialogAction::Ok => {
                let picked: Vec<BodyId> = self.bodies.iter().filter(|b| b.2).map(|b| b.0).collect();
                if let Some((k, s)) = current_setup(cx)
                    && k == self.key
                {
                    add_bodies(cx, k, s, &picked);
                }
                false
            }
            DialogAction::Cancel => false,
            DialogAction::None => true,
        }
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

// ---- Move/Rotate -------------------------------------------------------------------------

/// The numeric fields of Move/Rotate. Position is absolute (the footprint centre on the
/// plate); rotations are applied to the item's current rotation in the order tilt Y, tilt X,
/// then rotate Z (all about plate axes), so rotate Z alone spins the item in place.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MoveRotateFields {
    /// Footprint centre x, mm.
    pub x: f64,
    /// Footprint centre y, mm.
    pub y: f64,
    /// Lowest point above the plate (on-plate items) or the fixed z offset, mm.
    pub z: f64,
    /// Spin about the build axis, degrees.
    pub rotate_z: f64,
    /// Tilt about plate X, degrees.
    pub tilt_x: f64,
    /// Tilt about plate Y, degrees.
    pub tilt_y: f64,
}

/// Move/Rotate: numeric placement of one item with a live preview in the plate scene.
#[derive(Debug)]
pub struct MoveRotateDialog {
    key: ExtKey,
    item: ItemId,
    base: Placement,
    /// The fields.
    pub fields: MoveRotateFields,
}

impl MoveRotateDialog {
    /// The dialog on the selected item, or the only item of the plate.
    ///
    /// # Errors
    /// A message when there is no setup or no single item to work on.
    pub fn new(cx: &mut AppCx<'_>) -> Result<MoveRotateDialog, &'static str> {
        let Some((key, s)) = current_setup(cx) else {
            return Err("Create a print setup first (New Setup).");
        };
        let ids = targeted_items(cx, &s, false);
        let item = match (ids.as_slice(), active_items(&s)) {
            ([one], _) => *one,
            ([], [only]) => only.id,
            ([], _) => return Err("Select the item to move (one item)."),
            _ => return Err("Select one item: Move/Rotate works on one item at a time."),
        };
        let base = active_items(&s)
            .iter()
            .find(|i| i.id == item)
            .map(|i| i.placement)
            .ok_or("The item is not on the active plate.")?;
        let z = match base.z {
            ZMode::OnPlate { elevation } => elevation,
            ZMode::Fixed { z } => z,
        };
        Ok(MoveRotateDialog {
            key,
            item,
            base,
            fields: MoveRotateFields {
                x: base.xy.x,
                y: base.xy.y,
                z,
                rotate_z: 0.0,
                tilt_x: 0.0,
                tilt_y: 0.0,
            },
        })
    }

    /// The item being placed.
    pub fn item(&self) -> ItemId {
        self.item
    }

    /// The placement the fields describe.
    pub fn placement(&self) -> Placement {
        let f = &self.fields;
        let r = DQuat::from_rotation_z(f.rotate_z.to_radians())
            * DQuat::from_rotation_x(f.tilt_x.to_radians())
            * DQuat::from_rotation_y(f.tilt_y.to_radians())
            * self.base.rotation;
        let z = match self.base.z {
            ZMode::OnPlate { .. } => ZMode::OnPlate {
                elevation: f.z.max(0.0),
            },
            ZMode::Fixed { .. } => ZMode::Fixed { z: f.z },
        };
        Placement {
            rotation: r,
            xy: DVec2::new(f.x, f.y),
            z,
        }
        .normalized()
    }

    fn set_preview(&self, cx: &mut AppCx<'_>, on: bool) {
        let p = self.placement();
        if let Some(l) = cx.doc_layer::<AmLayer>() {
            l.preview = on.then_some((self.item, p));
        }
    }

    /// Applies the fields (one undo entry).
    pub fn apply(&self, cx: &mut AppCx<'_>) {
        self.set_preview(cx, false);
        let Some((k, mut s)) = current_setup(cx) else {
            return;
        };
        if k != self.key {
            return;
        }
        let p = self.placement();
        match s.item_mut(self.item) {
            Some(it) if it.placement != p => it.placement = p,
            _ => return,
        }
        submit_setup(cx, "Move/Rotate", k, &s);
    }

    fn form(&mut self, ui: &mut egui::Ui) {
        let row = |ui: &mut egui::Ui, label: &str, v: &mut f64, suffix: &str, speed: f64| {
            ui.horizontal(|ui| {
                ui.label(label);
                ui.add(egui::DragValue::new(v).speed(speed).suffix(suffix));
            });
        };
        ui.label(egui::RichText::new("Position (footprint centre)").strong());
        row(ui, "X", &mut self.fields.x, " mm", 0.5);
        row(ui, "Y", &mut self.fields.y, " mm", 0.5);
        let zl = match self.base.z {
            ZMode::OnPlate { .. } => "Elevation",
            ZMode::Fixed { .. } => "Z",
        };
        row(ui, zl, &mut self.fields.z, " mm", 0.1);
        ui.label(egui::RichText::new("Rotation (from the current one)").strong());
        row(ui, "Rotate Z", &mut self.fields.rotate_z, "°", 1.0);
        row(ui, "Tilt X", &mut self.fields.tilt_x, "°", 1.0);
        row(ui, "Tilt Y", &mut self.fields.tilt_y, "°", 1.0);
    }
}

impl AppDialog for MoveRotateDialog {
    fn title(&self) -> Cow<'static, str> {
        Cow::Borrowed("Move/Rotate")
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        let action = DialogFrame::new("Move/Rotate").show(ctx, |ui| self.form(ui));
        match action {
            DialogAction::Ok => {
                self.apply(cx);
                false
            }
            DialogAction::Cancel => {
                self.set_preview(cx, false);
                false
            }
            DialogAction::None => {
                self.set_preview(cx, true);
                true
            }
        }
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn summaries_of_the_built_in_machines() {
        let lib = MachineLibrary::builtin();
        let kcm = lib
            .get(&MachineKey(DEFAULT_MACHINE.to_owned()))
            .map(machine_summary);
        assert_eq!(
            kcm.as_deref(),
            Some("220 × 220 × 240 mm, FFF, 4 filament slots")
        );
        for m in lib.iter() {
            assert!(!machine_summary(m).is_empty());
        }
        assert_eq!(fmt_mm(143.43), "143.43");
        assert_eq!(fmt_mm(220.0), "220");
    }
}
