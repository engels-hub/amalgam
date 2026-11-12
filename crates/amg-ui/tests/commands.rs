//! UI-02 tests: command registry, toolbar/menu placement, shortcuts and S-key search through
//! `AppHarness` (kittest, no GPU).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::atomic::{AtomicUsize, Ordering};

use amg_ui::command::{CmdAction, CmdId, CommandDef, Enablement, Enabler, RegistryError};
use amg_ui::design::SKETCH_CONTEXT;
use amg_ui::module::{GroupId, PanelId, Registrar, UiModule};
use amg_ui::panel::{DockSlot, Panel, PanelDef};
use amg_ui::shortcut::{ShortcutMap, ShortcutOverrides, ShortcutScope};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable;
use amg_ui::{AppCx, UiRegistry};
use egui::Key;

static REMESH: AtomicUsize = AtomicUsize::new(0);
static LOCKED: AtomicUsize = AtomicUsize::new(0);
static ZETA_ON: AtomicUsize = AtomicUsize::new(0);
static ZETA_OFF: AtomicUsize = AtomicUsize::new(0);
static SKETCH_L: AtomicUsize = AtomicUsize::new(0);
static GLOBAL_L: AtomicUsize = AtomicUsize::new(0);
static PINNED: AtomicUsize = AtomicUsize::new(0);
static EXPORT: AtomicUsize = AtomicUsize::new(0);
static OVERRIDDEN: AtomicUsize = AtomicUsize::new(0);
static TOGGLE: AtomicUsize = AtomicUsize::new(0);
static DYN: AtomicUsize = AtomicUsize::new(0);

inventory::submit! { UiModule { id: "zz_cmds", register } }

fn bump(c: &'static AtomicUsize) {
    c.fetch_add(1, Ordering::SeqCst);
}

fn needs_mesh(_: &amg_ui::CmdEnv<'_>) -> Enablement {
    Enablement::disabled("Needs a mesh body")
}

struct TextPanel(String);

impl Panel for TextPanel {
    fn ui(&mut self, ui: &mut egui::Ui, _cx: &mut AppCx<'_>) {
        ui.add(egui::TextEdit::singleline(&mut self.0).hint_text("type here"));
    }
}

fn register(r: &mut Registrar<'_>) {
    r.command(
        CommandDef::new("test.remesh", "Remesh", CmdAction::Run(|_| bump(&REMESH)))
            .with_keywords(&["retopology"]),
    );
    r.command(CommandDef::new(
        "test.remove_shell",
        "Remove Shell Sections",
        CmdAction::Run(|_| {}),
    ));
    r.command(
        CommandDef::new(
            "test.locked",
            "Remesh Locked",
            CmdAction::Run(|_| bump(&LOCKED)),
        )
        .with_enabler(Enabler::When(needs_mesh))
        .in_group(GroupId("design.solid.create"), 20),
    );
    r.command(
        CommandDef::new(
            "test.zeta_off",
            "Zeta Off",
            CmdAction::Run(|_| bump(&ZETA_OFF)),
        )
        .with_enabler(Enabler::When(needs_mesh)),
    );
    r.command(CommandDef::new(
        "test.zeta_on",
        "Zeta Extended On",
        CmdAction::Run(|_| bump(&ZETA_ON)),
    ));
    r.command(
        CommandDef::new(
            "test.sketch_line",
            "Sketch Line",
            CmdAction::Run(|_| bump(&SKETCH_L)),
        )
        .with_shortcut("sketch:L"),
    );
    r.command(
        CommandDef::new(
            "test.global_l",
            "Global L",
            CmdAction::Run(|_| bump(&GLOBAL_L)),
        )
        .with_shortcut("L"),
    );
    r.command(
        CommandDef::new(
            "test.pinned",
            "Pinned Tool",
            CmdAction::Run(|_| bump(&PINNED)),
        )
        .placed(amg_ui::Placement::Toolbar {
            group: GroupId("design.solid.modify"),
            pinned: true,
            order: 0,
        }),
    );
    r.command(
        CommandDef::new(
            "test.export",
            "Export Thing",
            CmdAction::Run(|_| bump(&EXPORT)),
        )
        .in_menu("File/Export", 0)
        .with_shortcut("Ctrl+Alt+E"),
    );
    r.command(
        CommandDef::new(
            "test.overridden",
            "Overridden",
            CmdAction::Run(|_| bump(&OVERRIDDEN)),
        )
        .with_shortcut("Ctrl+Alt+O"),
    );
    r.command(
        CommandDef::new(
            "test.toggle",
            "Show Thing",
            CmdAction::Toggle {
                get: |_| TOGGLE.load(Ordering::SeqCst) % 2 == 1,
                set: |_, _| bump(&TOGGLE),
            },
        )
        .in_menu("View", 0),
    );
    r.panel(PanelDef {
        id: PanelId("test.text"),
        title: "Text",
        icon: amg_ui::module::IconId::NONE,
        slot: DockSlot::Right,
        workspaces: None,
        create: || Box::new(TextPanel(String::new())),
    });
}

/// Steps a few frames. `drive_until_idle` cannot be used while a text field is focused:
/// the blinking cursor keeps requesting repaints.
fn settle(h: &mut AppHarness) {
    for _ in 0..6 {
        h.kittest().step();
    }
}

fn count(c: &AtomicUsize) -> usize {
    c.load(Ordering::SeqCst)
}

fn open_search(h: &mut AppHarness) {
    h.kittest().key_press(Key::S);
    settle(h);
    assert!(h.app().search().is_open(), "S opens the search popup");
}

fn type_query(h: &mut AppHarness, q: &str) {
    h.kittest().event(egui::Event::Text(q.to_owned()));
    settle(h);
    assert_eq!(h.app().search().query(), q);
}

#[test]
fn s_remsh_lists_remesh_first_and_enter_runs_it() {
    let mut h = AppHarness::new();
    settle(&mut h);
    let before = count(&REMESH);
    open_search(&mut h);
    type_query(&mut h, "remsh");
    let hits = h.app().search().results();
    assert_eq!(
        hits.first().map(|x| x.id),
        Some(CmdId("test.remesh")),
        "{hits:?}"
    );
    assert!(
        h.kittest().query_by_label("Remesh").is_some(),
        "the first row is shown"
    );
    h.kittest().key_press(Key::Enter);
    settle(&mut h);
    assert_eq!(count(&REMESH), before + 1, "Enter ran test.remesh once");
    assert!(!h.app().search().is_open(), "popup closes after running");
    assert_eq!(
        h.app().recent_commands().rank(CmdId("test.remesh")),
        Some(0)
    );
}

#[test]
fn esc_closes_the_search_without_running() {
    let mut h = AppHarness::new();
    open_search(&mut h);
    type_query(&mut h, "remesh");
    let before = count(&REMESH);
    h.kittest().key_press(Key::Escape);
    settle(&mut h);
    assert!(!h.app().search().is_open());
    assert_eq!(count(&REMESH), before);
}

#[test]
fn s_while_the_search_is_open_does_not_toggle_it() {
    let mut h = AppHarness::new();
    open_search(&mut h);
    h.kittest().key_press(Key::S);
    settle(&mut h);
    assert!(h.app().search().is_open());
}

#[test]
fn disabled_commands_show_reason_and_are_skipped_by_enter() {
    let mut h = AppHarness::new();
    settle(&mut h);
    open_search(&mut h);
    type_query(&mut h, "zeta");
    let ids: Vec<_> = h.app().search().results().iter().map(|x| x.id.0).collect();
    assert_eq!(
        ids.first(),
        Some(&"test.zeta_off"),
        "disabled ranks first: {ids:?}"
    );
    assert!(
        h.kittest()
            .query_by_label_contains("Needs a mesh body")
            .is_some(),
        "the reason is shown in the row"
    );
    let (off, on) = (count(&ZETA_OFF), count(&ZETA_ON));
    h.kittest().key_press(Key::Enter);
    settle(&mut h);
    assert_eq!(count(&ZETA_OFF), off, "disabled command never runs");
    assert_eq!(
        count(&ZETA_ON),
        on + 1,
        "Enter skipped to the first enabled row"
    );

    // Only a disabled match: Enter does nothing and the popup stays.
    open_search(&mut h);
    type_query(&mut h, "locked");
    let locked = count(&LOCKED);
    h.kittest().key_press(Key::Enter);
    settle(&mut h);
    assert_eq!(count(&LOCKED), locked);
    assert!(h.app().search().is_open());
}

#[test]
fn disabled_toolbar_button_shows_its_reason_as_tooltip() {
    let mut h = AppHarness::new();
    h.kittest()
        .ctx
        .global_style_mut(|s| s.interaction.tooltip_delay = 0.0);
    settle(&mut h);
    h.kittest().get_by_label("CREATE ⏷").click();
    settle(&mut h);
    let locked = count(&LOCKED);
    h.kittest().get_by_label("Remesh Locked").hover();
    for _ in 0..4 {
        h.kittest().step();
    }
    assert!(
        h.kittest()
            .query_by_label_contains("Needs a mesh body")
            .is_some(),
        "tooltip text missing"
    );
    h.kittest().get_by_label("Remesh Locked").click();
    settle(&mut h);
    assert_eq!(
        count(&LOCKED),
        locked,
        "clicking a disabled button does nothing"
    );
}

#[test]
fn pinned_toolbar_button_and_menu_item_run_their_commands() {
    let mut h = AppHarness::new();
    settle(&mut h);
    let pinned = count(&PINNED);
    h.kittest().get_by_label("Pinned Tool").click();
    settle(&mut h);
    assert_eq!(count(&PINNED), pinned + 1);

    let export = count(&EXPORT);
    h.kittest().get_by_label("File").click();
    settle(&mut h);
    h.kittest().get_by_label_contains("Export").click();
    settle(&mut h);
    h.kittest().get_by_label_contains("Export Thing").click();
    settle(&mut h);
    assert_eq!(count(&EXPORT), export + 1);
    assert!(
        h.kittest().query_by_label("View").is_some(),
        "second top-level menu"
    );
}

#[test]
fn toggle_command_flips_its_state() {
    let mut h = AppHarness::new();
    let before = count(&TOGGLE);
    h.run_command(CmdId("test.toggle"));
    assert_eq!(count(&TOGGLE), before + 1);
}

#[test]
fn sketch_scope_beats_global_for_the_same_key() {
    let mut h = AppHarness::new();
    settle(&mut h);
    let (s0, g0) = (count(&SKETCH_L), count(&GLOBAL_L));
    h.kittest().key_press(Key::L);
    settle(&mut h);
    assert_eq!(
        (count(&SKETCH_L), count(&GLOBAL_L)),
        (s0, g0 + 1),
        "no sketch: global L"
    );

    h.app_mut().with_cx(|cx| cx.enter_context(SKETCH_CONTEXT));
    settle(&mut h);
    h.kittest().key_press(Key::L);
    settle(&mut h);
    assert_eq!(
        (count(&SKETCH_L), count(&GLOBAL_L)),
        (s0 + 1, g0 + 1),
        "sketch: sketch L"
    );

    h.app_mut().with_cx(|cx| cx.exit_context(SKETCH_CONTEXT));
    settle(&mut h);
    h.kittest().key_press(Key::L);
    settle(&mut h);
    assert_eq!((count(&SKETCH_L), count(&GLOBAL_L)), (s0 + 1, g0 + 2));
}

#[test]
fn single_letters_are_ignored_while_a_text_field_has_focus() {
    use egui::accesskit::Role;
    let mut h = AppHarness::new();
    h.app_mut().open_panel(PanelId("test.text"));
    settle(&mut h);
    h.kittest().get_by_role(Role::TextInput).click();
    settle(&mut h);
    assert!(
        h.kittest().ctx.egui_wants_keyboard_input(),
        "text field focused"
    );
    let g0 = count(&GLOBAL_L);
    h.kittest().key_press(Key::L);
    settle(&mut h);
    assert_eq!(count(&GLOBAL_L), g0, "L is text, not a command");
    // Ctrl/Alt chords still reach the map.
    let e0 = count(&EXPORT);
    h.kittest()
        .key_press_modifiers(egui::Modifiers::COMMAND | egui::Modifiers::ALT, Key::E);
    settle(&mut h);
    assert_eq!(count(&EXPORT), e0 + 1);
}

#[test]
fn shortcuts_toml_overrides_replace_defaults() {
    let mut h = AppHarness::new();
    h.app_mut().set_shortcut_overrides(
        ShortcutOverrides::parse("[shortcuts]\n\"test.overridden\" = \"Ctrl+Alt+K\"\n").unwrap(),
    );
    settle(&mut h);
    let o0 = count(&OVERRIDDEN);
    h.kittest()
        .key_press_modifiers(egui::Modifiers::COMMAND | egui::Modifiers::ALT, Key::O);
    settle(&mut h);
    assert_eq!(count(&OVERRIDDEN), o0, "old default is gone");
    h.kittest()
        .key_press_modifiers(egui::Modifiers::COMMAND | egui::Modifiers::ALT, Key::K);
    settle(&mut h);
    assert_eq!(count(&OVERRIDDEN), o0 + 1);
}

#[test]
fn shortcuts_file_loads_and_a_bad_file_keeps_the_bindings() {
    let dir = std::env::temp_dir().join(format!("amg-ui02-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let good = dir.join("shortcuts.toml");
    std::fs::write(&good, "[shortcuts]\n\"edit.undo\" = \"Ctrl+Alt+U\"\n").unwrap();
    let mut h = AppHarness::new();
    h.app_mut().load_shortcuts_file(&good).unwrap();
    assert_eq!(
        h.app()
            .shortcuts()
            .display_of(CmdId("edit.undo"))
            .as_deref(),
        Some("Ctrl+Alt+U")
    );
    let bad = dir.join("bad.toml");
    std::fs::write(&bad, "[shortcuts]\n\"edit.undo\" = \"Ctrl+Nope\"\n").unwrap();
    assert!(h.app_mut().load_shortcuts_file(&bad).is_err());
    assert_eq!(
        h.app()
            .shortcuts()
            .display_of(CmdId("edit.undo"))
            .as_deref(),
        Some("Ctrl+Alt+U"),
        "a bad file leaves bindings alone"
    );
    assert!(
        h.app_mut()
            .load_shortcuts_file(&dir.join("missing.toml"))
            .is_ok()
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn ctrl_z_still_undoes_through_the_shortcut_map() {
    use amg_doc::Command as DocCommand;
    let mut h = AppHarness::new();
    h.app_mut().submit(amg_ui::CommitBatch {
        label: "Add a".to_owned(),
        commands: vec![DocCommand::AddParam {
            name: "a".to_owned(),
            expr: "1".to_owned(),
            unit: None,
            comment: String::new(),
        }],
    });
    settle(&mut h);
    assert!(h.document().history().can_undo());
    h.kittest()
        .key_press_modifiers(egui::Modifiers::COMMAND, Key::Z);
    settle(&mut h);
    assert!(!h.document().history().can_undo());
    h.kittest()
        .key_press_modifiers(egui::Modifiers::COMMAND | egui::Modifiers::SHIFT, Key::Z);
    settle(&mut h);
    assert!(h.document().history().can_undo(), "Ctrl+Shift+Z redoes");
}

#[test]
fn dynamic_commands_register_search_run_and_unregister() {
    let mut h = AppHarness::new();
    settle(&mut h);
    let bad = CommandDef::new("addin.hello", "Hello", CmdAction::Run(|_| {}));
    assert_eq!(
        h.app_mut().registry_mut().register_dynamic(bad),
        Err(RegistryError::BadPrefix("addin.hello"))
    );
    let def = || {
        CommandDef::new(
            "script.hello",
            "Hello Addin",
            CmdAction::Dyn(std::sync::Arc::new(|_| bump(&DYN))),
        )
    };
    h.app_mut().registry_mut().register_dynamic(def()).unwrap();
    assert_eq!(
        h.app_mut().registry_mut().register_dynamic(def()),
        Err(RegistryError::Duplicate("script.hello"))
    );
    settle(&mut h);
    open_search(&mut h);
    type_query(&mut h, "hello");
    assert_eq!(
        h.app().search().results().first().map(|x| x.id),
        Some(CmdId("script.hello"))
    );
    h.kittest().key_press(Key::Enter);
    settle(&mut h);
    assert_eq!(count(&DYN), 1);

    assert!(
        h.app_mut()
            .registry_mut()
            .unregister_dynamic(CmdId("script.hello"))
    );
    assert!(
        !h.app_mut()
            .registry_mut()
            .unregister_dynamic(CmdId("script.hello"))
    );
    assert!(
        !h.app_mut()
            .registry_mut()
            .unregister_dynamic(CmdId("test.remesh")),
        "static commands cannot be unregistered"
    );
    h.run_command(CmdId("script.hello"));
    assert_eq!(count(&DYN), 1, "unknown command is ignored");
}

#[test]
fn shortcut_map_scope_resolution_without_the_app() {
    let reg = UiRegistry::build();
    let map = ShortcutMap::build(&reg, &ShortcutOverrides::none());
    let none = egui::Modifiers::NONE;
    use ShortcutScope::{Global, Sketch, Viewport};
    assert_eq!(
        map.resolve(Key::L, none, &[Sketch, Global]),
        Some(CmdId("test.sketch_line"))
    );
    assert_eq!(
        map.resolve(Key::L, none, &[Global]),
        Some(CmdId("test.global_l"))
    );
    assert_eq!(
        map.resolve(Key::L, none, &[Viewport, Global]),
        Some(CmdId("test.global_l"))
    );
    assert_eq!(
        map.resolve(Key::S, none, &[Global]),
        Some(CmdId("app.search"))
    );
    assert!(map.conflicts().is_empty(), "{:?}", map.conflicts());
}
