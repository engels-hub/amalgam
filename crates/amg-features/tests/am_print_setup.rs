//! AM-01: the `am.print_setup` record: round trip, `.amg` save/load/save, undo/redo through
//! `SetExt`, bad versions, and key allocation.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "tests"
)]

use std::sync::Arc;

use amg_am::machine::{MachineKey, MachineLibrary};
use amg_am::place::Placement;
use amg_core::glam::{DQuat, DVec2, DVec3};
use amg_doc::{
    BodyId, Command, CreatedId, Document, ExtRecord, FeatureId, SaveOptions, load, save,
};
use amg_features::am::{
    AmFeatureError, PRINT_SETUP_KIND, PRINT_SETUP_VERSION, PrintSetup, delete_setup_command,
    next_setup_key, setups,
};

fn body(n: u64) -> BodyId {
    BodyId {
        origin: FeatureId(n),
        slot: 0,
    }
}

fn machine(key: &str) -> amg_am::machine::Machine {
    MachineLibrary::builtin()
        .get(&MachineKey(key.into()))
        .unwrap()
        .clone()
}

fn sample() -> PrintSetup {
    let mut s = PrintSetup::new("Test setup", machine("builtin:creality/ender-3-v3-ke-kcm"));
    let p1 = s.plates[0].id;
    let p2 = s.add_plate("Plate 2");
    let rot = DQuat::from_axis_angle(DVec3::new(1.0, 2.0, 3.0).normalize(), 0.7);
    let a = s.add_body(
        p1,
        body(2),
        Placement::on_plate(rot, DVec2::new(110.1, 90.3)),
    );
    s.add_body(
        p1,
        body(3),
        Placement::on_plate(DQuat::IDENTITY, DVec2::new(30.0, 40.0)),
    );
    let c = s.add_body(
        p2,
        body(4),
        Placement::on_plate(rot, DVec2::new(1.0 / 3.0, 2.0)),
    );
    s.item_mut(a).unwrap().filament = Some(2);
    s.item_mut(c).unwrap().filament = Some(3);
    s.active_plate = p2;
    s
}

#[test]
fn record_round_trip_two_plates_three_items() {
    let s = sample();
    assert_eq!(s.plates.len(), 2);
    assert_eq!(s.plates.iter().map(|p| p.items.len()).sum::<usize>(), 3);
    let r = s.to_record();
    assert_eq!(
        (r.kind.as_str(), r.version),
        (PRINT_SETUP_KIND, PRINT_SETUP_VERSION)
    );
    let back = PrintSetup::from_record(&r).unwrap();
    assert_eq!(back, s);
    assert_eq!(back.to_record(), r);
}

#[test]
fn amg_save_load_save_is_byte_identical() {
    let dir = std::env::temp_dir().join(format!("amg-am01-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let mut doc = Document::new("am");
    let s = sample();
    doc.apply(s.add_command()).unwrap();
    let path = dir.join("a.amg");
    let design_json = |d: &amg_doc::Design| serde_json::to_vec(d).unwrap();
    let before = design_json(doc.design());
    save(
        &path,
        &Arc::new(doc.design().clone()),
        &SaveOptions::default(),
    )
    .unwrap();
    let loaded = load(&path).unwrap();
    assert!(loaded.warnings.is_empty(), "{:?}", loaded.warnings);
    assert_eq!(design_json(&loaded.design), before);
    let all = setups(&loaded.design);
    assert_eq!(all.len(), 1);
    assert_eq!(all[0].1.as_ref().unwrap(), &s);
    let path2 = dir.join("b.amg");
    save(
        &path2,
        &Arc::new(loaded.design.clone()),
        &SaveOptions::default(),
    )
    .unwrap();
    assert_eq!(design_json(&load(&path2).unwrap().design), before);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn set_ext_edits_undo_redo_to_equal_records() {
    let mut doc = Document::new("am");
    let mut s = sample();
    let out = doc.apply(s.add_command()).unwrap();
    let CreatedId::Ext(key) = out.created[0].clone() else {
        panic!()
    };
    let v1 = s.clone();
    s.add_body(
        s.plates[0].id,
        body(9),
        Placement::on_plate(DQuat::IDENTITY, DVec2::ZERO),
    );
    let v2 = s.clone();
    doc.apply(s.command(key.clone())).unwrap();
    let get = |d: &Document| PrintSetup::from_record(d.design().ext(&key).unwrap()).unwrap();
    assert_eq!(get(&doc), v2);
    assert!(doc.undo());
    assert_eq!(get(&doc), v1);
    assert!(doc.redo());
    assert_eq!(get(&doc), v2);
    doc.apply(delete_setup_command(key.clone())).unwrap();
    assert!(doc.design().ext(&key).is_none());
    assert!(doc.undo());
    assert_eq!(get(&doc), v2);
}

#[test]
fn bad_versions_are_rejected_without_panic() {
    let good = sample().to_record();
    let v0 = ExtRecord {
        version: 0,
        ..good.clone()
    };
    assert!(matches!(
        PrintSetup::from_record(&v0),
        Err(AmFeatureError::Decode(_))
    ));
    let future = ExtRecord {
        version: PRINT_SETUP_VERSION + 1,
        ..good.clone()
    };
    assert!(matches!(
        PrintSetup::from_record(&future),
        Err(AmFeatureError::TooNew(v)) if v == PRINT_SETUP_VERSION + 1
    ));
    for data in [
        serde_json::Value::Null,
        serde_json::json!({}),
        serde_json::json!({"name": 5}),
        serde_json::json!([1, 2]),
    ] {
        let r = ExtRecord {
            data,
            ..good.clone()
        };
        assert!(matches!(
            PrintSetup::from_record(&r),
            Err(AmFeatureError::Decode(_))
        ));
    }
    let wrong_kind = ExtRecord {
        kind: "fem.study".into(),
        ..good
    };
    assert!(PrintSetup::from_record(&wrong_kind).is_err());
}

#[test]
fn inconsistent_ids_are_rejected() {
    let mut s = sample();
    s.next_item_id = 1; // items 1..=3 exist
    assert!(matches!(
        PrintSetup::from_record(&s.to_record()),
        Err(AmFeatureError::Decode(_))
    ));
    let mut s = sample();
    s.active_plate = amg_features::am::PlateId(99);
    assert!(PrintSetup::from_record(&s.to_record()).is_err());
}

#[test]
fn ids_are_never_reused() {
    let mut s = sample();
    let ids: Vec<_> = s
        .plates
        .iter()
        .flat_map(|p| p.items.iter().map(|i| i.id))
        .collect();
    let last = *ids.iter().max().unwrap();
    assert!(s.remove_item(last));
    assert!(!s.remove_item(last));
    let again = s.add_body(
        s.plates[0].id,
        body(7),
        Placement::on_plate(DQuat::IDENTITY, DVec2::ZERO),
    );
    assert!(again > last);
}

#[test]
fn next_setup_key_never_reuses_a_deleted_key() {
    let mut doc = Document::new("am");
    let s = PrintSetup::new("s", machine("builtin:generic/fff-220"));
    let mut seen = Vec::new();
    for _ in 0..4 {
        let predicted = next_setup_key(doc.design());
        let out = doc.apply(s.add_command()).unwrap();
        let CreatedId::Ext(key) = out.created[0].clone() else {
            panic!()
        };
        assert_eq!(key, predicted);
        assert_eq!(key.kind, PRINT_SETUP_KIND);
        assert!(!seen.contains(&key), "key reused");
        // Delete the newest one each time; later keys must still be fresh.
        doc.apply(delete_setup_command(key.clone())).unwrap();
        seen.push(key);
    }
    assert!(setups(doc.design()).is_empty());
    // Undo of a delete brings the key back, but a fresh key is still above every seen key.
    let fresh = next_setup_key(doc.design());
    assert!(seen.iter().all(|k| k.id < fresh.id));
    let _ = Command::SetExt {
        key: fresh,
        record: None,
    };
}
