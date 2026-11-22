//! IO-04: the Orca/Bambu 3MF flavour (S12 §6, note §3.1 and §4).
//!
//! - Done-when 1: `paint::encode_state` / `decode_dominant` round-trip for states 0-32 and match
//!   the `CONST_FILAMENTS` table of OrcaSlicer `Model.cpp`.
//! - Done-when 2: `orca_cli_*` run `orca-slicer --info` / `--export-3mf` on the five scenes. The
//!   executable comes from `AMG_ORCA_SLICER` or `PATH`; without either the test is skipped
//!   (required with `AMG_REQUIRE_ORCA=1`).
//! - Done-when 3 (manual): `AMG_IO04_FIXTURE_DIR` makes `write_fixtures` write the five scenes,
//!   among them the 4-filament per-face test part (steps in the handoff report).
//! - Done-when 4: with `paint_color` and `model_settings.config` removed, the output passes our
//!   strict reader and, with `AMG_3MF_PYTHON`, `scripts/io/validate_3mf.py`.
#![allow(clippy::unwrap_used, clippy::expect_used, reason = "tests")]

use std::collections::BTreeMap;
use std::sync::Arc;

use amg_core::{Point3, Transform, Vec3};
use amg_io::paint::{MAX_STATE, decode_dominant, encode_state};
use amg_io::{
    ExportOptions, Format, ImportOptions, MaterialSlot, NO_SLOT, PartSubtype, Plate, Scene,
    SceneNode, SceneObject, Sink, SlicerFlavor, Source, export, import,
};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, Rgba8};

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn opts() -> ExportOptions {
    let mut o = ExportOptions::default();
    o.threemf.flavor = SlicerFlavor::Orca;
    o
}

fn write(scene: &Scene) -> Vec<u8> {
    let mut out = Vec::new();
    export(
        scene,
        Sink::Memory(&mut out),
        Format::ThreeMf,
        &opts(),
        &ctx(),
    )
    .unwrap();
    out
}

fn read(bytes: &[u8]) -> Scene {
    import(
        Source::Bytes(bytes),
        Some(Format::ThreeMf),
        &ImportOptions::default(),
        &ctx(),
    )
    .unwrap()
    .scene
}

fn cuboid(size: [f64; 3]) -> IndexedMesh {
    let [x, y, z] = size;
    let p = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(x, 0.0, 0.0),
        Point3::new(x, y, 0.0),
        Point3::new(0.0, y, 0.0),
        Point3::new(0.0, 0.0, z),
        Point3::new(x, 0.0, z),
        Point3::new(x, y, z),
        Point3::new(0.0, y, z),
    ];
    let t = vec![
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [3, 0, 4],
        [3, 4, 7],
    ];
    IndexedMesh::new(p, t).unwrap()
}

const COLORS: [[u8; 3]; 4] = [[220, 50, 40], [40, 110, 220], [60, 190, 80], [240, 200, 30]];

fn slots(n: usize) -> Vec<MaterialSlot> {
    (0..n)
        .map(|k| MaterialSlot {
            name: format!("PLA {}", k + 1),
            color: Rgba8([COLORS[k][0], COLORS[k][1], COLORS[k][2], 255]),
            filament: None,
        })
        .collect()
}

fn at(x: f64, y: f64) -> Transform {
    Transform::from_translation(Vec3::new(x, y, 0.0))
}

/// `n` cubes, cube `k` on filament `k + 1`.
fn per_object(n: usize) -> Scene {
    let mut s = Scene {
        materials: slots(n),
        ..Scene::default()
    };
    for k in 0..n {
        let mut o = SceneObject::new(Arc::new(cuboid([10.0, 10.0, 10.0])));
        o.name = Some(format!("Cube {}", k + 1));
        o.material = Some(k as u16);
        s.objects.push(o);
        let mut node = SceneNode::mesh(k as u32, at(20.0 * k as f64 + 100.0, 100.0));
        node.name = Some(format!("Cube {}", k + 1));
        s.roots.push(node);
    }
    s
}

/// One cube, object filament 1, its 12 faces spread over filaments 1-`n`.
fn per_face(n: usize) -> Scene {
    let mut s = Scene {
        materials: slots(n),
        ..Scene::default()
    };
    let mut o = SceneObject::new(Arc::new(cuboid([20.0, 20.0, 20.0])));
    o.name = Some("Painted cube".into());
    o.material = Some(0);
    let per = 12 / n;
    let faces: Vec<u16> = (0..12).map(|f| (f / per).min(n - 1) as u16).collect();
    o.face_materials = Some(faces.into());
    s.objects.push(o);
    s.roots.push(SceneNode::mesh(0, at(128.0, 128.0)));
    s
}

/// Two plates: a two-colour cube on plate 1, a filament-3 block and a modifier on plate 2.
fn two_plates() -> Scene {
    let mut s = per_face(2);
    s.materials = slots(3);
    let mut b = SceneObject::new(Arc::new(cuboid([15.0, 15.0, 15.0])));
    b.name = Some("Filament 3 block".into());
    b.material = Some(2);
    s.objects.push(b);
    let mut m = SceneObject::new(Arc::new(cuboid([5.0, 5.0, 5.0])));
    m.name = Some("Modifier".into());
    m.part_subtype = PartSubtype::Modifier;
    m.material = Some(2);
    s.objects.push(m);
    s.roots.push(SceneNode::mesh(1, at(300.0, 128.0)));
    s.roots.push(SceneNode::mesh(2, at(330.0, 128.0)));
    s.plates = vec![
        Plate {
            name: Some("First".into()),
            roots: vec![0],
        },
        Plate {
            name: None,
            roots: vec![1, 2],
        },
    ];
    s
}

/// The five scenes of Done-when 2: (file name, scene, expected extruder per object).
fn five_scenes() -> Vec<(&'static str, Scene, Vec<u8>)> {
    vec![
        ("io04-1-filament.3mf", per_object(1), vec![1]),
        ("io04-2-filaments.3mf", per_object(2), vec![1, 2]),
        ("io04-3-face.3mf", per_face(3), vec![1]),
        ("io04-4-face.3mf", per_face(4), vec![1]),
        ("io04-2-plates.3mf", two_plates(), vec![1, 3, 3]),
    ]
}

/// `paint_color` value → count, over every triangle of a model text.
fn paint_histogram(model: &str) -> BTreeMap<String, usize> {
    let mut h = BTreeMap::new();
    for t in model.split("<triangle ").skip(1) {
        let t = &t[..t.find('>').unwrap_or(t.len())];
        if let Some(i) = t.find("paint_color=\"") {
            let v = &t[i + 13..];
            *h.entry(v[..v.find('"').unwrap()].to_owned()).or_insert(0) += 1;
        }
    }
    h
}

/// The painted-face histogram a scene should produce: paint string → faces, over every placed
/// mesh object once (faces on the object's own filament are unpainted).
fn expected_histogram(s: &Scene) -> BTreeMap<String, usize> {
    let mut h = BTreeMap::new();
    let mut used = std::collections::BTreeSet::new();
    for (i, _) in s.instances() {
        if !used.insert(i) {
            continue;
        }
        let o = &s.objects[i as usize];
        let own = o.material.map_or(1, |m| m as u8 + 1);
        if let Some(fm) = &o.face_materials {
            for &sl in fm.iter().filter(|&&x| x != NO_SLOT) {
                let f = sl as u8 + 1;
                if f != own {
                    *h.entry(encode_state(f)).or_insert(0) += 1;
                }
            }
        }
    }
    h
}

// -------------------------------------------------------------------------------- Done-when 1

/// OrcaSlicer `Model.cpp` `CONST_FILAMENTS` (fetched from `main`, 2026-11-23).
const CONST_FILAMENTS: [&str; 33] = [
    "", "4", "8", "0C", "1C", "2C", "3C", "4C", "5C", "6C", "7C", "8C", "9C", "AC", "BC", "CC",
    "DC", "EC", "0FC", "1FC", "2FC", "3FC", "4FC", "5FC", "6FC", "7FC", "8FC", "9FC", "AFC", "BFC",
    "CFC", "DFC", "EFC",
];

#[test]
fn paint_states_match_const_filaments_and_round_trip() {
    for (state, s) in CONST_FILAMENTS.iter().enumerate() {
        let state = state as u8;
        assert_eq!(encode_state(state), *s, "encode {state}");
        assert_eq!(decode_dominant(s), Ok((state, false)), "decode {state}");
    }
    assert_eq!(MAX_STATE, 33);
    assert_eq!(decode_dominant(&encode_state(33)), Ok((33, false)));
}

// -------------------------------------------------------------------------------- structure

fn entry(bytes: &[u8], name: &str) -> Option<String> {
    use std::io::Read;
    let mut z = zip::ZipArchive::new(std::io::Cursor::new(bytes)).unwrap();
    let mut s = String::new();
    z.by_name(name).ok()?.read_to_string(&mut s).unwrap();
    Some(s)
}

#[test]
fn colour_groups_object_pids_and_paint_are_written() {
    let bytes = write(&per_face(4));
    let model = entry(&bytes, "3D/3dmodel.model").unwrap();
    for g in 1..=4 {
        assert!(model.contains(&format!("<m:colorgroup id=\"{g}\">")), "{g}");
    }
    assert!(model.contains("xmlns:m="));
    // Object id 5, filament 1: pid 1.
    assert!(model.contains("<object id=\"5\" type=\"model\" name=\"Painted cube\" pid=\"1\""));
    // 3 faces per filament: filament 1 is the object default (unpainted), 2-4 are painted.
    assert_eq!(paint_histogram(&model), expected_histogram(&per_face(4)));
    assert_eq!(
        paint_histogram(&model),
        BTreeMap::from([("8".into(), 3), ("0C".into(), 3), ("1C".into(), 3)])
    );
    let cfg = entry(&bytes, "Metadata/model_settings.config").unwrap();
    assert!(cfg.contains("<metadata key=\"name\" value=\"Painted cube\"/>"));
    assert!(cfg.contains("<metadata key=\"extruder\" value=\"1\"/>"));
    assert!(!cfg.contains("<plate>"));
    assert!(!model.contains("<basematerials"));
    assert!(!model.contains("amg:filament"));
}

#[test]
fn per_object_extruders_names_subtypes_and_plates() {
    let bytes = write(&two_plates());
    let cfg = entry(&bytes, "Metadata/model_settings.config").unwrap();
    assert!(cfg.contains("subtype=\"modifier_part\""));
    assert_eq!(cfg.matches("<plate>").count(), 2);
    assert!(cfg.contains("<metadata key=\"plater_name\" value=\"First\"/>"));
    let back = read(&bytes);
    assert_eq!(back.plates.len(), 2);
    assert_eq!(back.plates[0].name.as_deref(), Some("First"));
    assert_eq!(back.plates[0].roots, [0]);
    assert_eq!(back.plates[1].roots, [1, 2]);
    let fil: Vec<_> = back.objects.iter().map(|o| o.filament).collect();
    assert_eq!(fil, [Some(1), Some(3), Some(3)]);
    assert_eq!(back.objects[2].part_subtype, PartSubtype::Modifier);
    assert_eq!(back.objects[1].name.as_deref(), Some("Filament 3 block"));
    // Per-face filaments come back as slots.
    let faces = back.objects[0].face_materials.as_ref().unwrap();
    // Unpainted faces read back as "object default" (the object's filament 1).
    let fils: Vec<u8> = faces
        .iter()
        .map(|&s| {
            if s == NO_SLOT {
                1
            } else {
                back.materials[s as usize].filament.unwrap()
            }
        })
        .collect();
    assert_eq!(fils, [1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2]);
}

#[test]
fn grouped_scene_writes_one_object_with_parts() {
    let mut s = per_object(2);
    let mut g = SceneNode::group(
        at(0.0, 0.0),
        vec![
            SceneNode::mesh(0, at(0.0, 0.0)),
            SceneNode::mesh(1, at(20.0, 0.0)),
        ],
    );
    g.name = Some("Pair".into());
    s.roots = vec![g];
    let bytes = write(&s);
    let cfg = entry(&bytes, "Metadata/model_settings.config").unwrap();
    assert_eq!(cfg.matches("<object ").count(), 1);
    assert_eq!(cfg.matches("<part ").count(), 2);
    let back = read(&bytes);
    let fil: Vec<_> = back.objects.iter().map(|o| o.filament).collect();
    assert_eq!(fil, [Some(1), Some(2)]);
}

#[test]
fn orca_output_is_deterministic_and_plain_scenes_get_no_colour_groups() {
    let s = two_plates();
    assert_eq!(write(&s), write(&s));
    let mut plain = Scene::default();
    plain
        .objects
        .push(SceneObject::new(Arc::new(cuboid([5.0; 3]))));
    plain.roots.push(SceneNode::mesh(0, Transform::IDENTITY));
    let model = entry(&write(&plain), "3D/3dmodel.model").unwrap();
    assert!(!model.contains("colorgroup") && !model.contains("pid="));
}

// -------------------------------------------------------------------------------- Done-when 4

/// Removes the Orca-only parts: `paint_color` attributes and `Metadata/model_settings.config`.
fn strip_orca(bytes: &[u8]) -> Vec<u8> {
    let model = entry(bytes, "3D/3dmodel.model").unwrap();
    let mut out = String::new();
    let mut rest = model.as_str();
    while let Some(i) = rest.find(" paint_color=\"") {
        out.push_str(&rest[..i]);
        let tail = &rest[i + 14..];
        rest = &tail[tail.find('"').unwrap() + 1..];
    }
    out.push_str(rest);
    let ct = entry(bytes, "[Content_Types].xml").unwrap().replace(
        " <Default Extension=\"config\" ContentType=\"text/xml\"/>\n",
        "",
    );
    let rels = entry(bytes, "_rels/.rels").unwrap();
    use std::io::Write;
    let mut z = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
    let o =
        zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Stored);
    for (n, d) in [
        ("[Content_Types].xml", ct.as_bytes()),
        ("_rels/.rels", rels.as_bytes()),
        ("3D/3dmodel.model", out.as_bytes()),
    ] {
        z.start_file(n, o).unwrap();
        z.write_all(d).unwrap();
    }
    z.finish().unwrap().into_inner()
}

#[test]
fn stripped_output_passes_the_strict_reader() {
    for (name, scene, _) in five_scenes() {
        let stripped = strip_orca(&write(&scene));
        assert!(!String::from_utf8_lossy(&stripped).contains("paint_color"));
        let mut o = ImportOptions::default();
        o.threemf.strict = true;
        import(Source::Bytes(&stripped), Some(Format::ThreeMf), &o, &ctx())
            .unwrap_or_else(|e| panic!("{name}: {e}"));
    }
}

#[test]
fn stripped_output_passes_validate_3mf_py() {
    let python = std::env::var_os("AMG_3MF_PYTHON");
    let required = std::env::var("AMG_REQUIRE_3MF_VALIDATOR").is_ok_and(|v| v == "1");
    let Some(python) = python else {
        assert!(
            !required,
            "AMG_REQUIRE_3MF_VALIDATOR=1 but AMG_3MF_PYTHON is not set"
        );
        eprintln!("skipped: set AMG_3MF_PYTHON to a Python with lib3mf and xmlschema");
        return;
    };
    let dir = tempfile::tempdir().unwrap();
    let mut files = Vec::new();
    for (name, scene, _) in five_scenes() {
        let p = dir.path().join(name);
        std::fs::write(&p, strip_orca(&write(&scene))).unwrap();
        files.push(p);
    }
    let script =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scripts/io/validate_3mf.py");
    let out = std::process::Command::new(python)
        .arg("-I")
        .arg(&script)
        .args(&files)
        .output()
        .unwrap();
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    eprintln!("{text}");
    assert!(out.status.success(), "validate_3mf.py failed:\n{text}");
    assert_eq!(text.matches("OK ").count(), files.len(), "{text}");
}

// -------------------------------------------------------------------------------- fixtures

/// Done-when 3 (manual): writes the five scenes to `AMG_IO04_FIXTURE_DIR`.
#[test]
fn write_fixtures() {
    let Some(d) = std::env::var_os("AMG_IO04_FIXTURE_DIR") else {
        return;
    };
    let d = std::path::PathBuf::from(d);
    std::fs::create_dir_all(&d).unwrap();
    for (name, scene, _) in five_scenes() {
        std::fs::write(d.join(name), write(&scene)).unwrap();
    }
}

// -------------------------------------------------------------------------------- Done-when 2

/// The OrcaSlicer executable: `AMG_ORCA_SLICER`, else `orca-slicer` on `PATH`.
fn orca_exe() -> Option<std::path::PathBuf> {
    if let Some(p) = std::env::var_os("AMG_ORCA_SLICER") {
        return Some(p.into());
    }
    let path = std::env::var_os("PATH")?;
    std::env::split_paths(&path)
        .flat_map(|d| [d.join("orca-slicer.exe"), d.join("orca-slicer")])
        .find(|p| p.is_file())
}

/// Values of `<metadata key="extruder" value="N"/>` that are direct children of an `<object>`
/// of `model_settings.config` (not of its `<part>`s), in file order.
fn object_extruders(cfg: &str) -> Vec<u8> {
    let mut out = Vec::new();
    let mut in_part = false;
    for line in cfg.lines().map(str::trim) {
        if line.starts_with("<part ") {
            in_part = true;
        } else if line.starts_with("</part>") {
            in_part = false;
        } else if !in_part && line.starts_with("<metadata key=\"extruder\"") {
            let v = line.split("value=\"").nth(1).unwrap();
            out.push(v[..v.find('"').unwrap()].parse().unwrap());
        }
    }
    out
}

/// Done-when 2 (skipped without OrcaSlicer, required with `AMG_REQUIRE_ORCA=1`): for the five
/// scenes `orca-slicer --info` lists the expected objects and parts, and its `--export-3mf`
/// re-export has the expected extruder per object and the same `paint_color` histogram.
#[test]
fn orca_cli_reads_the_five_scenes() {
    let Some(exe) = orca_exe() else {
        assert!(
            std::env::var("AMG_REQUIRE_ORCA").map_or(true, |v| v != "1"),
            "AMG_REQUIRE_ORCA=1 but neither AMG_ORCA_SLICER nor orca-slicer on PATH is set"
        );
        eprintln!("skipped: set AMG_ORCA_SLICER to orca-slicer(.exe) or put it on PATH");
        return;
    };
    let dir = tempfile::tempdir().unwrap();
    for (name, scene, extruders) in five_scenes() {
        let src = dir.path().join(name);
        std::fs::write(&src, write(&scene)).unwrap();

        // --info: one block per object (`[file]` header), `number_of_parts` per object.
        let info = std::process::Command::new(&exe)
            .arg("--info")
            .arg(&src)
            .output()
            .unwrap();
        let text = String::from_utf8_lossy(&info.stdout);
        let blocks = text.matches(&format!("[{name}]")).count();
        let parts: usize = text
            .lines()
            .filter_map(|l| l.strip_prefix("number_of_parts ="))
            .map(|v| v.trim().parse::<usize>().unwrap())
            .sum();
        let normal = scene
            .instances()
            .iter()
            .filter(|(i, _)| scene.objects[*i as usize].part_subtype == PartSubtype::NormalPart)
            .count();
        assert_eq!(blocks, scene.instances().len(), "{name} objects:\n{text}");
        assert_eq!(parts, normal, "{name} parts:\n{text}");

        // --export-3mf re-export.
        let out = dir.path().join(format!("out-{name}"));
        std::fs::create_dir_all(&out).unwrap();
        let st = std::process::Command::new(&exe)
            .arg("--outputdir")
            .arg(&out)
            .arg("--export-3mf")
            .arg("re.3mf")
            .arg(&src)
            .output()
            .unwrap();
        assert!(st.status.success(), "{name}: {:?}", st.status);
        let re = std::fs::read(out.join("re.3mf")).unwrap();
        let cfg = entry(&re, "Metadata/model_settings.config").unwrap();
        let mut got = object_extruders(&cfg);
        let mut want = extruders.clone();
        got.sort_unstable();
        want.sort_unstable();
        assert_eq!(got, want, "{name} extruders:\n{cfg}");

        let mut z = zip::ZipArchive::new(std::io::Cursor::new(&re)).unwrap();
        let names: Vec<String> = (0..z.len())
            .map(|i| z.by_index(i).unwrap().name().unwrap().into_owned())
            .filter(|n| n.ends_with(".model"))
            .collect();
        let mut hist: BTreeMap<String, usize> = BTreeMap::new();
        for n in names {
            let m = entry(&re, &n).unwrap();
            for (k, v) in paint_histogram(&m) {
                *hist.entry(k).or_insert(0) += v;
            }
        }
        assert_eq!(hist, expected_histogram(&scene), "{name} paint_color");
    }
}
