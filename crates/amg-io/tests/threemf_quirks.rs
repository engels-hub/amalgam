//! IO-03: synthetic 3MF cases for every reader feature and every note §9.2 row that applies.
//! Slicer-style packages (Orca/Bambu Production multi-part + `paint_color` +
//! `model_settings.config`, PrusaSlicer `mmu_segmentation` + `Slic3r_PE_model.config`) are
//! built here with the structure those slicers write.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    reason = "test code; a failed expectation is a test failure"
)]

mod threemf_common;

use std::io::{Cursor, Write};

use amg_core::units::LengthUnit;
use amg_io::{
    Format, ImportOptions, Imported, IoError, IoWarning, Location, NO_SLOT, NodeContent,
    ObjectKind, PartSubtype, Source, Thumbnail, UnitChoice, import, paint, registry,
};
use amg_jobs::JobCtx;
use threemf_common::*;

fn opts() -> ImportOptions {
    ImportOptions::default()
}

fn strict() -> ImportOptions {
    let mut o = ImportOptions::default();
    o.threemf.strict = true;
    o
}

fn read(bytes: &[u8], o: &ImportOptions) -> Result<Imported, IoError> {
    import(
        Source::Bytes(bytes),
        Some(Format::ThreeMf),
        o,
        &JobCtx::detached(),
    )
}

fn ok(bytes: &[u8]) -> Imported {
    read(bytes, &opts()).expect("import")
}

#[test]
fn registry_reports_3mf_import() {
    let i = registry()
        .iter()
        .find(|f| f.format == Format::ThreeMf)
        .unwrap();
    assert!(i.can_import);
    // Sniffed from content.
    let b = one_cube();
    let r = import(Source::Bytes(&b), None, &opts(), &JobCtx::detached()).expect("sniffed");
    assert_eq!(r.report.format, Some(Format::ThreeMf));
}

#[test]
fn a_cube_loads_in_strict_and_normal_mode() {
    let b = one_cube();
    for o in [opts(), strict()] {
        let r = read(&b, &o).expect("import");
        assert_eq!(r.scene.objects.len(), 1);
        assert_eq!(r.scene.objects[0].name.as_deref(), Some("cube"));
        assert_eq!(r.scene.objects[0].mesh.face_count(), 12);
        assert_eq!(r.report.triangles_read, 12);
        assert_eq!(r.report.vertices, 8);
        assert_eq!(r.report.declared_unit, Some(LengthUnit::Millimetre));
        assert!(r.report.warnings.is_empty(), "{:?}", r.report.warnings);
    }
}

#[test]
fn units_scale_vertices_and_translations() {
    let m = model(
        "",
        &format!(
            r#"<object id="1">{}</object>"#,
            cube_mesh(1.0, [0.0; 3], &no_attrs)
        ),
        r#"<item objectid="1" transform="1 0 0 0 1 0 0 0 1 2 0 0"/>"#,
    )
    .replace(r#"unit="millimeter""#, r#"unit="inch""#);
    let r = ok(&package(&m, &[]));
    assert_eq!(r.report.declared_unit, Some(LengthUnit::Inch));
    assert_eq!(r.report.applied_unit, Some(LengthUnit::Inch));
    let (_, t) = r.scene.instances()[0];
    assert_eq!(t.translation.x, 50.8);
    let ext = r.scene.world_aabb();
    assert!((ext.max.x - ext.min.x - 25.4).abs() < 1e-12);
    // The unit can be overridden.
    let mut o = opts();
    o.unit = UnitChoice::Assume(LengthUnit::Millimetre);
    let r = read(&package(&m, &[]), &o).expect("import");
    assert_eq!(r.scene.instances()[0].1.translation.x, 2.0);
    // No unit attribute = millimetres.
    let m2 = m.replace(r#"unit="inch""#, "");
    assert_eq!(
        ok(&package(&m2, &[])).report.declared_unit,
        Some(LengthUnit::Millimetre)
    );
}

#[test]
fn components_become_groups_and_can_be_flattened() {
    let m = model(
        "",
        &format!(
            r#"<object id="1" name="part">{}</object>
               <object id="2" name="assembly"><components>
                 <component objectid="1"/>
                 <component objectid="1" transform="1 0 0 0 1 0 0 0 1 20 0 0"/>
               </components></object>"#,
            cube_mesh(10.0, [0.0; 3], &no_attrs)
        ),
        r#"<item objectid="2" transform="1 0 0 0 1 0 0 0 1 0 0 5"/><item objectid="1"/>"#,
    );
    let b = package(&m, &[]);
    let r = read(&b, &strict()).expect("strict");
    assert_eq!(r.scene.objects.len(), 1, "the mesh is shared");
    assert_eq!(r.scene.roots.len(), 2);
    let NodeContent::Group(c) = &r.scene.roots[0].content else {
        panic!("group")
    };
    assert_eq!(c.len(), 2);
    assert_eq!(r.scene.roots[0].name.as_deref(), Some("assembly"));
    let inst = r.scene.instances();
    assert_eq!(inst.len(), 3);
    assert_eq!(inst[1].1.translation.x, 20.0);
    assert_eq!(inst[1].1.translation.z, 5.0);
    let mut o = opts();
    o.threemf.keep_components = false;
    let f = read(&b, &o).expect("flat");
    assert_eq!(f.scene.roots.len(), 3);
    assert!(
        f.scene
            .roots
            .iter()
            .all(|n| matches!(n.content, NodeContent::Mesh(0)))
    );
    assert_eq!(f.scene.instances(), inst);
}

#[test]
fn object_types_metadata_and_build_items() {
    let m = model(
        "",
        &format!(
            r#"<object id="1" type="support">{}</object><object id="2" type="other">{}</object>"#,
            cube_mesh(1.0, [0.0; 3], &no_attrs),
            cube_mesh(1.0, [0.0; 3], &no_attrs)
        ),
        r#"<item objectid="1"/>"#,
    )
    .replace(
        "<resources>",
        r#"<metadata name="Title">T</metadata><metadata name="Designer">D &amp; E</metadata><resources>"#,
    );
    let r = ok(&package(&m, &[]));
    assert_eq!(r.scene.objects.len(), 1, "only built objects are loaded");
    assert_eq!(r.scene.objects[0].kind, ObjectKind::Support);
    assert_eq!(r.scene.metadata.get("Title").map(String::as_str), Some("T"));
    assert_eq!(
        r.scene.metadata.get("Designer").map(String::as_str),
        Some("D & E")
    );
    // A build item of type other is an error in strict mode.
    let m2 = m.replace(r#"<item objectid="1"/>"#, r#"<item objectid="2"/>"#);
    assert!(matches!(
        read(&package(&m2, &[]), &strict()),
        Err(IoError::Malformed { .. })
    ));
}

#[test]
fn base_materials_object_and_triangle_level() {
    let tri = |k: usize| {
        if k == 3 {
            r#" pid="5" p1="1""#.to_owned()
        } else {
            String::new()
        }
    };
    let m = model(
        "",
        &format!(
            r##"<basematerials id="5"><base name="Red" displaycolor="#FF0000"/><base name="Blue" displaycolor="#0000FF80"/></basematerials>
               <object id="1" pid="5" pindex="0">{}</object>"##,
            cube_mesh(10.0, [0.0; 3], &tri)
        ),
        r#"<item objectid="1"/>"#,
    );
    let r = read(&package(&m, &[]), &strict()).expect("strict");
    let s = &r.scene;
    assert_eq!(s.materials.len(), 2);
    assert_eq!(s.materials[0].name, "Red");
    assert_eq!(s.materials[1].color.0, [0, 0, 255, 128]);
    let o = &s.objects[0];
    assert_eq!(o.material, Some(0));
    let f = o.face_materials.as_ref().expect("per-face");
    assert_eq!(f[3], 1);
    assert!(f.iter().enumerate().all(|(k, &v)| k == 3 || v == NO_SLOT));
    // pid without p1 means the object default.
    let m2 = m.replace(r#" pid="5" p1="1""#, r#" pid="5""#);
    assert!(
        ok(&package(&m2, &[])).scene.objects[0]
            .face_materials
            .is_none()
    );
}

#[test]
fn colour_groups_textures_composites_and_multiproperties() {
    let tri = |k: usize| match k {
        0 => r#" pid="2" p1="1""#.to_owned(),
        1 => r#" pid="4" p1="0" p2="1" p3="2""#.to_owned(),
        2 => r#" pid="6" p1="0""#.to_owned(),
        3 => r#" pid="7" p1="0""#.to_owned(),
        _ => String::new(),
    };
    let res = format!(
        r##"<basematerials id="1"><base name="PLA" displaycolor="#FFFFFF"/><base name="TPU" displaycolor="#000000"/></basematerials>
           <m:colorgroup id="2"><m:color color="#FF0000"/><m:color color="#00FF00"/></m:colorgroup>
           <m:texture2d id="3" path="/3D/Textures/t.png" contenttype="image/png"/>
           <m:texture2dgroup id="4" texid="3"><m:tex2coord u="0" v="0"/><m:tex2coord u="1" v="0"/><m:tex2coord u="0" v="1"/></m:texture2dgroup>
           <m:compositematerials id="6" matid="1" matindices="1 0"><m:composite values="0.5 0.5"/></m:compositematerials>
           <m:multiproperties id="7" pids="1 2"><m:multi pindices="0 1"/></m:multiproperties>
           <object id="9" pid="2" pindex="0">{}</object>"##,
        cube_mesh(10.0, [0.0; 3], &tri)
    );
    let m = model(
        &format!(r#"xmlns:m="{NS_M}""#),
        &res,
        r#"<item objectid="9"/>"#,
    );
    let rels = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Target="/3D/Textures/t.png" Id="t1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dtexture"/></Relationships>"#;
    let png = b"\x89PNG\r\n\x1a\n";
    let b = package(
        &m,
        &[
            ("3D/_rels/3dmodel.model.rels", rels.as_bytes()),
            ("3D/Textures/t.png", png),
        ],
    );
    let r = read(&b, &strict()).expect("strict");
    let o = &r.scene.objects[0];
    let mats = &r.scene.materials;
    let name = |s: u16| mats[usize::from(s)].name.clone();
    assert_eq!(name(o.material.unwrap()), "Color #FF0000FF");
    let f = o.face_materials.as_ref().unwrap();
    assert_eq!(name(f[0]), "Color #00FF00FF");
    assert_eq!(f[1], NO_SLOT, "a textured face has no slot");
    assert_eq!(name(f[2]), "TPU", "composite → its first base material");
    assert_eq!(name(f[3]), "PLA", "multi → its first base/colour layer");
    let uv = o.mesh.attrs.corner_uvs.as_ref().expect("uvs");
    assert_eq!(uv[1][1].x, 1.0);
    assert_eq!(uv[1][2].y, 1.0);
    let ignored: Vec<_> = r
        .report
        .warnings
        .iter()
        .filter_map(|w| match w {
            IoWarning::UnsupportedContentIgnored { what, .. } => Some(what.clone()),
            _ => None,
        })
        .collect();
    assert!(
        ignored.iter().any(|w| w.starts_with("texture images")),
        "{ignored:?}"
    );
    assert!(
        ignored.iter().any(|w| w.starts_with("composite")),
        "{ignored:?}"
    );
    assert!(
        ignored.iter().any(|w| w.starts_with("multiproperties")),
        "{ignored:?}"
    );
    // The texture relationship is required in strict mode.
    let b = package(&m, &[("3D/Textures/t.png", png)]);
    assert!(read(&b, &strict()).is_err());
    assert!(read(&b, &opts()).is_ok());
}

/// An Orca/Bambu-style project: root object with components in `/3D/Objects/*.model`
/// (Production `p:path`), `paint_color`, `model_settings.config` names, extruders and plates.
fn orca_like() -> Vec<u8> {
    let paint = |k: usize| match k {
        0 | 1 => r#" paint_color="8""#.to_owned(),
        2 => r#" paint_color="0C""#.to_owned(),
        // Split triangle: dominant state 2 (half), a quarter of state 1, a quarter unpainted.
        3 => {
            let nibbles = [0b0010u8, 2 << 2, 1 << 2, 0];
            let s: String = nibbles.iter().rev().map(|n| format!("{n:X}")).collect();
            format!(r#" paint_color="{s}""#)
        }
        _ => String::new(),
    };
    let part = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="{NS}" xmlns:p="{NS_P}" requiredextensions="p">
<resources>
 <object id="1" p:UUID="00000001-61cb-4c03-9d28-80fed5dfa1dc" type="model">{}</object>
 <object id="2" p:UUID="00000002-61cb-4c03-9d28-80fed5dfa1dc" type="model">{}</object>
</resources><build/></model>"#,
        cube_mesh(10.0, [0.0; 3], &paint),
        cube_mesh(4.0, [20.0, 0.0, 0.0], &no_attrs)
    );
    let root = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="{NS}" xmlns:p="{NS_P}" xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" requiredextensions="p">
<metadata name="Application">BambuStudio-02.08.02.61</metadata>
<metadata name="BambuStudio:3mfVersion">1</metadata>
<resources>
 <object id="3" p:UUID="00000003-61cb-4c03-9d28-80fed5dfa1dc" type="model"><components>
   <component p:path="/3D/Objects/object_1.model" objectid="1" p:UUID="00010000-b206-40ff-9872-83e8017abed1" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>
   <component p:path="/3D/Objects/object_1.model" objectid="2" p:UUID="00010001-b206-40ff-9872-83e8017abed1" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>
 </components></object>
</resources>
<build p:UUID="2c7c17d8-22b5-4d84-8835-1976022ea369">
 <item objectid="3" p:UUID="00000004-b1ec-4553-aec9-835e5b724bb4" transform="1 0 0 0 1 0 0 0 1 100 100 0" printable="1"/>
 <item objectid="3" p:UUID="00000005-b1ec-4553-aec9-835e5b724bb4" transform="1 0 0 0 1 0 0 0 1 300 100 0" printable="1"/>
</build></model>"#
    );
    let rels = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/3D/Objects/object_1.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>"#;
    let settings = r#"<?xml version="1.0" encoding="UTF-8"?>
<config>
  <object id="3">
    <metadata key="name" value="Bracket"/>
    <metadata key="extruder" value="1"/>
    <part id="1" subtype="normal_part"><metadata key="name" value="Body"/></part>
    <part id="2" subtype="modifier_part"><metadata key="name" value="Insert"/><metadata key="extruder" value="3"/></part>
  </object>
  <plate><metadata key="plater_id" value="1"/><metadata key="plater_name" value="A"/>
    <model_instance><metadata key="object_id" value="3"/><metadata key="instance_id" value="0"/></model_instance></plate>
  <plate><metadata key="plater_id" value="2"/>
    <model_instance><metadata key="object_id" value="3"/><metadata key="instance_id" value="1"/></model_instance></plate>
</config>"#;
    let project = br##"{"filament_colour": ["#FFFFFF", "#FF0000", "#00FF00"]}"##;
    zip(
        &[
            ("[Content_Types].xml", CONTENT_TYPES.as_bytes()),
            ("_rels/.rels", ROOT_RELS.as_bytes()),
            ("3D/3dmodel.model", root.as_bytes()),
            ("3D/_rels/3dmodel.model.rels", rels.as_bytes()),
            ("3D/Objects/object_1.model", part.as_bytes()),
            ("Metadata/model_settings.config", settings.as_bytes()),
            ("Metadata/project_settings.config", project),
        ],
        false,
    )
}

#[test]
fn orca_project_multi_part_paint_names_extruders_plates() {
    let r = ok(&orca_like());
    let s = &r.scene;
    assert_eq!(s.roots.len(), 2);
    assert_eq!(s.roots[0].name.as_deref(), Some("Bracket"));
    assert_eq!(s.objects.len(), 2, "two parts shared by both instances");
    let body = &s.objects[0];
    let insert = &s.objects[1];
    assert_eq!(body.name.as_deref(), Some("Body"));
    assert_eq!(body.filament, Some(1));
    assert_eq!(insert.name.as_deref(), Some("Insert"));
    assert_eq!(insert.filament, Some(3));
    assert_eq!(insert.part_subtype, PartSubtype::Modifier);
    assert_eq!(s.instances()[2].1.translation.x, 300.0);
    // Paint: faces 0, 1 → filament 2, face 2 → 3, face 3 split with dominant 2.
    let f = body.face_materials.as_ref().expect("painted");
    let fil = |k: usize| s.materials[usize::from(f[k])].filament;
    assert_eq!(fil(0), Some(2));
    assert_eq!(fil(1), Some(2));
    assert_eq!(fil(2), Some(3));
    assert_eq!(fil(3), Some(2));
    assert_eq!(f[4], NO_SLOT);
    assert_eq!(
        s.materials[usize::from(f[0])].color.0,
        [255, 0, 0, 255],
        "project colour"
    );
    assert!(
        r.report
            .warnings
            .contains(&IoWarning::PaintApproximated { triangles: 1 })
    );
    // Plates.
    assert_eq!(s.plates.len(), 2);
    assert_eq!(s.plates[0].name.as_deref(), Some("A"));
    assert_eq!(s.plates[0].roots, vec![0]);
    assert_eq!(s.plates[1].roots, vec![1]);
    assert_eq!(
        s.metadata.get("Application").map(String::as_str),
        Some("BambuStudio-02.08.02.61")
    );
    // Without slicer extras: no paint, no config names.
    let mut o = opts();
    o.threemf.read_slicer_extras = false;
    let r = read(&orca_like(), &o).expect("plain");
    assert!(r.scene.objects[0].face_materials.is_none());
    assert_eq!(r.scene.objects[0].name, None);
    // paint_color is not valid 3MF: strict mode refuses it.
    assert!(read(&orca_like(), &strict()).is_err());
}

#[test]
fn prusa_project_mmu_segmentation_and_volumes() {
    let mmu = |k: usize| match k {
        0 => r#" slic3rpe:mmu_segmentation="4""#.to_owned(),
        13 => r#" slic3rpe:mmu_segmentation="8""#.to_owned(),
        _ => String::new(),
    };
    // One object whose mesh holds two volumes (two cubes, 12 triangles each).
    let a = cube_mesh(10.0, [0.0; 3], &mmu);
    let b = cube_mesh(5.0, [20.0, 0.0, 0.0], &|k| mmu(k + 12));
    let verts =
        |m: &str| m[m.find("<vertices>").unwrap() + 10..m.find("</vertices>").unwrap()].to_owned();
    let tris = |m: &str| {
        m[m.find("<triangles>").unwrap() + 11..m.find("</triangles>").unwrap()].to_owned()
    };
    let shift = |t: String| {
        let mut out = t;
        for i in (0..8).rev() {
            out = out
                .replace(&format!("v1=\"{i}\""), &format!("v1=\"{}\"", i + 8))
                .replace(&format!("v2=\"{i}\""), &format!("v2=\"{}\"", i + 8))
                .replace(&format!("v3=\"{i}\""), &format!("v3=\"{}\"", i + 8));
        }
        out
    };
    let mesh = format!(
        "<mesh><vertices>{}{}</vertices><triangles>{}{}</triangles></mesh>",
        verts(&a),
        verts(&b),
        tris(&a),
        shift(tris(&b))
    );
    let m = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="{NS}" xmlns:slic3rpe="http://schemas.slic3r.org/3mf/2017/06">
<metadata name="slic3rpe:Version3mf">1</metadata>
<resources><object id="1" type="model">{mesh}</object></resources>
<build><item objectid="1" transform="1 0 0 0 1 0 0 0 1 50 50 0"/></build></model>"#
    );
    let cfg = r#"<?xml version="1.0" encoding="UTF-8"?>
<config><object id="1" instances_count="1">
 <metadata type="object" key="name" value="pair"/>
 <metadata type="object" key="extruder" value="1"/>
 <volume firstid="0" lastid="11"><metadata type="volume" key="name" value="big"/><metadata type="volume" key="extruder" value="0"/></volume>
 <volume firstid="12" lastid="23"><metadata type="volume" key="name" value="small"/><metadata type="volume" key="volume_type" value="NegativeVolume"/><metadata type="volume" key="extruder" value="2"/></volume>
</object></config>"#;
    let pe = b"; extruder_colour = #112233;#445566\n";
    let bytes = package(
        &m,
        &[
            ("Metadata/Slic3r_PE_model.config", cfg.as_bytes()),
            ("Metadata/Slic3r_PE.config", pe),
        ],
    );
    let r = ok(&bytes);
    let s = &r.scene;
    assert_eq!(s.roots.len(), 1);
    assert_eq!(s.roots[0].name.as_deref(), Some("pair"));
    let NodeContent::Group(c) = &s.roots[0].content else {
        panic!("group")
    };
    assert_eq!(c.len(), 2);
    assert_eq!(s.objects.len(), 2);
    assert_eq!(s.objects[0].name.as_deref(), Some("big"));
    assert_eq!(s.objects[0].mesh.face_count(), 12);
    assert_eq!(s.objects[0].mesh.vertex_count(), 8);
    assert_eq!(s.objects[0].filament, Some(1));
    assert_eq!(s.objects[1].filament, Some(2));
    assert_eq!(s.objects[1].part_subtype, PartSubtype::NegativePart);
    let f0 = s.objects[0].face_materials.as_ref().unwrap();
    assert_eq!(s.materials[usize::from(f0[0])].filament, Some(1));
    assert_eq!(
        s.materials[usize::from(f0[0])].color.0,
        [0x11, 0x22, 0x33, 255]
    );
    let f1 = s.objects[1].face_materials.as_ref().unwrap();
    assert_eq!(s.materials[usize::from(f1[1])].filament, Some(2));
}

#[test]
fn paint_decoder_is_public() {
    assert_eq!(paint::decode_dominant("4"), Ok((1, false)));
    assert_eq!(paint::encode_state(3), "0C");
}

fn err(bytes: &[u8], o: &ImportOptions) -> IoError {
    match read(bytes, o) {
        Err(e) => e,
        Ok(_) => panic!("expected an error"),
    }
}

#[test]
fn hostile_and_degenerate_inputs() {
    // Empty and not a zip.
    assert!(matches!(err(b"", &opts()), IoError::Empty));
    assert!(matches!(
        err(b"PK\x03\x04garbage", &opts()),
        IoError::Malformed { .. }
    ));
    // Missing StartPart.
    let no_start = zip(
        &[("[Content_Types].xml", CONTENT_TYPES.as_bytes()), ("_rels/.rels", b"<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\"/>"), ("3D/3dmodel.model", b"<model/>")],
        false,
    );
    assert!(matches!(err(&no_start, &opts()), IoError::Malformed { .. }));
    // Out-of-range vertex index → Malformed with an XML location.
    let m = model(
        "",
        &format!(
            r#"<object id="1">{}</object>"#,
            cube_mesh(1.0, [0.0; 3], &no_attrs)
        ),
        r#"<item objectid="1"/>"#,
    )
    .replace(r#"v1="3" v2="4" v3="7""#, r#"v1="3" v2="4" v3="99""#);
    match err(&package(&m, &[]), &opts()) {
        IoError::Malformed {
            location: Location::Xml { part, line, .. },
            message,
            ..
        } => {
            assert_eq!(part, "3D/3dmodel.model");
            assert_eq!(line, 3, "the object's line");
            assert!(message.contains("beyond"), "{message}");
        }
        e => panic!("{e:?}"),
    }
    // Missing referenced object / part.
    let m = model("", "", r#"<item objectid="7"/>"#);
    assert!(matches!(
        err(&package(&m, &[]), &opts()),
        IoError::Malformed { .. }
    ));
    // Unsupported required extension.
    let m = model(
        r#"xmlns:b="http://schemas.microsoft.com/3dmanufacturing/beamlattice/2017/02" requiredextensions="b""#,
        "",
        "",
    );
    assert!(
        matches!(err(&package(&m, &[]), &opts()), IoError::UnsupportedRequiredExtension { namespace } if namespace.contains("beamlattice"))
    );
    // A DTD.
    let m = one_cube_model().replacen("<model", "<!DOCTYPE model [<!ENTITY a \"b\">]><model", 1);
    assert!(matches!(
        err(&package(&m, &[]), &opts()),
        IoError::Malformed { .. }
    ));
    // No triangles at all.
    let m = model(
        "",
        r#"<object id="1"><mesh><vertices/><triangles/></mesh></object>"#,
        r#"<item objectid="1"/>"#,
    );
    assert!(matches!(err(&package(&m, &[]), &opts()), IoError::Empty));
}

fn one_cube_model() -> String {
    model(
        "",
        &format!(
            r#"<object id="1">{}</object>"#,
            cube_mesh(10.0, [0.0; 3], &no_attrs)
        ),
        r#"<item objectid="1"/>"#,
    )
}

/// A component cycle across two parts (only expressible with p:path or forward references).
pub fn cycle() -> Vec<u8> {
    let m = model(
        "",
        r#"<object id="1"><components><component objectid="2"/></components></object>
           <object id="2"><components><component objectid="1"/></components></object>"#,
        r#"<item objectid="1"/>"#,
    );
    package(&m, &[])
}

#[test]
fn component_cycles_and_depth() {
    match err(&cycle(), &opts()) {
        IoError::Malformed { message, .. } => assert!(message.contains("cycle"), "{message}"),
        e => panic!("{e:?}"),
    }
    // 40 nested components: deeper than max_component_depth (32).
    let mut res = format!(
        r#"<object id="1">{}</object>"#,
        cube_mesh(1.0, [0.0; 3], &no_attrs)
    );
    for i in 2..=41 {
        res.push_str(&format!(
            r#"<object id="{i}"><components><component objectid="{}"/></components></object>"#,
            i - 1
        ));
    }
    let m = model("", &res, r#"<item objectid="41"/>"#);
    assert!(matches!(
        err(&package(&m, &[]), &opts()),
        IoError::LimitExceeded {
            what: "component depth",
            ..
        }
    ));
    // An exponential DAG is bounded by the node limit, not by memory.
    let mut res = format!(
        r#"<object id="1">{}</object>"#,
        cube_mesh(1.0, [0.0; 3], &no_attrs)
    );
    for i in 2..=30 {
        res.push_str(&format!(r#"<object id="{i}"><components><component objectid="{0}"/><component objectid="{0}"/></components></object>"#, i - 1));
    }
    let m = model("", &res, r#"<item objectid="30"/>"#);
    assert!(matches!(
        err(&package(&m, &[]), &opts()),
        IoError::LimitExceeded {
            what: "scene nodes",
            ..
        }
    ));
}

#[test]
fn non_manifold_model_warns_in_normal_mode_and_fails_strict() {
    // Drop two triangles: an open box.
    let m = one_cube_model().replace(
        r#"<triangle v1="3" v2="0" v3="4"/><triangle v1="3" v2="4" v3="7"/>"#,
        "",
    );
    let b = package(&m, &[]);
    let r = ok(&b);
    assert!(
        r.report.warnings.iter().any(|w| matches!(
            w,
            IoWarning::NotManifold {
                boundary_edges: 4,
                non_manifold_edges: 0,
                ..
            }
        )),
        "{:?}",
        r.report.warnings
    );
    assert!(matches!(err(&b, &strict()), IoError::Malformed { .. }));
    // Inward-facing (all triangles flipped) fails strict.
    let flipped = one_cube_model()
        .replace("v1=", "vx=")
        .replace("v2=", "v1=")
        .replace("vx=", "v2=");
    assert!(matches!(
        err(&package(&flipped, &[]), &strict()),
        IoError::Malformed { .. }
    ));
    // A support may be open.
    let s = m.replace(r#"<object id="1">"#, r#"<object id="1" type="support">"#);
    assert!(read(&package(&s, &[]), &strict()).is_ok());
}

#[test]
fn package_details() {
    // Case-insensitive part names and relationship targets.
    let rels = ROOT_RELS.replace("/3D/3dmodel.model", "/3d/3DMODEL.MODEL");
    let b = zip(
        &[
            ("[Content_Types].xml", CONTENT_TYPES.as_bytes()),
            ("_rels/.rels", rels.as_bytes()),
            ("3D/3dmodel.model", one_cube_model().as_bytes()),
        ],
        false,
    );
    assert_eq!(ok(&b).scene.objects.len(), 1);
    // Stored entries.
    let b = zip(
        &[
            ("[Content_Types].xml", CONTENT_TYPES.as_bytes()),
            ("_rels/.rels", ROOT_RELS.as_bytes()),
            ("3D/3dmodel.model", one_cube_model().as_bytes()),
        ],
        true,
    );
    assert_eq!(ok(&b).scene.objects.len(), 1);
    // Streaming entries with data descriptors.
    let mut w = zip::ZipWriter::new_stream(Vec::new());
    let o = zip::write::SimpleFileOptions::default();
    for (n, d) in [
        ("[Content_Types].xml", CONTENT_TYPES.to_owned()),
        ("_rels/.rels", ROOT_RELS.to_owned()),
        ("3D/3dmodel.model", one_cube_model()),
    ] {
        w.start_file(n, o).unwrap();
        w.write_all(d.as_bytes()).unwrap();
    }
    let b = w.finish().unwrap().into_inner();
    assert_eq!(ok(&b).scene.objects.len(), 1);
    // ZIP64 extra fields.
    let mut w = zip::ZipWriter::new(Cursor::new(Vec::new()));
    let o = zip::write::SimpleFileOptions::default().large_file(true);
    for (n, d) in [
        ("[Content_Types].xml", CONTENT_TYPES.to_owned()),
        ("_rels/.rels", ROOT_RELS.to_owned()),
        ("3D/3dmodel.model", one_cube_model()),
    ] {
        w.start_file(n, o).unwrap();
        w.write_all(d.as_bytes()).unwrap();
    }
    let b = w.finish().unwrap().into_inner();
    assert_eq!(ok(&b).scene.objects.len(), 1);
    // A PNG package thumbnail.
    let rels = ROOT_RELS.replace(
        "</Relationships>",
        r#"<Relationship Target="/Metadata/thumbnail.png" Id="rel1" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail"/></Relationships>"#,
    );
    let png: &[u8] = b"\x89PNG\r\n\x1a\nrest";
    let b = zip(
        &[
            ("[Content_Types].xml", CONTENT_TYPES.as_bytes()),
            ("_rels/.rels", rels.as_bytes()),
            ("3D/3dmodel.model", one_cube_model().as_bytes()),
            ("Metadata/thumbnail.png", png),
        ],
        false,
    );
    let r = read(&b, &strict()).expect("strict");
    assert!(matches!(r.scene.thumbnail, Some(Thumbnail::Png(ref p)) if &p[..] == png));
}

#[test]
fn cancellation() {
    let b = one_cube();
    let token = amg_jobs::CancelToken::new();
    token.cancel();
    let ctx = JobCtx::with_token(token);
    assert!(matches!(
        import(Source::Bytes(&b), Some(Format::ThreeMf), &opts(), &ctx),
        Err(IoError::Cancelled)
    ));
}

#[test]
fn deterministic_for_any_thread_count() {
    let big = {
        let mut res = String::new();
        for i in 1..=20 {
            res.push_str(&format!(
                r#"<object id="{i}">{}</object>"#,
                cube_mesh(
                    1.0 + f64::from(i),
                    [f64::from(i) * 30.0, 0.0, 0.0],
                    &no_attrs
                )
            ));
        }
        let items: String = (1..=20)
            .map(|i| format!(r#"<item objectid="{i}"/>"#))
            .collect();
        package(&model("", &res, &items), &[])
    };
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        pool.install(|| {
            let r = amg_io::threemf::read_with(
                Source::Bytes(&big),
                &opts(),
                &JobCtx::detached(),
                &amg_io::threemf::Tuning {
                    chunk_bytes: 64,
                    reference_only: false,
                },
            )
            .expect("read");
            r.scene
                .objects
                .iter()
                .map(|o| format!("{:?}{:?}", o.mesh.positions, o.mesh.tris))
                .collect::<Vec<_>>()
        })
    };
    assert_eq!(run(1), run(16));
}
