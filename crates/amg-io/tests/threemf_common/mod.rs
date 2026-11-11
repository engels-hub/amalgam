//! Synthetic 3MF packages for the IO-03 tests (CC0, generated, nothing stored).
#![allow(
    dead_code,
    clippy::expect_used,
    clippy::unwrap_used,
    reason = "test support"
)]

use std::io::{Cursor, Write};

use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, ZipWriter};

pub const NS: &str = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02";
pub const NS_M: &str = "http://schemas.microsoft.com/3dmanufacturing/material/2015/02";
pub const NS_P: &str = "http://schemas.microsoft.com/3dmanufacturing/production/2015/06";

pub const CONTENT_TYPES: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
  <Default Extension="png" ContentType="image/png"/>
  <Default Extension="config" ContentType="text/xml"/>
</Types>"#;

pub const ROOT_RELS: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>"#;

/// A zip with the given entries (Deflate unless `stored`).
pub fn zip(entries: &[(&str, &[u8])], stored: bool) -> Vec<u8> {
    let mut w = ZipWriter::new(Cursor::new(Vec::new()));
    let o = SimpleFileOptions::default().compression_method(if stored {
        CompressionMethod::Stored
    } else {
        CompressionMethod::Deflated
    });
    for (name, data) in entries {
        w.start_file(*name, o).unwrap();
        w.write_all(data).unwrap();
    }
    w.finish().unwrap().into_inner()
}

/// A package with the standard content types and root rels plus `model` and `extra` parts.
pub fn package(model: &str, extra: &[(&str, &[u8])]) -> Vec<u8> {
    let mut e: Vec<(&str, &[u8])> = vec![
        ("[Content_Types].xml", CONTENT_TYPES.as_bytes()),
        ("_rels/.rels", ROOT_RELS.as_bytes()),
        ("3D/3dmodel.model", model.as_bytes()),
    ];
    e.extend_from_slice(extra);
    zip(&e, false)
}

/// A closed, outward-facing cube mesh (8 vertices, 12 triangles) of side `s` at `o`.
pub fn cube_mesh(s: f64, o: [f64; 3], tri_attrs: &dyn Fn(usize) -> String) -> String {
    let mut v = String::from("<vertices>");
    for z in [0.0, s] {
        for (x, y) in [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s)] {
            v.push_str(&format!(
                r#"<vertex x="{}" y="{}" z="{}"/>"#,
                o[0] + x,
                o[1] + y,
                o[2] + z
            ));
        }
    }
    v.push_str("</vertices><triangles>");
    let t: [[u32; 3]; 12] = [
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
    for (k, t) in t.iter().enumerate() {
        v.push_str(&format!(
            r#"<triangle v1="{}" v2="{}" v3="{}"{}/>"#,
            t[0],
            t[1],
            t[2],
            tri_attrs(k)
        ));
    }
    v.push_str("</triangles>");
    format!("<mesh>{v}</mesh>")
}

pub fn no_attrs(_: usize) -> String {
    String::new()
}

/// A root model with `resources` and `build` content.
pub fn model(model_attrs: &str, resources: &str, build: &str) -> String {
    format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="{NS}" {model_attrs}>
<resources>{resources}</resources>
<build>{build}</build>
</model>"#
    )
}

/// One cube object and one build item.
pub fn one_cube() -> Vec<u8> {
    let m = model(
        "",
        &format!(
            r#"<object id="1" type="model" name="cube">{}</object>"#,
            cube_mesh(10.0, [0.0; 3], &no_attrs)
        ),
        r#"<item objectid="1"/>"#,
    );
    package(&m, &[])
}
