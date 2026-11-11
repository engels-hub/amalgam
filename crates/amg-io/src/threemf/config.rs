//! Slicer project parts read for names, extruders, part roles and plates (note §3.1, §3.2):
//! Orca/Bambu `Metadata/model_settings.config` (+ `project_settings.config` for filament
//! colours) and PrusaSlicer `Metadata/Slic3r_PE_model.config` (+ `Slic3r_PE.config`).
//!
//! Interop facts were read from OrcaSlicer `bbs_3mf.cpp` and PrusaSlicer `Model3mf.cpp`; no
//! code was copied.

use std::collections::BTreeMap;

use amg_mesh::Rgba8;

use super::package::walk;
use super::xml::{parse_color, trim};
use crate::error::IoError;
use crate::options::ReadLimits;
use crate::scene::PartSubtype;

/// Orca part settings.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct PartSettings {
    pub name: Option<String>,
    pub extruder: Option<u8>,
    pub subtype: PartSubtype,
}

/// Orca object settings (key: root model object id).
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct ObjectSettings {
    pub name: Option<String>,
    pub extruder: Option<u8>,
    /// Key: the `objectid` of the component (Orca writes it as the part id).
    pub parts: BTreeMap<u32, PartSettings>,
}

/// An Orca plate: its name and the (object id, instance index) pairs placed on it.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct PlateSettings {
    pub name: Option<String>,
    pub instances: Vec<(u32, u32)>,
}

/// `Metadata/model_settings.config`.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct OrcaConfig {
    pub objects: BTreeMap<u32, ObjectSettings>,
    pub plates: Vec<PlateSettings>,
}

/// A PrusaSlicer volume: a triangle range of the object's single mesh.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct PrusaVolume {
    pub first: u32,
    pub last: u32,
    pub name: Option<String>,
    pub extruder: Option<u8>,
    pub subtype: PartSubtype,
}

/// A PrusaSlicer object (key: model object id).
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct PrusaObject {
    pub name: Option<String>,
    pub extruder: Option<u8>,
    pub volumes: Vec<PrusaVolume>,
}

/// `Metadata/Slic3r_PE_model.config`.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct PrusaConfig {
    pub objects: BTreeMap<u32, PrusaObject>,
}

fn attr(e: &quick_xml::events::BytesStart<'_>, name: &str) -> Option<String> {
    e.attributes()
        .with_checks(false)
        .flatten()
        .find(|a| a.key.0 == name)
        .and_then(|a| {
            a.normalized_value(quick_xml::XmlVersion::Implicit1_0)
                .ok()
                .map(|v| v.into_owned())
        })
}

/// An extruder number: 1-based, 0 = "inherit" (`None`).
fn extruder(v: &str) -> Option<u8> {
    trim(v)
        .parse::<u32>()
        .ok()
        .filter(|&n| n > 0)
        .map(|n| n.min(255) as u8)
}

fn orca_subtype(v: &str) -> PartSubtype {
    match trim(v) {
        "negative_part" => PartSubtype::NegativePart,
        "modifier_part" => PartSubtype::Modifier,
        "support_enforcer" => PartSubtype::SupportEnforcer,
        "support_blocker" => PartSubtype::SupportBlocker,
        _ => PartSubtype::NormalPart,
    }
}

fn prusa_subtype(v: &str) -> PartSubtype {
    match trim(v) {
        "NegativeVolume" => PartSubtype::NegativePart,
        "ParameterModifier" => PartSubtype::Modifier,
        "SupportEnforcer" => PartSubtype::SupportEnforcer,
        "SupportBlocker" => PartSubtype::SupportBlocker,
        _ => PartSubtype::NormalPart,
    }
}

/// Parses `Metadata/model_settings.config`.
pub(crate) fn parse_orca(
    data: &[u8],
    part: &str,
    limits: &ReadLimits,
) -> Result<OrcaConfig, IoError> {
    let mut cfg = OrcaConfig::default();
    #[derive(Clone, Copy)]
    enum At {
        None,
        Object(u32),
        Part(u32, u32),
        Plate,
        Instance,
    }
    let mut at = At::None;
    let mut inst: (Option<u32>, Option<u32>) = (None, None);
    let flush_inst = |cfg: &mut OrcaConfig, inst: &mut (Option<u32>, Option<u32>)| {
        if let (Some(o), Some(plate)) = (inst.0, cfg.plates.last_mut()) {
            plate.instances.push((o, inst.1.unwrap_or(0)));
        }
        *inst = (None, None);
    };
    walk(data, part, limits, |depth, _s, e| {
        let local = e.local_name();
        let local = local.as_ref();
        match (depth, local) {
            (2, "object") => {
                flush_inst(&mut cfg, &mut inst);
                let id = attr(e, "id").and_then(|v| trim(&v).parse().ok());
                at = match id {
                    Some(id) => {
                        cfg.objects.entry(id).or_default();
                        At::Object(id)
                    }
                    None => At::None,
                };
            }
            (2, "plate") => {
                flush_inst(&mut cfg, &mut inst);
                cfg.plates.push(PlateSettings::default());
                at = At::Plate;
            }
            (2, _) => {
                flush_inst(&mut cfg, &mut inst);
                at = At::None;
            }
            (3, "part") => {
                if let At::Object(o) | At::Part(o, _) = at {
                    let id = attr(e, "id").and_then(|v| trim(&v).parse().ok());
                    if let Some(id) = id {
                        let p = cfg
                            .objects
                            .entry(o)
                            .or_default()
                            .parts
                            .entry(id)
                            .or_default();
                        if let Some(s) = attr(e, "subtype") {
                            p.subtype = orca_subtype(&s);
                        }
                        at = At::Part(o, id);
                    }
                }
            }
            (3, "model_instance") => {
                flush_inst(&mut cfg, &mut inst);
                at = At::Instance;
            }
            (3, "metadata") => {
                let (k, v) = (
                    attr(e, "key").unwrap_or_default(),
                    attr(e, "value").unwrap_or_default(),
                );
                match at {
                    At::Object(o) | At::Part(o, _) => {
                        // A metadata directly in <object> after a <part> still belongs to the object.
                        let obj = cfg.objects.entry(o).or_default();
                        match k.as_str() {
                            "name" => obj.name = Some(v),
                            "extruder" => obj.extruder = extruder(&v),
                            _ => {}
                        }
                        at = At::Object(o);
                    }
                    At::Plate | At::Instance => {
                        if k == "plater_name"
                            && let Some(p) = cfg.plates.last_mut()
                            && !trim(&v).is_empty()
                        {
                            p.name = Some(v);
                        }
                    }
                    At::None => {}
                }
            }
            (4, "metadata") => {
                let (k, v) = (
                    attr(e, "key").unwrap_or_default(),
                    attr(e, "value").unwrap_or_default(),
                );
                match at {
                    At::Part(o, id) => {
                        let p = cfg
                            .objects
                            .entry(o)
                            .or_default()
                            .parts
                            .entry(id)
                            .or_default();
                        match k.as_str() {
                            "name" => p.name = Some(v),
                            "extruder" => p.extruder = extruder(&v),
                            _ => {}
                        }
                    }
                    At::Instance => match k.as_str() {
                        "object_id" => inst.0 = trim(&v).parse().ok(),
                        "instance_id" => inst.1 = trim(&v).parse().ok(),
                        _ => {}
                    },
                    _ => {}
                }
            }
            _ => {}
        }
        Ok(())
    })?;
    flush_inst(&mut cfg, &mut inst);
    Ok(cfg)
}

/// Parses `Metadata/Slic3r_PE_model.config`.
pub(crate) fn parse_prusa(
    data: &[u8],
    part: &str,
    limits: &ReadLimits,
) -> Result<PrusaConfig, IoError> {
    let mut cfg = PrusaConfig::default();
    let mut obj: Option<u32> = None;
    let mut in_volume = false;
    walk(data, part, limits, |depth, _s, e| {
        let local = e.local_name();
        let local = local.as_ref();
        match (depth, local) {
            (2, "object") => {
                obj = attr(e, "id").and_then(|v| trim(&v).parse().ok());
                if let Some(o) = obj {
                    cfg.objects.entry(o).or_default();
                }
                in_volume = false;
            }
            (3, "metadata") => {
                in_volume = false;
                if let Some(o) = obj {
                    let (k, v) = (
                        attr(e, "key").unwrap_or_default(),
                        attr(e, "value").unwrap_or_default(),
                    );
                    let ob = cfg.objects.entry(o).or_default();
                    match k.as_str() {
                        "name" => ob.name = Some(v),
                        "extruder" => ob.extruder = extruder(&v),
                        _ => {}
                    }
                }
            }
            (3, "volume") => {
                in_volume = false;
                if let Some(o) = obj {
                    let first = attr(e, "firstid").and_then(|v| trim(&v).parse().ok());
                    let last = attr(e, "lastid").and_then(|v| trim(&v).parse().ok());
                    if let (Some(first), Some(last)) = (first, last)
                        && first <= last
                    {
                        cfg.objects.entry(o).or_default().volumes.push(PrusaVolume {
                            first,
                            last,
                            ..PrusaVolume::default()
                        });
                        in_volume = true;
                    }
                }
            }
            (4, "metadata") if in_volume => {
                if let Some(o) = obj
                    && let Some(vol) = cfg.objects.entry(o).or_default().volumes.last_mut()
                {
                    let (k, v) = (
                        attr(e, "key").unwrap_or_default(),
                        attr(e, "value").unwrap_or_default(),
                    );
                    match k.as_str() {
                        "name" => vol.name = Some(v),
                        "extruder" => vol.extruder = extruder(&v),
                        "volume_type" => vol.subtype = prusa_subtype(&v),
                        "modifier" if trim(&v) == "1" => vol.subtype = PartSubtype::Modifier,
                        _ => {}
                    }
                }
            }
            _ => {}
        }
        Ok(())
    })?;
    Ok(cfg)
}

/// Filament colours of an Orca/Bambu `project_settings.config` (JSON, `filament_colour`).
pub(crate) fn orca_filament_colors(data: &[u8]) -> Vec<Rgba8> {
    let Ok(v) = serde_json::from_slice::<serde_json::Value>(data) else {
        return Vec::new();
    };
    v.get("filament_colour")
        .and_then(|c| c.as_array())
        .map(|a| {
            a.iter()
                .filter_map(|c| c.as_str().and_then(|s| parse_color(s).ok()))
                .collect()
        })
        .unwrap_or_default()
}

/// Extruder colours of a PrusaSlicer `Slic3r_PE.config` (`; extruder_colour = #..;#..`,
/// falling back to `filament_colour`).
pub(crate) fn prusa_filament_colors(data: &[u8]) -> Vec<Rgba8> {
    let text = String::from_utf8_lossy(data);
    let find = |key: &str| -> Vec<Rgba8> {
        for line in text.lines() {
            let l = line.trim_start_matches([';', ' ']);
            if let Some(rest) = l.strip_prefix(key)
                && let Some(v) = rest.trim_start().strip_prefix('=')
            {
                return v
                    .split(';')
                    .filter_map(|c| parse_color(c.trim().trim_matches('"')).ok())
                    .collect();
            }
        }
        Vec::new()
    };
    let c = find("extruder_colour");
    if c.is_empty() {
        find("filament_colour")
    } else {
        c
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orca_config() {
        let xml = br#"<?xml version="1.0" encoding="UTF-8"?>
<config>
  <object id="2">
    <metadata key="name" value="Cube &amp; co"/>
    <metadata key="extruder" value="2"/>
    <part id="1" subtype="normal_part">
      <metadata key="name" value="body"/>
      <metadata key="extruder" value="3"/>
    </part>
    <part id="2" subtype="modifier_part">
      <metadata key="name" value="mod"/>
    </part>
  </object>
  <plate>
    <metadata key="plater_id" value="1"/>
    <metadata key="plater_name" value="first"/>
    <model_instance>
      <metadata key="object_id" value="2"/>
      <metadata key="instance_id" value="0"/>
    </model_instance>
  </plate>
  <plate>
    <metadata key="plater_id" value="2"/>
    <model_instance>
      <metadata key="object_id" value="2"/>
      <metadata key="instance_id" value="1"/>
    </model_instance>
  </plate>
</config>"#;
        let c = parse_orca(
            xml,
            "Metadata/model_settings.config",
            &ReadLimits::default(),
        )
        .expect("parse");
        let o = &c.objects[&2];
        assert_eq!(o.name.as_deref(), Some("Cube & co"));
        assert_eq!(o.extruder, Some(2));
        assert_eq!(o.parts[&1].name.as_deref(), Some("body"));
        assert_eq!(o.parts[&1].extruder, Some(3));
        assert_eq!(o.parts[&2].subtype, PartSubtype::Modifier);
        assert_eq!(c.plates.len(), 2);
        assert_eq!(c.plates[0].name.as_deref(), Some("first"));
        assert_eq!(c.plates[0].instances, vec![(2, 0)]);
        assert_eq!(c.plates[1].instances, vec![(2, 1)]);
    }

    #[test]
    fn prusa_config() {
        let xml = br#"<?xml version="1.0" encoding="UTF-8"?>
<config>
 <object id="1" instances_count="1">
  <metadata type="object" key="name" value="two"/>
  <metadata type="object" key="extruder" value="1"/>
  <volume firstid="0" lastid="11">
   <metadata type="volume" key="name" value="a"/>
   <metadata type="volume" key="volume_type" value="ModelPart"/>
   <metadata type="volume" key="extruder" value="0"/>
   <mesh edges_fixed="0" degenerate_facets="0"/>
  </volume>
  <volume firstid="12" lastid="23">
   <metadata type="volume" key="name" value="b"/>
   <metadata type="volume" key="volume_type" value="NegativeVolume"/>
   <metadata type="volume" key="extruder" value="2"/>
  </volume>
 </object>
</config>"#;
        let c = parse_prusa(
            xml,
            "Metadata/Slic3r_PE_model.config",
            &ReadLimits::default(),
        )
        .expect("parse");
        let o = &c.objects[&1];
        assert_eq!(o.name.as_deref(), Some("two"));
        assert_eq!(o.volumes.len(), 2);
        assert_eq!(o.volumes[0].extruder, None);
        assert_eq!((o.volumes[1].first, o.volumes[1].last), (12, 23));
        assert_eq!(o.volumes[1].subtype, PartSubtype::NegativePart);
        assert_eq!(o.volumes[1].extruder, Some(2));
    }

    #[test]
    fn filament_colours() {
        let j = br##"{"filament_colour": ["#FF0000", "#00FF00FF", "junk"]}"##;
        assert_eq!(
            orca_filament_colors(j),
            vec![Rgba8([255, 0, 0, 255]), Rgba8([0, 255, 0, 255])]
        );
        let p = b"; bed_shape = 0x0\n; extruder_colour = #0000FF;\"\";#FFFFFF\n";
        assert_eq!(
            prusa_filament_colors(p),
            vec![Rgba8([0, 0, 255, 255]), Rgba8([255, 255, 255, 255])]
        );
    }
}
