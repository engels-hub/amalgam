//! Interned encoding of element maps for the `.amg` geometry cache (note §8.1): a table of
//! unique segments in first-use order and names as index lists into it, so shared sub-names
//! (the face names inside every `Bound`) are written once. Decoding shares the `Arc` of every
//! repeated name again.

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;

use serde::{Deserialize, Serialize};

use super::{BodyNaming, NameSeg, NameSource, Role, TopoName};
use crate::id::{ConstructionId, FeatureId, SketchId};
use crate::stub::SketchEntityKey;

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
enum WireSource {
    SketchCurve {
        sketch: SketchId,
        curve: SketchEntityKey,
    },
    SketchPoint {
        sketch: SketchId,
        point: SketchEntityKey,
    },
    Profile {
        sketch: SketchId,
        outer: Vec<SketchEntityKey>,
    },
    Selection {
        key: u32,
    },
    Construction(ConstructionId),
    Topo(u32),
    Digest(u64),
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
enum WireSeg {
    Gen {
        feature: FeatureId,
        role: Role,
        from: Option<WireSource>,
    },
    Bound {
        faces: Vec<u32>,
        index: u32,
        count: u32,
    },
    Split {
        feature: FeatureId,
        index: u32,
        count: u32,
    },
    Merge {
        feature: FeatureId,
        with: Vec<u32>,
    },
    Inst {
        feature: FeatureId,
        instance: u32,
    },
}

/// The interned form of a [`BodyNaming`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub(crate) struct WireNaming {
    segs: Vec<WireSeg>,
    /// Each name as segment indices; a name only refers to earlier names.
    names: Vec<Vec<u32>>,
    faces: Vec<u32>,
    edges: Vec<u32>,
    vertices: Vec<u32>,
    #[serde(default)]
    groups: Vec<(u32, u32)>,
    #[serde(default)]
    removed: Vec<(u32, FeatureId)>,
}

#[derive(Default)]
struct Interner {
    segs: Vec<WireSeg>,
    seg_ids: HashMap<WireSeg, u32>,
    names: Vec<Vec<u32>>,
    name_ids: HashMap<TopoName, u32>,
}

fn id(n: usize) -> u32 {
    u32::try_from(n).unwrap_or(u32::MAX)
}

impl Interner {
    fn name(&mut self, n: &TopoName) -> u32 {
        if let Some(&i) = self.name_ids.get(n) {
            return i;
        }
        let segs: Vec<u32> = n.segments().iter().map(|s| self.seg(s)).collect();
        let i = id(self.names.len());
        self.names.push(segs);
        self.name_ids.insert(n.clone(), i);
        i
    }

    fn seg(&mut self, s: &NameSeg) -> u32 {
        let w = match s {
            NameSeg::Gen {
                feature,
                role,
                from,
            } => WireSeg::Gen {
                feature: *feature,
                role: role.clone(),
                from: from.as_ref().map(|f| match f {
                    NameSource::SketchCurve { sketch, curve } => WireSource::SketchCurve {
                        sketch: *sketch,
                        curve: *curve,
                    },
                    NameSource::SketchPoint { sketch, point } => WireSource::SketchPoint {
                        sketch: *sketch,
                        point: *point,
                    },
                    NameSource::Profile { sketch, outer } => WireSource::Profile {
                        sketch: *sketch,
                        outer: outer.clone(),
                    },
                    NameSource::Selection { key } => WireSource::Selection { key: *key },
                    NameSource::Construction(c) => WireSource::Construction(*c),
                    NameSource::Topo(t) => WireSource::Topo(self.name(t)),
                    NameSource::Digest(d) => WireSource::Digest(*d),
                }),
            },
            NameSeg::Bound {
                faces,
                index,
                count,
            } => WireSeg::Bound {
                faces: faces.iter().map(|f| self.name(f)).collect(),
                index: *index,
                count: *count,
            },
            NameSeg::Split {
                feature,
                index,
                count,
            } => WireSeg::Split {
                feature: *feature,
                index: *index,
                count: *count,
            },
            NameSeg::Merge { feature, with } => WireSeg::Merge {
                feature: *feature,
                with: with.iter().map(|w| self.name(w)).collect(),
            },
            NameSeg::Inst { feature, instance } => WireSeg::Inst {
                feature: *feature,
                instance: *instance,
            },
        };
        if let Some(&i) = self.seg_ids.get(&w) {
            return i;
        }
        let i = id(self.segs.len());
        self.segs.push(w.clone());
        self.seg_ids.insert(w, i);
        i
    }
}

impl WireNaming {
    pub(crate) fn encode(n: &BodyNaming) -> WireNaming {
        let mut it = Interner::default();
        let faces = n.faces.iter().map(|f| it.name(f)).collect();
        let edges = n.edges.iter().map(|f| it.name(f)).collect();
        let vertices = n.vertices.iter().map(|f| it.name(f)).collect();
        let groups = n.groups.iter().map(|(g, f)| (*g, it.name(f))).collect();
        let removed = n.removed.iter().map(|(f, by)| (it.name(f), *by)).collect();
        WireNaming {
            segs: it.segs,
            names: it.names,
            faces,
            edges,
            vertices,
            groups,
            removed,
        }
    }

    pub(crate) fn decode(&self) -> Result<BodyNaming, String> {
        let mut names: Vec<TopoName> = Vec::with_capacity(self.names.len());
        let get = |names: &[TopoName], i: u32| -> Result<TopoName, String> {
            names
                .get(i as usize)
                .cloned()
                .ok_or_else(|| format!("name {i} is used before it is defined"))
        };
        for segs in &self.names {
            let mut out = Vec::with_capacity(segs.len());
            for &s in segs {
                let w = self
                    .segs
                    .get(s as usize)
                    .ok_or_else(|| format!("segment {s} out of range"))?;
                out.push(match w {
                    WireSeg::Gen {
                        feature,
                        role,
                        from,
                    } => NameSeg::Gen {
                        feature: *feature,
                        role: role.clone(),
                        from: match from {
                            None => None,
                            Some(f) => Some(match f {
                                WireSource::SketchCurve { sketch, curve } => {
                                    NameSource::SketchCurve {
                                        sketch: *sketch,
                                        curve: *curve,
                                    }
                                }
                                WireSource::SketchPoint { sketch, point } => {
                                    NameSource::SketchPoint {
                                        sketch: *sketch,
                                        point: *point,
                                    }
                                }
                                WireSource::Profile { sketch, outer } => NameSource::Profile {
                                    sketch: *sketch,
                                    outer: outer.clone(),
                                },
                                WireSource::Selection { key } => {
                                    NameSource::Selection { key: *key }
                                }
                                WireSource::Construction(c) => NameSource::Construction(*c),
                                WireSource::Topo(t) => NameSource::Topo(get(&names, *t)?),
                                WireSource::Digest(d) => NameSource::Digest(*d),
                            }),
                        },
                    },
                    WireSeg::Bound {
                        faces,
                        index,
                        count,
                    } => NameSeg::Bound {
                        faces: faces
                            .iter()
                            .map(|f| get(&names, *f))
                            .collect::<Result<_, _>>()?,
                        index: *index,
                        count: *count,
                    },
                    WireSeg::Split {
                        feature,
                        index,
                        count,
                    } => NameSeg::Split {
                        feature: *feature,
                        index: *index,
                        count: *count,
                    },
                    WireSeg::Merge { feature, with } => NameSeg::Merge {
                        feature: *feature,
                        with: with
                            .iter()
                            .map(|f| get(&names, *f))
                            .collect::<Result<_, _>>()?,
                    },
                    WireSeg::Inst { feature, instance } => NameSeg::Inst {
                        feature: *feature,
                        instance: *instance,
                    },
                });
            }
            names.push(TopoName::new(out));
        }
        let list = |ids: &[u32]| -> Result<Vec<TopoName>, String> {
            ids.iter().map(|i| get(&names, *i)).collect()
        };
        let mut n = BodyNaming::new(
            list(&self.faces)?,
            list(&self.edges)?,
            list(&self.vertices)?,
        );
        n.groups = self
            .groups
            .iter()
            .map(|(g, i)| Ok((*g, get(&names, *i)?)))
            .collect::<Result<BTreeMap<_, _>, String>>()?;
        n.removed = Arc::new(
            self.removed
                .iter()
                .map(|(i, by)| Ok((get(&names, *i)?, *by)))
                .collect::<Result<BTreeMap<_, _>, String>>()?,
        );
        Ok(n)
    }
}
