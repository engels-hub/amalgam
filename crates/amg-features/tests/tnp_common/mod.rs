//! Shared harness of the DOC-13 golden TNP corpus (`tests/tnp.rs`) and its held-out fuzz set
//! (`tests/tnp_fuzz.rs`): persistent-naming note §11.2 on the S04 solid features.
//!
//! - **Corpus macros** (note §11.2): `$root`, `$name`, `$index_of:`, `$pick`, `$profile`,
//!   `patch`, `accept`, `saveload`, and the DOC-13 macros `sketch`, `sketch_edit`, `EXT`,
//!   `EXTN`, `FIL`, `CHA`.
//! - **Real features for the DOC-06 cases.** The DOC-06-phase cases are written on the lane-E
//!   test features; this harness maps them to the S04 features (README of the corpus):
//!   `test.brep_box` / `test.brep_prism` become a `sketch.sketch` on XY (one line per polygon
//!   edge, keyed by the edge's start-point tag) plus a `solid.extrude` (the prism's `z0` is the
//!   extrude's offset start, `Cut`/`Join` its body operation on `target`), and
//!   `test.brep_fillet` becomes a constant-radius `solid.fillet`. Edits keep the mapping: a
//!   changed point list moves the lines of kept tags, adds lines for new tags and removes the
//!   lines of dropped ones; the extrude's profile is re-picked. Binds name the extrude, so
//!   `{"origin": "$box", "slot": 0}` is the extruded body; timeline indices (`$index_of:`,
//!   `MoveFeature.to`) count the pair as one feature. `test.probe` stays (no S04 feature
//!   references a non-planar face only).

#![allow(dead_code, reason = "shared by the corpus and the fuzz test")]

use std::path::{Path, PathBuf};

use serde_json::{Map, Value};

pub fn corpus_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/tnp")
}

pub fn read_json(p: &Path) -> Value {
    let s = std::fs::read_to_string(p).unwrap_or_else(|e| panic!("{}: {e}", p.display()));
    serde_json::from_str(&s).unwrap_or_else(|e| panic!("{}: {e}", p.display()))
}

pub fn cases() -> Vec<Value> {
    let mut v: Vec<PathBuf> = std::fs::read_dir(corpus_dir().join("cases"))
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .collect();
    v.sort();
    v.iter().map(|p| read_json(p)).collect()
}

/// JSON Merge Patch (RFC 7396).
pub fn merge_patch(target: &mut Value, patch: &Value) {
    match patch {
        Value::Object(p) => {
            if !target.is_object() {
                *target = Value::Object(Map::new());
            }
            let t = target.as_object_mut().unwrap();
            for (k, v) in p {
                if v.is_null() {
                    t.remove(k);
                } else {
                    merge_patch(t.entry(k.clone()).or_insert(Value::Null), v);
                }
            }
        }
        other => *target = other.clone(),
    }
}

/// [`merge_patch`] for feature data: a patch value `{"Inline": "…"}` (an S01 `Expr`) replaces
/// the field, whose stored form after `AddFeature` is `{"Param": id}`, instead of merging into it.
pub fn merge_patch_exprs(target: &mut Value, patch: &Value) {
    match patch {
        Value::Object(p) if p.len() == 1 && p.contains_key("Inline") => *target = patch.clone(),
        Value::Object(p) => {
            if !target.is_object() {
                *target = Value::Object(Map::new());
            }
            let t = target.as_object_mut().unwrap();
            for (k, v) in p {
                if v.is_null() {
                    t.remove(k);
                } else {
                    merge_patch_exprs(t.entry(k.clone()).or_insert(Value::Null), v);
                }
            }
        }
        other => *target = other.clone(),
    }
}

/// The steps of a case: base, script (inline or shared) in order.
pub fn base_steps(case: &Value) -> Vec<Value> {
    let mut steps: Vec<Value> = Vec::new();
    if let Some(b) = case["base"].as_str() {
        steps.extend(
            read_json(&corpus_dir().join(format!("bases/{b}.json")))
                .as_array()
                .unwrap()
                .clone(),
        );
    }
    match &case["script"] {
        Value::String(s) => steps.extend(
            read_json(&corpus_dir().join(format!("bases/{s}.json")))
                .as_array()
                .unwrap()
                .clone(),
        ),
        Value::Array(a) => steps.extend(a.clone()),
        _ => {}
    }
    steps
}

#[cfg(feature = "occt")]
pub use run::*;

#[cfg(feature = "occt")]
mod run {
    use std::collections::{HashMap, HashSet};
    use std::sync::Arc;

    use amg_core::{Plane, Point2, Point3};
    use amg_doc::io::CacheExport;
    use amg_doc::naming::{GeomSource, KernelGeom, NamingView, ProfileFp, default_kernel};
    use amg_doc::stub::ProfileKey;
    use amg_doc::{
        BodyGeom, BodyId, Command, ComputeMode, CreatedId, Design, EntityRef, Evaluated, Expr,
        ExprSrc, FeatureBox, FeatureId, GeomCache, GeomFingerprint, Health, ResolutionRecord,
        ResolveError, SaveOptions, SketchId, TopoKind, TopoRef, apply_command, engine_salt, load,
        recompute, save,
    };
    use amg_features::sketch::{PlaneInput, SketchFeature};
    use amg_features::solid::{
        BodyOp, Chamfer, Extent, Extrude, Fillet, FilletRadius, OpTarget, Start,
    };
    use amg_jobs::JobCtx;
    use amg_sketch::{EntityId, EntityKind};
    use serde_json::json;

    use super::*;

    /// A picked entity: body, kind, in-shape index (naive baseline).
    pub type Pick = (BodyId, TopoKind, u32);

    /// A sketch made by the `sketch` macro or by a translated test feature.
    struct SketchInfo {
        /// Case entity id (or polygon tag) → sketch entity.
        ids: HashMap<i64, EntityId>,
        /// The plane the world coordinates are converted with (face sketches, note §11.2).
        plane: Option<Plane>,
    }

    /// A lane-E test feature mapped to S04 features.
    struct Logical {
        /// The sketch of a translated box/prism (`None` for a fillet).
        sketch: Option<FeatureId>,
        /// The test feature's current `data`.
        data: Value,
        kind: String,
    }

    pub struct Run {
        pub design: Design,
        pub cache: Arc<GeomCache>,
        pub binds: HashMap<String, FeatureId>,
        /// (feature, n-th pick in its data) → the picked entity (naive baseline).
        pub picks: HashMap<(FeatureId, usize), Pick>,
        /// (feature, n-th pick) → the `near` point of the pick (fuzz oracle).
        pub pick_points: HashMap<(FeatureId, usize), [f64; 3]>,
        sketches: HashMap<FeatureId, SketchInfo>,
        logical: HashMap<FeatureId, Logical>,
        /// Sketches of translated features (not counted by logical timeline indices).
        hidden: HashSet<FeatureId>,
        tmp: PathBuf,
        pub log: Vec<String>,
    }

    pub fn kind_of(s: &str) -> TopoKind {
        match s {
            "Face" => TopoKind::Face,
            "Edge" => TopoKind::Edge,
            "Vertex" => TopoKind::Vertex,
            other => panic!("unknown kind {other}"),
        }
    }

    pub fn class_name(fp: &GeomFingerprint) -> String {
        format!("{:?}", fp.class)
    }

    pub fn vec3(v: &Value) -> [f64; 3] {
        let a = v.as_array().unwrap();
        [0, 1, 2].map(|i| a[i].as_f64().unwrap())
    }

    fn vec2(v: &Value) -> [f64; 2] {
        let a = v.as_array().unwrap();
        [a[0].as_f64().unwrap(), a[1].as_f64().unwrap()]
    }

    fn inline(s: impl Into<String>) -> Expr {
        Expr::Inline(ExprSrc(s.into()))
    }

    fn mm(v: &Value) -> String {
        match v {
            Value::String(s) => s.clone(),
            other => format!("{} mm", other.as_f64().unwrap()),
        }
    }

    impl Run {
        pub fn new(tmp: PathBuf) -> Run {
            amg_features::sketch::link();
            Run {
                design: Design::new("tnp"),
                cache: GeomCache::new(1 << 30),
                binds: HashMap::new(),
                picks: HashMap::new(),
                pick_points: HashMap::new(),
                sketches: HashMap::new(),
                logical: HashMap::new(),
                hidden: HashSet::new(),
                tmp,
                log: Vec::new(),
            }
        }

        pub fn eval_at(&self, marker: Option<usize>) -> Evaluated {
            let mut d = self.design.clone();
            if let Some(m) = marker {
                d.set_marker(m).unwrap();
            }
            recompute(&d, &self.cache, ComputeMode::Full, &JobCtx::detached())
                .unwrap()
                .evaluated
        }

        pub fn feature(&self, name: &str) -> FeatureId {
            let n = name.strip_prefix('$').unwrap_or(name);
            *self
                .binds
                .get(n)
                .unwrap_or_else(|| panic!("unbound feature {name}"))
        }

        fn real_index(&self, f: FeatureId) -> usize {
            self.design.timeline().node(f).unwrap().0
        }

        /// The timeline index to insert before `f` (before its sketch, for a translated
        /// box/prism), and the marker of `f`'s input state.
        pub fn index_of(&self, f: FeatureId) -> usize {
            match self.logical.get(&f).and_then(|l| l.sketch) {
                Some(s) => self.real_index(s),
                None => self.real_index(f),
            }
        }

        /// The real index of logical timeline index `n` (translated sketches not counted).
        fn logical_to_real(&self, n: usize) -> usize {
            let mut logical = 0;
            for node in self.design.timeline().nodes() {
                if self.hidden.contains(&node.id) {
                    continue;
                }
                if logical == n {
                    return self.index_of(node.id);
                }
                logical += 1;
            }
            self.design.timeline().len()
        }

        /// Expands macros in `v`. `state` is the evaluated state picks are made against.
        fn expand(
            &self,
            v: &Value,
            state: &Evaluated,
            body: Option<BodyId>,
            picks: &mut Vec<(Pick, [f64; 3])>,
            key: u32,
        ) -> Value {
            match v {
                Value::String(s) if s == "$root" => json!(self.design.root()),
                Value::String(s) if s.starts_with("$index_of:") => {
                    json!(self.index_of(self.feature(&s["$index_of:".len()..])))
                }
                Value::String(s) if s.starts_with('$') => json!(self.feature(s)),
                Value::Object(m) if m.len() == 1 && m.contains_key("$pick") => {
                    let r = self.pick(&m["$pick"], state, body, key, picks);
                    serde_json::to_value(r).unwrap()
                }
                Value::Object(m) if m.len() == 1 && m.contains_key("$pick_on") => {
                    let r = self.pick_on(&m["$pick_on"], state, body, key, picks);
                    serde_json::to_value(r).unwrap()
                }
                Value::Object(m) if m.len() == 1 && m.contains_key("$profile") => {
                    serde_json::to_value(self.profile(&m["$profile"], state)).unwrap()
                }
                Value::Object(m) => {
                    let body = m
                        .get("body")
                        .filter(|b| b.is_object())
                        .map(|b| {
                            serde_json::from_value(self.expand(b, state, None, picks, 0)).unwrap()
                        })
                        .or(body);
                    Value::Object(
                        m.iter()
                            .map(|(k, x)| (k.clone(), self.expand(x, state, body, picks, 1)))
                            .collect(),
                    )
                }
                Value::Array(a) => Value::Array(
                    (1u32..)
                        .zip(a)
                        .map(|(k, x)| self.expand(x, state, body, picks, k))
                        .collect(),
                ),
                other => other.clone(),
            }
        }

        /// `$pick`: the nearest entity of `kind` and `class` (by fingerprint centroid) to `near`
        /// in `body` (or any body); a tie within 1e-6 mm is a harness error (note §11.2).
        fn pick(
            &self,
            p: &Value,
            state: &Evaluated,
            body: Option<BodyId>,
            key: u32,
            picks: &mut Vec<(Pick, [f64; 3])>,
        ) -> TopoRef {
            let kind = kind_of(p["kind"].as_str().unwrap());
            let near_a = vec3(&p["near"]);
            let class = p["class"].as_str().unwrap();
            let near = Point3::from_array(near_a);
            let geom = KernelGeom::new(state, default_kernel());
            let bodies: Vec<BodyId> = match body {
                Some(b) => vec![b],
                None => state.bodies().map(|(id, _)| id).collect(),
            };
            let mut found: Vec<(f64, BodyId, u32)> = Vec::new();
            for b in bodies {
                for (i, fp) in geom.all(b, kind) {
                    if class_name(&fp) == class {
                        found.push(((fp.centroid - near).length(), b, i));
                    }
                }
            }
            found.sort_by(|a, b| a.0.total_cmp(&b.0));
            assert!(
                !found.is_empty(),
                "pick {p}: nothing of that kind and class"
            );
            if found.len() > 1 {
                assert!(
                    found[1].0 - found[0].0 > 1e-6,
                    "pick {p}: tied between two entities (broken case)"
                );
            }
            let (_, b, i) = found[0];
            let mut r = amg_doc::topo_ref_at(state, default_kernel(), b, kind, i)
                .unwrap_or_else(|| panic!("pick {p}: entity has no name"));
            r.key = key;
            picks.push(((b, kind, i), near_a));
            r
        }

        /// `$pick_on` (fuzz set, note §11.6): the Line edge whose geometry contains `at`; none or
        /// several is a harness error.
        fn pick_on(
            &self,
            p: &Value,
            state: &Evaluated,
            body: Option<BodyId>,
            key: u32,
            picks: &mut Vec<(Pick, [f64; 3])>,
        ) -> TopoRef {
            let at = vec3(&p["at"]);
            let bodies: Vec<BodyId> = match body {
                Some(b) => vec![b],
                None => state.bodies().map(|(id, _)| id).collect(),
            };
            let found: Vec<(BodyId, u32)> = bodies
                .into_iter()
                .flat_map(|b| {
                    lines_containing(state, b, at, 1e-6)
                        .into_iter()
                        .map(move |(i, _)| (b, i))
                })
                .collect();
            assert_eq!(
                found.len(),
                1,
                "pick_on {p}: {} edges contain the point",
                found.len()
            );
            let (b, i) = found[0];
            let mut r = amg_doc::topo_ref_at(state, default_kernel(), b, TopoKind::Edge, i)
                .unwrap_or_else(|| panic!("pick_on {p}: entity has no name"));
            r.key = key;
            picks.push(((b, TopoKind::Edge, i), at));
            r
        }

        /// `$profile`: the region of the sketch containing `at` (world x, y on a face sketch).
        fn profile(&self, p: &Value, state: &Evaluated) -> EntityRef {
            let s = self.feature(p["sketch"].as_str().unwrap());
            let at = self.local(s, vec2(&p["at"]));
            profile_at(state, s, at)
        }

        /// Case coordinates → sketch-local coordinates.
        fn local(&self, sketch: FeatureId, p: [f64; 2]) -> Point2 {
            to_local(self.sketches.get(&sketch).and_then(|i| i.plane), p)
        }

        fn add_feature_at(&self, at: &Value) -> Option<usize> {
            match at {
                Value::String(s) => Some(self.index_of(self.feature(&s["$index_of:".len()..]))),
                Value::Number(n) => Some(self.logical_to_real(n.as_u64().unwrap() as usize)),
                _ => None,
            }
        }

        /// Applies `cmd`; returns the first created feature.
        fn apply(&mut self, cmd: Command) -> Option<FeatureId> {
            let created = apply_command(&mut self.design, cmd)
                .unwrap_or_else(|e| panic!("command failed: {e}"));
            match created.first() {
                Some(CreatedId::Feature(f)) => Some(*f),
                _ => None,
            }
        }

        fn add(&mut self, at: Option<usize>, f: FeatureBox) -> FeatureId {
            let root = self.design.root();
            self.apply(Command::AddFeature {
                at,
                feature: f,
                context: root,
                name: None,
            })
            .expect("a feature was created")
        }

        fn record_picks(&mut self, f: FeatureId, picks: Vec<(Pick, [f64; 3])>) {
            for (n, (p, near)) in picks.into_iter().enumerate() {
                self.picks.insert((f, n), p);
                self.pick_points.insert((f, n), near);
            }
        }

        fn bind(&mut self, bind: Option<&str>, f: FeatureId) {
            if let Some(b) = bind {
                self.binds.insert(b.to_owned(), f);
            }
        }

        fn edit(&mut self, id: FeatureId, f: FeatureBox) {
            self.apply(Command::EditFeature { id, feature: f });
        }

        fn feature_json(&self, id: FeatureId) -> Value {
            serde_json::to_value(&self.design.timeline().node(id).unwrap().1.feature).unwrap()
        }

        fn sketch_feature(&self, id: FeatureId) -> SketchFeature {
            self.design
                .timeline()
                .node(id)
                .unwrap()
                .1
                .feature
                .downcast_ref::<SketchFeature>()
                .expect("a sketch")
                .clone()
        }

        pub fn step(&mut self, s: &Value) {
            let bind = s["bind"].as_str();
            if let Some(cmd) = s.get("cmd") {
                self.raw_command(cmd, bind);
            } else if let Some(p) = s.get("patch") {
                let id = self.feature(p["id"].as_str().unwrap());
                if self.logical.contains_key(&id) {
                    let mut data = self.logical[&id].data.clone();
                    merge_patch(&mut data, &p["merge"]);
                    self.retranslate(id, data);
                    return;
                }
                let mut f = self.feature_json(id);
                let state = self.eval_at(None);
                let merge = self.expand(&p["merge"], &state, None, &mut Vec::new(), 1);
                merge_patch_exprs(&mut f["data"], &merge);
                let cmd: Command =
                    serde_json::from_value(json!({"EditFeature": {"id": id, "feature": f}}))
                        .unwrap_or_else(|e| panic!("bad patch {merge}: {e}"));
                self.apply(cmd);
            } else if let Some(a) = s.get("accept") {
                let id = self.feature(a.as_str().unwrap());
                let eval = self.eval_at(None);
                let records = eval.node(id).unwrap().resolutions.clone();
                let mut f = self.feature_json(id);
                accept(&mut f["data"], &records);
                let cmd: Command =
                    serde_json::from_value(json!({"EditFeature": {"id": id, "feature": f}}))
                        .unwrap();
                self.apply(cmd);
            } else if s.get("saveload").is_some() {
                self.saveload();
            } else if let Some(sk) = s.get("sketch") {
                self.sketch_macro(sk, bind);
            } else if let Some(e) = s.get("sketch_edit") {
                self.sketch_edit_macro(e);
            } else if let Some(a) = s.get("EXT") {
                let a = a.as_array().unwrap();
                let mut args = a.clone();
                args[1] = json!([a[1].clone()]);
                self.ext_macro(&args, bind);
            } else if let Some(a) = s.get("EXTN") {
                self.ext_macro(a.as_array().unwrap(), bind);
            } else if let Some(a) = s.get("FIL") {
                self.edge_macro(a.as_array().unwrap(), bind, true);
            } else if let Some(a) = s.get("CHA") {
                self.edge_macro(a.as_array().unwrap(), bind, false);
            } else {
                panic!("unknown step {s}");
            }
        }

        fn raw_command(&mut self, cmd: &Value, bind: Option<&str>) {
            if let Some(add) = cmd.get("AddFeature") {
                let at = self.add_feature_at(&add["at"]);
                let kind = add["feature"]["kind"].as_str().unwrap_or_default();
                match kind {
                    "test.brep_box" | "test.brep_prism" | "test.brep_fillet" => {
                        let f = self.translate_add(at, kind, &add["feature"]["data"]);
                        self.bind(bind, f);
                        return;
                    }
                    _ => {}
                }
                let state = self.eval_at(at);
                let mut picks = Vec::new();
                let mut add = add.clone();
                add["at"] = json!(at);
                let expanded =
                    self.expand(&json!({"AddFeature": add}), &state, None, &mut picks, 1);
                let cmd: Command = serde_json::from_value(expanded.clone())
                    .unwrap_or_else(|e| panic!("bad command {expanded}: {e}"));
                let f = self.apply(cmd).unwrap();
                self.bind(bind, f);
                self.record_picks(f, picks);
                return;
            }
            if let Some(e) = cmd.get("EditFeature") {
                let id = self.feature(e["id"].as_str().unwrap());
                if self.logical.contains_key(&id) {
                    self.retranslate(id, e["feature"]["data"].clone());
                    return;
                }
            }
            if let Some(s) = cmd.get("SetSuppressed") {
                let mut ids = Vec::new();
                for i in s["ids"].as_array().unwrap() {
                    let f = self.feature(i.as_str().unwrap());
                    ids.push(f);
                    if let Some(sk) = self.logical.get(&f).and_then(|l| l.sketch) {
                        ids.push(sk);
                    }
                }
                self.apply(Command::SetSuppressed {
                    ids,
                    suppressed: s["suppressed"].as_bool().unwrap(),
                });
                return;
            }
            if let Some(m) = cmd.get("MoveFeature") {
                let f = self.feature(m["id"].as_str().unwrap());
                let to = self.logical_to_real(m["to"].as_u64().unwrap() as usize);
                match self.logical.get(&f).and_then(|l| l.sketch) {
                    Some(sk) => {
                        self.apply(Command::MoveFeature { id: sk, to });
                        self.apply(Command::MoveFeature { id: f, to: to + 1 });
                    }
                    None => {
                        self.apply(Command::MoveFeature { id: f, to });
                    }
                }
                return;
            }
            let state = self.eval_at(None);
            let expanded = self.expand(cmd, &state, None, &mut Vec::new(), 1);
            let cmd: Command = serde_json::from_value(expanded.clone())
                .unwrap_or_else(|e| panic!("bad command {expanded}: {e}"));
            self.apply(cmd);
        }

        // ---- lane-E test features → S04 features -----------------------------------------

        fn translate_add(&mut self, at: Option<usize>, kind: &str, data: &Value) -> FeatureId {
            match kind {
                "test.brep_fillet" => {
                    let state = self.eval_at(at);
                    let mut picks = Vec::new();
                    let body: BodyId = serde_json::from_value(self.expand(
                        &data["body"],
                        &state,
                        None,
                        &mut picks,
                        0,
                    ))
                    .unwrap();
                    let edges = self.expand(&data["edges"], &state, Some(body), &mut picks, 1);
                    let edges: Vec<TopoRef> = serde_json::from_value(edges).unwrap();
                    let fil = Fillet::constant(edges, &mm(&data["radius"]));
                    let f = self.add(at, FeatureBox::new(fil));
                    self.record_picks(f, picks);
                    self.logical.insert(
                        f,
                        Logical {
                            sketch: None,
                            data: data.clone(),
                            kind: kind.to_owned(),
                        },
                    );
                    f
                }
                _ => {
                    let (points, z0, h, op, target) = solid_params(kind, data);
                    let mut sk = SketchFeature::new(PlaneInput::OriginXY);
                    let mut ids = HashMap::new();
                    set_polygon(&mut sk, &mut ids, &points);
                    let s = self.add(at, FeatureBox::new(sk));
                    self.hidden.insert(s);
                    self.sketches.insert(s, SketchInfo { ids, plane: None });
                    let state = self.eval_at(None);
                    let profile = profile_at(&state, s, polygon_inside(&points));
                    let participants = match target {
                        Some(t) => vec![self.body_of(&t)],
                        None => Vec::new(),
                    };
                    let mut ext = Extrude::distance(
                        vec![profile],
                        &format!("{h} mm"),
                        OpTarget { op, participants },
                    );
                    if z0 != 0.0 {
                        ext.start = Start::Offset(inline(format!("{z0} mm")));
                    }
                    let f = self.add(at.map(|i| i + 1), FeatureBox::new(ext));
                    self.logical.insert(
                        f,
                        Logical {
                            sketch: Some(s),
                            data: data.clone(),
                            kind: kind.to_owned(),
                        },
                    );
                    f
                }
            }
        }

        /// Every B-rep feature of the run is an S04 feature (the lane-E B-rep test features
        /// were all mapped; `test.probe` only reads).
        pub fn assert_s04_only(&self) {
            for node in self.design.timeline().nodes() {
                let k = node.feature.kind();
                assert!(
                    !k.starts_with("test.brep_"),
                    "{k} left in the timeline (not mapped to S04)"
                );
            }
        }

        fn body_of(&self, v: &Value) -> BodyId {
            let state = self.eval_at(None);
            serde_json::from_value(self.expand(v, &state, None, &mut Vec::new(), 0)).unwrap()
        }

        /// A changed test feature (`EditFeature` or `patch`) applied to its S04 features.
        fn retranslate(&mut self, id: FeatureId, data: Value) {
            let l = self.logical.get_mut(&id).unwrap();
            l.data = data.clone();
            let kind = l.kind.clone();
            let sketch = l.sketch;
            match sketch {
                None => {
                    let mut fil = self
                        .design
                        .timeline()
                        .node(id)
                        .unwrap()
                        .1
                        .feature
                        .downcast_ref::<Fillet>()
                        .unwrap()
                        .clone();
                    fil.sets[0].radius = FilletRadius::Constant(inline(mm(&data["radius"])));
                    self.edit(id, FeatureBox::new(fil));
                }
                Some(s) => {
                    let (points, z0, h, _, _) = solid_params(&kind, &data);
                    let mut sk = self.sketch_feature(s);
                    let mut ids = std::mem::take(&mut self.sketches.get_mut(&s).unwrap().ids);
                    set_polygon(&mut sk, &mut ids, &points);
                    self.sketches.get_mut(&s).unwrap().ids = ids;
                    self.edit(s, FeatureBox::new(sk));
                    let state = self.eval_at(None);
                    let profile = profile_at(&state, s, polygon_inside(&points));
                    let mut ext = self
                        .design
                        .timeline()
                        .node(id)
                        .unwrap()
                        .1
                        .feature
                        .downcast_ref::<Extrude>()
                        .unwrap()
                        .clone();
                    ext.profiles = vec![profile];
                    ext.side1.extent = Extent::Distance(inline(format!("{h} mm")));
                    ext.start = if z0 == 0.0 {
                        Start::Profile
                    } else {
                        Start::Offset(inline(format!("{z0} mm")))
                    };
                    self.edit(id, FeatureBox::new(ext));
                }
            }
        }

        // ---- DOC-13 macros ----------------------------------------------------------------

        fn sketch_macro(&mut self, sk: &Value, bind: Option<&str>) {
            let state = self.eval_at(None);
            let mut picks = Vec::new();
            let plane = match &sk["plane"] {
                Value::String(s) if s == "XY" => PlaneInput::OriginXY,
                Value::String(s) if s == "XZ" => PlaneInput::OriginXZ,
                Value::String(s) if s == "YZ" => PlaneInput::OriginYZ,
                p => {
                    let r: TopoRef =
                        serde_json::from_value(self.expand(p, &state, None, &mut picks, 1))
                            .unwrap();
                    PlaneInput::Ref(EntityRef::Topo(r))
                }
            };
            let on_face = matches!(plane, PlaneInput::Ref(_));
            let f = self.add(None, FeatureBox::new(SketchFeature::new(plane)));
            self.bind(bind, f);
            self.record_picks(f, picks);
            let face_plane = if on_face {
                let eval = self.eval_at(None);
                let g = eval.sketch(SketchId(f)).unwrap_or_else(|| {
                    panic!(
                        "sketch on a face did not evaluate: {:?}",
                        eval.node(f).map(|n| n.health.clone())
                    )
                });
                Some(g.plane)
            } else {
                None
            };
            self.sketches.insert(
                f,
                SketchInfo {
                    ids: HashMap::new(),
                    plane: face_plane,
                },
            );
            self.sketch_edit(f, &[], sk["entities"].as_array().unwrap(), &[]);
        }

        fn sketch_edit_macro(&mut self, e: &Value) {
            let id = self.feature(e["id"].as_str().unwrap());
            let list = |k: &str| {
                e.get(k)
                    .and_then(Value::as_array)
                    .cloned()
                    .unwrap_or_default()
            };
            let remove: Vec<i64> = list("remove").iter().map(|v| v.as_i64().unwrap()).collect();
            self.sketch_edit(id, &remove, &list("add"), &list("move"));
        }

        fn sketch_edit(&mut self, id: FeatureId, remove: &[i64], add: &[Value], moves: &[Value]) {
            let mut sk = self.sketch_feature(id);
            let plane = self.sketches[&id].plane;
            let local = |p: &Value| to_local(plane, vec2(p));
            let mut ids = std::mem::take(&mut self.sketches.get_mut(&id).unwrap().ids);
            let remove_ids: Vec<EntityId> = remove.iter().map(|k| ids[k]).collect();
            for k in remove {
                ids.remove(k);
            }
            let mut new = Vec::new();
            for a in add {
                let a = a.as_array().unwrap();
                let key = a[1].as_i64().unwrap();
                new.push((a[0].as_str().unwrap().to_owned(), key, a[2..].to_vec()));
            }
            let mut moved = Vec::new();
            for m in moves {
                let m = m.as_array().unwrap();
                let e = ids[&m[0].as_i64().unwrap()];
                moved.push((e, local(&m[1]), local(&m[2])));
            }
            let new_conv: Vec<(String, i64, Vec<Value>)> = new;
            let added = sk
                .edit_sketch(|s| {
                    if !remove_ids.is_empty() {
                        s.remove_entities(&remove_ids)?;
                    }
                    let mut out = Vec::new();
                    for (kind, key, args) in &new_conv {
                        let e = match kind.as_str() {
                            "L" => s.add_line(local(&args[0]), local(&args[1]))?,
                            "C" => s.add_circle(local(&args[0]), args[1].as_f64().unwrap())?,
                            "P" => s.add_point(local(&args[0]))?,
                            other => panic!("unknown sketch entity {other}"),
                        };
                        out.push((*key, e));
                    }
                    for (e, a, b) in &moved {
                        move_line(s, *e, *a, *b);
                    }
                    Ok(out)
                })
                .unwrap();
            ids.extend(added);
            self.sketches.get_mut(&id).unwrap().ids = ids;
            self.edit(id, FeatureBox::new(sk));
        }

        fn ext_macro(&mut self, a: &[Value], bind: Option<&str>) {
            let state = self.eval_at(None);
            let sketch = a[0].clone();
            let profiles: Vec<EntityRef> = a[1]
                .as_array()
                .unwrap()
                .iter()
                .map(|at| self.profile(&json!({"sketch": sketch, "at": at}), &state))
                .collect();
            let op: BodyOp = serde_json::from_value(a[3].clone()).unwrap();
            let participants = a
                .get(4)
                .and_then(Value::as_array)
                .map(|ps| {
                    ps.iter()
                        .map(|p| BodyId {
                            origin: self.feature(p.as_str().unwrap()),
                            slot: 0,
                        })
                        .collect()
                })
                .unwrap_or_default();
            let extent = a[2].as_str().unwrap();
            let mut ext = Extrude::distance(profiles, extent, OpTarget { op, participants });
            if extent == "All" {
                ext.side1.extent = Extent::All;
            }
            let f = self.add(None, FeatureBox::new(ext));
            self.bind(bind, f);
        }

        fn edge_macro(&mut self, a: &[Value], bind: Option<&str>, fillet: bool) {
            let state = self.eval_at(None);
            let mut picks = Vec::new();
            let r: TopoRef =
                serde_json::from_value(self.expand(&a[0], &state, None, &mut picks, 1)).unwrap();
            let size = a[1].as_str().unwrap();
            let f = if fillet {
                FeatureBox::new(Fillet::constant(vec![r], size))
            } else {
                FeatureBox::new(Chamfer::equal(vec![r], size))
            };
            let f = self.add(None, f);
            self.bind(bind, f);
            self.record_picks(f, picks);
        }

        /// Note §11.2 `saveload`, with the geometry cache (DOC-14): names, keys and
        /// fingerprints must round-trip, and every output must come back from the file.
        fn saveload(&mut self) {
            let before = recompute(
                &self.design,
                &self.cache,
                ComputeMode::Full,
                &JobCtx::detached(),
            )
            .unwrap()
            .evaluated;
            let keys = before.nodes().filter_map(|(_, n)| n.key).collect();
            let path = self.tmp.join("saveload.amg");
            let opts = SaveOptions {
                cache: Some(CacheExport::new(Arc::clone(&self.cache), keys)),
                ..SaveOptions::default()
            };
            save(&path, &Arc::new(self.design.clone()), &opts).unwrap();
            let loaded = load(&path).unwrap();
            self.design = loaded.design;
            self.cache = GeomCache::new(1 << 30);
            assert!(
                loaded
                    .cache_seed
                    .attach_to(&self.cache, &engine_salt(&self.design.meta().tolerance)),
                "cache seed attaches"
            );
            let r = recompute(
                &self.design,
                &self.cache,
                ComputeMode::Full,
                &JobCtx::detached(),
            )
            .unwrap();
            // Solved sketches are not persisted in the cache yet (DOC-14, `io/cachefile.rs`):
            // they recompute on open; every other output must come from the file.
            let recomputed: Vec<FeatureId> = r
                .computed
                .iter()
                .copied()
                .filter(|f| {
                    let node = self.design.timeline().node(*f).unwrap().1;
                    node.feature.downcast_ref::<SketchFeature>().is_none()
                })
                .collect();
            assert!(
                recomputed.is_empty(),
                "every non-sketch output comes from the file: {recomputed:?}"
            );
            for (id, b) in before.bodies() {
                let after = r.evaluated.body(id).unwrap();
                assert_eq!(
                    after.naming, b.naming,
                    "element map of {id:?} survives save/load"
                );
            }
            self.log.push(format!(
                "saveload: {} outputs restored from the .amg cache ({} sketches recomputed), element maps equal",
                r.cached.len(),
                r.computed.len()
            ));
        }
    }

    /// World x, y (face sketches, note §11.2) or local coordinates → sketch-local.
    fn to_local(plane: Option<Plane>, p: [f64; 2]) -> Point2 {
        match plane {
            Some(plane) => plane.to_local(Point3::new(p[0], p[1], 0.0)),
            None => Point2::new(p[0], p[1]),
        }
    }

    /// (points `[tag, x, y]`, z0, height, op, target) of a test box/prism.
    fn solid_params(kind: &str, d: &Value) -> (Vec<[f64; 3]>, f64, f64, BodyOp, Option<Value>) {
        if kind == "test.brep_box" {
            let s = vec3(&d["size"]);
            let pts = vec![
                [1.0, 0.0, 0.0],
                [2.0, s[0], 0.0],
                [3.0, s[0], s[1]],
                [4.0, 0.0, s[1]],
            ];
            return (pts, 0.0, s[2], BodyOp::NewBody, None);
        }
        let pts = d["points"].as_array().unwrap().iter().map(vec3).collect();
        let op = match d["op"].as_str().unwrap() {
            "New" => BodyOp::NewBody,
            "Cut" => BodyOp::Cut,
            "Join" => BodyOp::Join,
            other => panic!("unknown prism op {other}"),
        };
        let target = Some(d["target"].clone()).filter(|t| !t.is_null());
        (
            pts,
            d["z0"].as_f64().unwrap(),
            d["height"].as_f64().unwrap(),
            op,
            target,
        )
    }

    /// A point inside a convex polygon (its vertex mean).
    fn polygon_inside(points: &[[f64; 3]]) -> Point2 {
        let n = points.len() as f64;
        Point2::new(
            points.iter().map(|p| p[1]).sum::<f64>() / n,
            points.iter().map(|p| p[2]).sum::<f64>() / n,
        )
    }

    /// Makes the sketch's lines the polygon `points` (`[tag, x, y]`): the line from point `i`
    /// to `i + 1` is keyed by point `i`'s tag. Lines of kept tags are moved (their entity, and
    /// so the lateral's lineage, is kept), new tags get new lines, dropped tags are removed.
    fn set_polygon(sk: &mut SketchFeature, ids: &mut HashMap<i64, EntityId>, points: &[[f64; 3]]) {
        let n = points.len();
        let tags: Vec<i64> = points.iter().map(|p| p[0] as i64).collect();
        let dropped: Vec<i64> = ids.keys().filter(|t| !tags.contains(t)).copied().collect();
        let dropped_ids: Vec<EntityId> = dropped.iter().map(|t| ids[t]).collect();
        for t in &dropped {
            ids.remove(t);
        }
        let existing = ids.clone();
        let added = sk
            .edit_sketch(|s| {
                if !dropped_ids.is_empty() {
                    s.remove_entities(&dropped_ids)?;
                }
                let mut out = Vec::new();
                for i in 0..n {
                    let a = Point2::new(points[i][1], points[i][2]);
                    let b = Point2::new(points[(i + 1) % n][1], points[(i + 1) % n][2]);
                    match existing.get(&tags[i]) {
                        Some(&e) => move_line(s, e, a, b),
                        None => out.push((tags[i], s.add_line(a, b)?)),
                    }
                }
                Ok(out)
            })
            .unwrap();
        ids.extend(added);
    }

    fn move_line(s: &mut amg_sketch::Sketch, e: EntityId, a: Point2, b: Point2) {
        let EntityKind::Line { start, end } = s.entity(e).expect("line exists").kind else {
            panic!("{e:?} is not a line");
        };
        for (p, at) in [(start, a), (end, b)] {
            let EntityKind::Point { x, y } = s.entity(p).unwrap().kind else {
                panic!("line end is not a point");
            };
            s.params_mut().set(x, at.x);
            s.params_mut().set(y, at.y);
        }
    }

    /// `EntityRef::Profile` of the region of `sketch` containing `at` (local coordinates).
    fn profile_at(state: &Evaluated, sketch: FeatureId, at: Point2) -> EntityRef {
        let g = state
            .sketch(SketchId(sketch))
            .unwrap_or_else(|| panic!("sketch {sketch} is not evaluated"));
        let id = g
            .profiles
            .at_point(at)
            .unwrap_or_else(|| panic!("no profile of {sketch} at {at:?}"));
        let p = g.profiles.get(id).unwrap();
        EntityRef::Profile {
            sketch: SketchId(sketch),
            profile: ProfileKey(id.0),
            fingerprint: Some(ProfileFp(p.fingerprint.clone())),
        }
    }

    /// Accept match (note §7): each ref of the feature data is replaced by its record's
    /// `current`, key kept.
    fn accept(v: &mut Value, records: &[ResolutionRecord]) {
        if let Ok(r) = serde_json::from_value::<TopoRef>(v.clone()) {
            for rec in records {
                if let (EntityRef::Topo(t), [EntityRef::Topo(cur)]) =
                    (&rec.r, rec.current.as_slice())
                    && t.key == r.key
                    && *t == r
                {
                    *v = serde_json::to_value(cur).unwrap();
                    return;
                }
            }
            return;
        }
        match v {
            Value::Object(m) => m.values_mut().for_each(|x| accept(x, records)),
            Value::Array(a) => a.iter_mut().for_each(|x| accept(x, records)),
            _ => {}
        }
    }

    pub fn entity_ok(e: &Value, fp: &GeomFingerprint) -> Result<(), String> {
        let class = e["class"].as_str().unwrap();
        let curved = !matches!(class, "Line" | "Plane");
        let tol = if curved { 1e-4 } else { 1e-6 };
        let c = vec3(&e["centroid"]);
        let m = e["measure"].as_f64().unwrap();
        let dc = (fp.centroid - Point3::from_array(c)).length();
        let dm = (fp.measure - m).abs() / m.abs().max(1e-12);
        if class_name(fp) != class || dc > tol || dm > tol {
            return Err(format!(
                "entity {} {:?} measure {} (expected {class} {c:?} {m}; dc {dc:.2e}, dm {dm:.2e})",
                class_name(fp),
                fp.centroid.to_array(),
                fp.measure
            ));
        }
        Ok(())
    }

    pub fn variant(dbg: &str) -> &str {
        dbg.split([' ', '{', '(']).next().unwrap_or(dbg)
    }

    /// Per-expectation outcome class of the DOC-13 table.
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub enum Class {
        /// Meets the recorded `expect`.
        Met,
        /// The subject feature (or a base feature) failed.
        FeatureError,
        /// Misses the `expect` but is flagged (warning or error resolution).
        Warning,
        /// A quiet `Ok` on a wrong entity, or where an error is expected (note §11.2).
        SilentWrong,
    }

    pub struct Checked {
        pub failures: Vec<String>,
        pub class: Class,
        pub summary: String,
    }

    pub fn check(run: &Run, eval: &Evaluated, exp: &Value) -> Checked {
        let mut failures = Vec::new();
        let subj = &exp["subject"];
        let fid = run.feature(subj["feature"].as_str().unwrap());
        let n = subj["ref"].as_u64().unwrap() as usize;
        let node = eval.node(fid).expect("subject evaluated");
        let failed = matches!(node.health, Health::Error(_));
        if let Some(h) = exp.get("health").and_then(Value::as_str) {
            let got = match node.health {
                Health::Ok => "Ok",
                Health::Warning(_) => "Warning",
                Health::Error(_) => "Error",
                _ => "Other",
            };
            if got != h {
                failures.push(format!("health {got}, expected {h} ({:?})", node.health));
            }
        }
        let Some(rec) = node.resolutions.get(n) else {
            return Checked {
                failures: vec![format!(
                    "no record {n} (health {:?}, {} records)",
                    node.health,
                    node.resolutions.len()
                )],
                class: if failed {
                    Class::FeatureError
                } else {
                    Class::Warning
                },
                summary: "missing".into(),
            };
        };
        let outcome = match &rec.result {
            Ok(v) if v[0].warning.is_some() => "Warning",
            Ok(_) => "Ok",
            Err(_) => "Error",
        };
        let want = exp["outcome"].as_str().unwrap();
        if outcome != want {
            failures.push(format!(
                "outcome {outcome}, expected {want}: {:?}",
                rec.result
            ));
        }
        let mut summary = outcome.to_owned();
        let mut entity_wrong = false;
        match &rec.result {
            Ok(v) => {
                let q = format!("{:?}", v[0].quality);
                summary = format!("{summary} {q}");
                if let Some(w) = exp.get("quality").and_then(Value::as_str)
                    && w != q
                {
                    failures.push(format!("quality {q}, expected {w}"));
                }
                let got_w = v[0].warning.map(|w| format!("{w:?}"));
                if let Some(w) = &got_w {
                    summary = format!("{summary} {}", variant(w));
                }
                match (exp.get("warning").and_then(Value::as_str), &got_w) {
                    (Some(w), Some(g)) if variant(g) != w => {
                        failures.push(format!("warning {g}, expected {w}"));
                    }
                    (Some(w), None) => failures.push(format!("no warning, expected {w}")),
                    _ => {}
                }
                if let Some(e) = exp.get("entity") {
                    let fp = match rec.current.first() {
                        Some(EntityRef::Topo(t)) => t.fingerprint.clone(),
                        _ => None,
                    };
                    match fp {
                        Some(fp) => {
                            let c = fp.centroid;
                            summary = format!(
                                "{summary} @ {} ({:.4}, {:.4}, {:.4}) m {:.4}",
                                class_name(&fp),
                                c.x,
                                c.y,
                                c.z,
                                fp.measure
                            );
                            if let Err(m) = entity_ok(e, &fp) {
                                entity_wrong = true;
                                failures.push(m);
                            }
                        }
                        None => failures.push("no fingerprint for the chosen entity".into()),
                    }
                    if let Some(b) = exp.get("body") {
                        let want: BodyId =
                            serde_json::from_value(run.expand(b, eval, None, &mut Vec::new(), 0))
                                .unwrap();
                        if v[0].body != want {
                            entity_wrong = true;
                            failures.push(format!("body {:?}, expected {want:?}", v[0].body));
                        }
                    }
                }
            }
            Err(err) => {
                let name = variant(&format!("{err:?}")).to_owned();
                summary = format!("{summary} {name}");
                if let ResolveError::NotFound {
                    removed_by: Some(f),
                    ..
                } = err
                {
                    summary = format!("{summary} (removed by {f})");
                }
                if let Some(w) = exp.get("error").and_then(Value::as_str)
                    && w != name
                {
                    failures.push(format!("error {name}, expected {w}"));
                }
            }
        }
        if failed {
            summary = format!("{summary}; feature Error");
        }
        // Note §11.2: a quiet Ok on a wrong entity, or where an error is expected.
        let quiet_ok = matches!(&rec.result, Ok(v) if v[0].warning.is_none());
        let silent_wrong = quiet_ok && (entity_wrong || want == "Error");
        let class = if failures.is_empty() {
            Class::Met
        } else if silent_wrong {
            Class::SilentWrong
        } else if failed {
            Class::FeatureError
        } else {
            Class::Warning
        };
        Checked {
            failures,
            class,
            summary,
        }
    }

    /// The naive index baseline (note §11.2): the picked index of the subject ref, read at the
    /// subject's input state after the edit. `None` for cases without an expected entity.
    pub fn naive_breaks(run: &Run, exp: &Value) -> Option<bool> {
        let e = exp.get("entity")?;
        let subj = &exp["subject"];
        let fid = run.feature(subj["feature"].as_str().unwrap());
        let n = subj["ref"].as_u64().unwrap() as usize;
        let &(body, kind, index) = run.picks.get(&(fid, n))?;
        let state = run.eval_at(Some(run.index_of(fid)));
        let geom = KernelGeom::new(&state, default_kernel());
        let count = state
            .naming(body)
            .and_then(|nm| nm.list(kind))
            .map_or(0, <[amg_doc::TopoName]>::len);
        if index as usize >= count {
            return Some(true);
        }
        let fp = geom.fingerprint(body, kind, index)?;
        Some(entity_ok(e, &fp).is_err())
    }

    /// The first failed feature of a state, if any.
    pub fn first_error(eval: &Evaluated) -> Option<String> {
        eval.nodes().find_map(|(f, n)| match &n.health {
            Health::Error(d) => Some(format!("feature {f} failed: {}", d.message)),
            _ => None,
        })
    }

    /// Line edges of `body` in `state` containing `p` (within `tol`), as (index, fingerprint).
    pub fn lines_containing(
        state: &Evaluated,
        body: BodyId,
        p: [f64; 3],
        tol: f64,
    ) -> Vec<(u32, GeomFingerprint)> {
        use amg_brep::{BrepKernel, EdgeId};
        let Some(b) = state.body(body) else {
            return Vec::new();
        };
        let BodyGeom::Solid(s) = &b.geom else {
            return Vec::new();
        };
        let Some(shape) = s.brep() else {
            return Vec::new();
        };
        let p = Point3::from_array(p);
        let geom = KernelGeom::new(state, default_kernel());
        geom.all(body, TopoKind::Edge)
            .into_iter()
            .filter(|(_, fp)| class_name(fp) == "Line")
            .filter(|(i, _)| {
                let Ok(ends) = default_kernel().edge_ends(shape, EdgeId(*i)) else {
                    return false;
                };
                let (a, b) = (ends.p0, ends.p1);
                let ab = b - a;
                let t = ((p - a).dot(ab) / ab.dot(ab)).clamp(0.0, 1.0);
                (a + ab * t - p).length() <= tol
            })
            .collect()
    }

    /// One case's result.
    pub struct CaseResult {
        pub classes: Vec<Class>,
        pub text: String,
        pub naive: Option<bool>,
    }

    impl CaseResult {
        pub fn ok(&self) -> bool {
            self.classes.iter().all(|c| *c == Class::Met)
        }
        pub fn silent_wrong(&self) -> bool {
            self.classes.contains(&Class::SilentWrong)
        }
        /// The table outcome: the worst class of the case's expectations.
        pub fn outcome(&self) -> &'static str {
            if self.silent_wrong() {
                "silent wrong"
            } else if self.classes.contains(&Class::FeatureError) {
                "feature error"
            } else if self.classes.contains(&Class::Warning) {
                "warning"
            } else {
                "expect met"
            }
        }
    }

    pub fn run_case(case: &Value, tmp: &Path) -> CaseResult {
        let id = case["id"].as_str().unwrap();
        let mut run = Run::new(tmp.join(id));
        std::fs::create_dir_all(&run.tmp).unwrap();
        for s in &base_steps(case) {
            run.step(s);
        }
        let base = run.eval_at(None);
        if let Some(e) = first_error(&base) {
            return CaseResult {
                classes: vec![Class::FeatureError],
                text: format!("base: {e}"),
                naive: None,
            };
        }
        for s in case["edit"].as_array().unwrap() {
            run.step(s);
        }
        let eval = run.eval_at(None);
        run.assert_s04_only();
        let exps: Vec<Value> = match &case["expect"] {
            Value::Array(a) => a.clone(),
            e => vec![e.clone()],
        };
        let mut classes = Vec::new();
        let mut text = Vec::new();
        let mut naive = None;
        for e in &exps {
            let c = check(&run, &eval, e);
            classes.push(c.class);
            text.push(if c.failures.is_empty() {
                c.summary
            } else {
                format!("{} FAIL: {}", c.summary, c.failures.join("; "))
            });
            if naive.is_none() {
                naive = naive_breaks(&run, e);
            }
        }
        text.extend(run.log.iter().cloned());
        CaseResult {
            classes,
            text: text.join(" | "),
            naive,
        }
    }
}
