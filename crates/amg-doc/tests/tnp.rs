//! DOC-06 Done-when 2-4: the DOC-06-phase cases of the golden TNP corpus
//! (`tests/golden/tnp/`, persistent-naming note §11) on the lane-E B-rep test features.
//!
//! The harness expands the corpus macros of note §11.2 (`$root`, `$name`, `$index_of:`,
//! `$pick`, `patch`, `accept`, `saveload`), runs the base, recomputes, applies the edit,
//! recomputes and checks every `expect` against the subject feature's `ResolutionRecord`. It
//! also measures the naive index baseline (the picked in-shape index read back after the edit)
//! and counts silent wrong resolutions (note §11.2). The run needs OCCT (`--features occt`);
//! without it only the corpus files are parsed.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stdout,
    reason = "test harness"
)]

use std::path::{Path, PathBuf};

use serde_json::{Map, Value, json};

fn corpus_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/tnp")
}

fn read_json(p: &Path) -> Value {
    let s = std::fs::read_to_string(p).unwrap_or_else(|e| panic!("{}: {e}", p.display()));
    serde_json::from_str(&s).unwrap_or_else(|e| panic!("{}: {e}", p.display()))
}

fn cases() -> Vec<Value> {
    let mut v: Vec<PathBuf> = std::fs::read_dir(corpus_dir().join("cases"))
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .collect();
    v.sort();
    v.iter().map(|p| read_json(p)).collect()
}

#[test]
fn corpus_files_parse_and_have_the_recorded_phases() {
    let all = cases();
    assert_eq!(all.len(), 30, "30 core cases (note §11.4)");
    let doc06: Vec<&str> = all
        .iter()
        .filter(|c| c["phase"] == "DOC-06")
        .map(|c| c["id"].as_str().unwrap())
        .collect();
    assert_eq!(
        doc06,
        [
            "TNP-01", "TNP-02", "TNP-03", "TNP-04", "TNP-05", "TNP-06", "TNP-07", "TNP-08",
            "TNP-09", "TNP-10", "TNP-11", "TNP-12", "TNP-24", "TNP-25", "TNP-26", "TNP-27",
            "TNP-28"
        ]
    );
    for c in &all {
        for key in ["id", "phase", "edit", "expect"] {
            assert!(c.get(key).is_some(), "{}: missing {key}", c["id"]);
        }
        if let Some(b) = c["base"].as_str() {
            assert!(corpus_dir().join(format!("bases/{b}.json")).exists());
        }
        if let Some(s) = c["script"].as_str() {
            assert!(corpus_dir().join(format!("bases/{s}.json")).exists());
        }
    }
}

/// JSON Merge Patch (RFC 7396).
fn merge_patch(target: &mut Value, patch: &Value) {
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

#[test]
fn merge_patch_follows_rfc_7396() {
    let mut a = json!({"a": 1, "b": {"c": 2, "d": 3}});
    merge_patch(&mut a, &json!({"b": {"c": null, "e": 4}, "f": [1]}));
    assert_eq!(a, json!({"a": 1, "b": {"d": 3, "e": 4}, "f": [1]}));
}

#[cfg(feature = "occt")]
mod run {
    use std::sync::Arc;

    use amg_doc::io::CacheExport;
    use amg_doc::naming::{GeomSource, KernelGeom, NamingView, default_kernel};
    use amg_doc::{
        BodyId, Command, ComputeMode, CreatedId, Design, EntityRef, Evaluated, FeatureId,
        GeomCache, GeomFingerprint, Health, MatchQuality, ResolutionRecord, ResolveError,
        ResolveWarning, SaveOptions, TopoKind, TopoRef, apply_command, engine_salt, load,
        recompute, save,
    };
    use amg_jobs::JobCtx;
    use std::collections::HashMap;

    use super::*;

    type Pick = (BodyId, TopoKind, u32);

    struct Run {
        design: Design,
        cache: Arc<GeomCache>,
        binds: HashMap<String, FeatureId>,
        /// (feature, n-th pick in its data) → the picked entity (naive baseline).
        picks: HashMap<(FeatureId, usize), Pick>,
        tmp: PathBuf,
        log: Vec<String>,
    }

    fn kind_of(s: &str) -> TopoKind {
        match s {
            "Face" => TopoKind::Face,
            "Edge" => TopoKind::Edge,
            "Vertex" => TopoKind::Vertex,
            other => panic!("unknown kind {other}"),
        }
    }

    fn class_name(fp: &GeomFingerprint) -> String {
        format!("{:?}", fp.class)
    }

    fn vec3(v: &Value) -> [f64; 3] {
        let a = v.as_array().unwrap();
        [0, 1, 2].map(|i| a[i].as_f64().unwrap())
    }

    impl Run {
        fn new(tmp: PathBuf) -> Run {
            Run {
                design: Design::new("tnp"),
                cache: GeomCache::new(1 << 30),
                binds: HashMap::new(),
                picks: HashMap::new(),
                tmp,
                log: Vec::new(),
            }
        }

        fn eval_at(&self, marker: Option<usize>) -> Evaluated {
            let mut d = self.design.clone();
            if let Some(m) = marker {
                d.set_marker(m).unwrap();
            }
            recompute(&d, &self.cache, ComputeMode::Full, &JobCtx::detached())
                .unwrap()
                .evaluated
        }

        fn feature(&self, name: &str) -> FeatureId {
            let n = name.strip_prefix('$').unwrap_or(name);
            *self
                .binds
                .get(n)
                .unwrap_or_else(|| panic!("unbound feature {name}"))
        }

        fn index_of(&self, f: FeatureId) -> usize {
            self.design.timeline().node(f).unwrap().0
        }

        /// Expands macros in `v`. `state` is the evaluated state picks are made against.
        fn expand(
            &self,
            v: &Value,
            state: &Evaluated,
            body: Option<BodyId>,
            picks: &mut Vec<Pick>,
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
                Value::Object(m) => {
                    let body = m
                        .get("body")
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
            picks: &mut Vec<Pick>,
        ) -> TopoRef {
            let kind = kind_of(p["kind"].as_str().unwrap());
            let near = vec3(&p["near"]);
            let class = p["class"].as_str().unwrap();
            let near = amg_core::Point3::from_array(near);
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
            picks.push((b, kind, i));
            r
        }

        fn add_feature_json(&self, cmd: &Value) -> Option<usize> {
            let add = cmd.get("AddFeature")?;
            Some(match add["at"].as_str() {
                Some(s) => self.index_of(self.feature(&s["$index_of:".len()..])),
                None => add["at"]
                    .as_u64()
                    .map_or(self.design.timeline().len(), |x| x as usize),
            })
        }

        fn apply(&mut self, cmd: Command, bind: Option<&str>, picks: Vec<Pick>) {
            let created = apply_command(&mut self.design, cmd)
                .unwrap_or_else(|e| panic!("command failed: {e}"));
            if let Some(CreatedId::Feature(f)) = created.first() {
                if let Some(b) = bind {
                    self.binds.insert(b.to_owned(), *f);
                }
                for (n, p) in picks.into_iter().enumerate() {
                    self.picks.insert((*f, n), p);
                }
            }
        }

        fn step(&mut self, s: &Value) {
            let bind = s["bind"].as_str();
            if let Some(cmd) = s.get("cmd") {
                let state = self.eval_at(self.add_feature_json(cmd));
                let mut picks = Vec::new();
                let expanded = self.expand(cmd, &state, None, &mut picks, 1);
                let cmd: Command = serde_json::from_value(expanded.clone())
                    .unwrap_or_else(|e| panic!("bad command {expanded}: {e}"));
                self.apply(cmd, bind, picks);
            } else if let Some(p) = s.get("patch") {
                let id = self.feature(p["id"].as_str().unwrap());
                let mut f =
                    serde_json::to_value(&self.design.timeline().node(id).unwrap().1.feature)
                        .unwrap();
                let at = self.index_of(id);
                let state = self.eval_at(Some(at));
                let mut picks = Vec::new();
                let merge = self.expand(&p["merge"], &state, None, &mut picks, 1);
                merge_patch(&mut f["data"], &merge);
                let cmd: Command =
                    serde_json::from_value(json!({"EditFeature": {"id": id, "feature": f}}))
                        .unwrap();
                self.apply(cmd, None, Vec::new());
            } else if let Some(a) = s.get("accept") {
                let id = self.feature(a.as_str().unwrap());
                let eval = self.eval_at(None);
                let records = eval.node(id).unwrap().resolutions.clone();
                let mut f =
                    serde_json::to_value(&self.design.timeline().node(id).unwrap().1.feature)
                        .unwrap();
                accept(&mut f["data"], &records);
                let cmd: Command =
                    serde_json::from_value(json!({"EditFeature": {"id": id, "feature": f}}))
                        .unwrap();
                self.apply(cmd, None, Vec::new());
            } else if s.get("saveload").is_some() {
                self.saveload();
            } else {
                panic!("unknown step {s}");
            }
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
            assert!(
                r.computed.is_empty(),
                "every output comes from the file: {:?}",
                r.computed
            );
            for (id, b) in before.bodies() {
                let after = r.evaluated.body(id).unwrap();
                assert_eq!(
                    after.naming, b.naming,
                    "element map of {id:?} survives save/load"
                );
            }
            self.log.push(format!(
                "saveload: {} outputs restored from the .amg cache, element maps equal",
                r.cached.len()
            ));
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

    fn entity_ok(e: &Value, fp: &GeomFingerprint) -> Result<(), String> {
        let class = e["class"].as_str().unwrap();
        let curved = !matches!(class, "Line" | "Plane");
        let tol = if curved { 1e-4 } else { 1e-6 };
        let c = vec3(&e["centroid"]);
        let m = e["measure"].as_f64().unwrap();
        let dc = (fp.centroid - amg_core::Point3::from_array(c)).length();
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

    fn variant(dbg: &str) -> &str {
        dbg.split([' ', '{', '(']).next().unwrap_or(dbg)
    }

    struct Checked {
        failures: Vec<String>,
        silent_wrong: bool,
        summary: String,
    }

    fn check(run: &Run, eval: &Evaluated, exp: &Value) -> Checked {
        let mut failures = Vec::new();
        let subj = &exp["subject"];
        let fid = run.feature(subj["feature"].as_str().unwrap());
        let n = subj["ref"].as_u64().unwrap() as usize;
        let node = eval.node(fid).expect("subject evaluated");
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
                silent_wrong: false,
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
        // Note §11.2: a quiet Ok on a wrong entity, or where an error is expected.
        let quiet_ok = matches!(&rec.result, Ok(v) if v[0].warning.is_none());
        let silent_wrong = quiet_ok && (entity_wrong || want == "Error");
        let _ = MatchQuality::Exact;
        let _: Option<ResolveWarning> = None;
        Checked {
            failures,
            silent_wrong,
            summary,
        }
    }

    /// The naive index baseline (note §11.2): the picked index of the subject ref, read at the
    /// subject's input state after the edit. `None` for cases without an expected entity.
    fn naive_breaks(run: &Run, exp: &Value) -> Option<bool> {
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

    fn run_case(case: &Value, tmp: &Path) -> (bool, bool, String, Option<bool>) {
        let id = case["id"].as_str().unwrap();
        let mut run = Run::new(tmp.join(id));
        std::fs::create_dir_all(&run.tmp).unwrap();
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
        for s in &steps {
            run.step(s);
        }
        let base = run.eval_at(None);
        for (f, n) in base.nodes() {
            if let Health::Error(d) = &n.health {
                return (
                    false,
                    false,
                    format!("base: feature {f} failed: {}", d.message),
                    None,
                );
            }
        }
        for s in case["edit"].as_array().unwrap() {
            run.step(s);
        }
        let eval = run.eval_at(None);
        let exps: Vec<Value> = match &case["expect"] {
            Value::Array(a) => a.clone(),
            e => vec![e.clone()],
        };
        let mut ok = true;
        let mut silent = false;
        let mut text = Vec::new();
        let mut naive = None;
        for e in &exps {
            let c = check(&run, &eval, e);
            ok &= c.failures.is_empty();
            silent |= c.silent_wrong;
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
        (ok, silent, text.join(" | "), naive)
    }

    #[test]
    fn doc06_corpus_meets_its_expectations_with_no_silent_wrong() {
        let tmp = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("tnp");
        std::fs::create_dir_all(&tmp).unwrap();
        let mut report = Vec::new();
        let mut failed = Vec::new();
        let mut silent = 0;
        let mut breaks = 0;
        for case in cases().iter().filter(|c| c["phase"] == "DOC-06") {
            let id = case["id"].as_str().unwrap().to_owned();
            let started = std::time::Instant::now();
            let (ok, sw, text, naive) = run_case(case, &tmp);
            let ms = started.elapsed().as_millis();
            println!(
                "{id}: {} [{ms} ms] {text}",
                if ok { "ok" } else { "FAILED" }
            );
            if !ok {
                failed.push(id.clone());
            }
            if sw {
                silent += 1;
            }
            if naive == Some(true) {
                breaks += 1;
            }
            let predicted = &case["naive_index_breaks"];
            if let (Some(m), Some(p)) = (naive, predicted.as_bool())
                && m != p
            {
                println!("{id}: naive baseline measured {m}, predicted {p} (correct the case)");
            }
            report.push(json!({
                "id": id, "ok": ok, "silent_wrong": sw, "result": text,
                "naive_index_breaks_measured": naive, "naive_index_breaks_predicted": predicted,
                "ms": ms,
            }));
        }
        let path = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("tnp-report.json");
        std::fs::write(&path, serde_json::to_string_pretty(&report).unwrap()).unwrap();
        println!(
            "{} DOC-06 cases, {} failed, {silent} silent wrong, {breaks} measured to break the naive index baseline; report {}",
            report.len(),
            failed.len(),
            path.display()
        );
        assert_eq!(silent, 0, "silent wrong resolutions");
        assert!(
            failed.is_empty(),
            "cases not meeting their expect: {failed:?}"
        );
    }
}
