use super::*;
use crate::id::BodyId;
use crate::model::BodyProps;
use amg_core::{LengthUnit, Transform};

const L: Dimension = Dimension::LENGTH;

/// Saves and reloads `d`; the reload must succeed, validate, be byte-identical and carry
/// the same values.
#[track_caller]
fn rt(d: &Design) -> Design {
    d.validate().unwrap();
    let s = serde_json::to_string(d).unwrap();
    let back: Design =
        serde_json::from_str(&s).unwrap_or_else(|e| panic!("saved design won't load: {e}\n{s}"));
    assert_eq!(serde_json::to_string(&back).unwrap(), s);
    for (id, _) in d.params().iter() {
        assert_eq!(back.params().value(id), d.params().value(id));
    }
    back
}

fn mm(v: f64) -> Option<Quantity> {
    Some(Quantity::new(v, L))
}

fn design() -> Design {
    Design::with_uuid("P", "0190f5a3-0000-7000-8000-000000000001".parse().unwrap())
}

#[test]
fn add_and_read_parameters() {
    let mut d = design();
    let rev = d.revision();
    let w = d
        .add_param("width", "40", Some(Unit::Millimetre), "plate")
        .unwrap();
    let h = d
        .add_param("height", "width / 2 + 1 cm", Some(Unit::Millimetre), "")
        .unwrap();
    let t = d.add_param("thick", "1/8", Some(Unit::Inch), "").unwrap();
    let r = d.add_param("ratio", "width / height", None, "").unwrap();
    assert!(d.revision() > rev);
    let p = d.params();
    assert_eq!(p.len(), 4);
    assert_eq!(p.by_name("width"), Some(w));
    assert_eq!(p.get(w).unwrap().comment, "plate");
    assert_eq!(p.value(w), mm(40.0));
    assert_eq!(p.value(h), mm(30.0));
    assert_eq!(p.value(t), mm(3.175));
    assert_eq!(p.value(r), Some(Quantity::dimensionless(40.0 / 30.0)));
    assert_eq!(p.references(h), &[w]);
    assert_eq!(p.dependents(w), vec![h, r]);
    assert_eq!(
        d.try_eval("width + 5", Some(L)).unwrap(),
        Quantity::new(45.0, L)
    );
    rt(&d);
}

#[test]
fn rejected_edits_change_nothing() {
    let mut d = design();
    let a = d
        .add_param("a", "10 mm", Some(Unit::Millimetre), "")
        .unwrap();
    let before = serde_json::to_string(&d).unwrap();
    let rev = d.revision();
    let next = d.meta().next_id;
    let cases: Vec<(Result<ParamId, ParamError>, &str)> = vec![
        (d.clone().add_param("b", "a + x", None, ""), "unknown"),
        (d.clone().add_param("b", "a +", None, ""), "syntax"),
        (
            d.clone()
                .add_param("b", "5 deg", Some(Unit::Millimetre), ""),
            "dimension",
        ),
        (d.clone().add_param("b", "a + 1 s", None, ""), "mismatch"),
        (d.clone().add_param("a", "1", None, ""), "taken"),
        (d.clone().add_param("mm", "1", None, ""), "reserved"),
        (d.clone().add_param("sin", "1", None, ""), "reserved"),
        (d.clone().add_param("2x", "1", None, ""), "invalid"),
        (d.clone().add_param("b", "1 / 0", None, ""), "domain"),
    ];
    for (r, what) in cases {
        assert!(r.is_err(), "{what}");
    }
    assert!(
        matches!(d.add_param("b", "a + x", None, ""), Err(ParamError::Expr(ExprError::UnknownParam { span, .. })) if span == (4..5))
    );
    assert_eq!(
        d.add_param("a", "1", None, ""),
        Err(ParamError::NameTaken("a".into()))
    );
    assert_eq!(
        d.add_param("in", "1", None, ""),
        Err(ParamError::Expr(ExprError::ReservedName("in".into())))
    );
    assert!(d.set_param_expr(a, "5 deg").is_err());
    assert!(d.set_param_expr(ParamId(999), "1").is_err());
    assert_eq!(serde_json::to_string(&d).unwrap(), before);
    assert_eq!(d.revision(), rev);
    assert_eq!(d.meta().next_id, next, "failed adds allocate no id");
    assert_eq!(d.params().value(a), mm(10.0));
}

#[test]
fn self_reference_is_a_cycle() {
    let mut d = design();
    // The name is registered before its expression is resolved, so a self reference is a cycle.
    assert!(matches!(
        d.add_param("a", "a + 1", None, ""),
        Err(ParamError::Expr(ExprError::Cycle(p))) if p.len() == 2 && p[0] == p[1]
    ));
    let a = d.add_param("a", "1", None, "").unwrap();
    assert_eq!(
        d.set_param_expr(a, "a + 1"),
        Err(ParamError::Expr(ExprError::Cycle(vec![a, a])))
    );
}

#[test]
fn cycles_are_rejected_with_their_path() {
    let mut d = design();
    let a = d.add_param("a", "1 mm", None, "").unwrap();
    let b = d.add_param("b", "a * 2", None, "").unwrap();
    // a = b while b = a.
    assert_eq!(
        d.set_param_expr(a, "b"),
        Err(ParamError::Expr(ExprError::Cycle(vec![a, b, a])))
    );
    let c = d.add_param("c", "b + a", None, "").unwrap();
    let e = d.add_param("e", "c - 1 mm", None, "").unwrap();
    assert_eq!(
        d.set_param_expr(a, "e / 2"),
        Err(ParamError::Expr(ExprError::Cycle(vec![a, e, c, a])))
    );
    // (c = b + a also closes a -> e -> c -> a, which the DFS finds before the longer path.)
    // Diamond shapes are not cycles.
    d.set_param_expr(e, "c + b + a").unwrap();
    assert_eq!(d.params().value(e), mm(1.0 + 2.0 + 3.0));
    rt(&d);
}

#[test]
fn edits_propagate_to_dependents() {
    let mut d = design();
    let a = d.add_param("a", "10", Some(Unit::Millimetre), "").unwrap();
    let b = d.add_param("b", "a * 2", None, "").unwrap();
    let c = d.add_param("c", "b + a", None, "").unwrap();
    let x = d.add_param("x", "7", None, "").unwrap();
    d.set_param_expr(a, "1 in").unwrap();
    assert_eq!(d.params().value(b), mm(50.8));
    let c_mm = d.params().value(c).unwrap();
    assert!((c_mm.value - 76.2).abs() < 1e-9, "{c_mm:?}");
    assert_eq!(d.params().value(x), Some(Quantity::dimensionless(7.0)));
    assert!(d.set_param_expr(a, "1 deg").is_err(), "a has unit mm");
    let rev = d.revision();
    d.set_param_expr(x, "7").unwrap();
    assert_eq!(d.revision(), rev, "no-op edit doesn't bump");
    rt(&d);
}

#[test]
fn dependent_failure_rejects_the_edit() {
    let mut d = design();
    let a = d.add_param("a", "2 mm", None, "").unwrap();
    let b = d.add_param("b", "10 mm / (a - 1 mm)", None, "").unwrap();
    let before = serde_json::to_string(&d).unwrap();
    // b would divide by zero.
    assert!(matches!(
        d.set_param_expr(a, "1 mm"),
        Err(ParamError::Expr(ExprError::Domain { func: "/", .. }))
    ));
    // b's unit-less sum would become a mismatch.
    assert!(matches!(
        d.set_param_expr(a, "1 s"),
        Err(ParamError::Expr(ExprError::UnitMismatch { .. }))
    ));
    assert_eq!(serde_json::to_string(&d).unwrap(), before);
    assert_eq!(d.params().value(b), Some(Quantity::dimensionless(10.0)));
}

#[test]
fn rename_rewrites_all_references() {
    let mut d = design();
    let a = d.add_param("a", "10 mm", None, "").unwrap();
    let ab = d.add_param("ab", "a+1", None, "").unwrap();
    let b = d.add_param("b", "a*2 +  a", None, "").unwrap();
    let c = d.add_param("c", "max(a, b) - ab", None, "").unwrap();
    let k = d.add_param("k", "5", None, "").unwrap();
    let rev = d.revision();
    d.rename_param(a, "width").unwrap();
    assert!(d.revision() > rev);
    let src = |id| d.params().get(id).unwrap().expr.0.clone();
    assert_eq!(src(ab), "width+1");
    assert_eq!(src(b), "width*2 +  width", "spacing kept");
    assert_eq!(src(c), "max(width, b) - ab", "`ab` untouched");
    assert_eq!(src(k), "5");
    assert_eq!(d.params().by_name("width"), Some(a));
    assert_eq!(d.params().by_name("a"), None);
    assert_eq!(d.params().value(c), mm(30.0 - 11.0));
    // The old name is free again; the new name resolves.
    d.add_param("a", "1", None, "").unwrap();
    d.set_param_expr(k, "width / 2").unwrap();
    assert_eq!(d.params().value(k), mm(5.0));
    // Rejections.
    assert_eq!(
        d.rename_param(b, "ab"),
        Err(ParamError::NameTaken("ab".into()))
    );
    assert_eq!(
        d.rename_param(b, "deg"),
        Err(ParamError::Expr(ExprError::ReservedName("deg".into())))
    );
    assert_eq!(
        d.rename_param(b, "b c"),
        Err(ParamError::Expr(ExprError::InvalidName("b c".into())))
    );
    assert_eq!(
        d.rename_param(ParamId(77), "q"),
        Err(ParamError::NotFound(ParamId(77)))
    );
    let rev = d.revision();
    d.rename_param(b, "b").unwrap();
    assert_eq!(d.revision(), rev);
    let back = rt(&d);
    assert_eq!(back.params().get(b).unwrap().expr.0, "width*2 +  width");
}

#[test]
fn rename_handles_unicode_and_repeated_references() {
    let mut d = design();
    let a = d.add_param("\u{e9}p", "2 mm", None, "").unwrap();
    let b = d
        .add_param("b", "\u{e9}p*\u{e9}p/\u{e9}p", None, "")
        .unwrap();
    d.rename_param(a, "thickness").unwrap();
    assert_eq!(
        d.params().get(b).unwrap().expr.0,
        "thickness*thickness/thickness"
    );
    assert_eq!(d.params().value(b), mm(2.0));
}

#[test]
fn delete_is_refused_while_referenced() {
    let mut d = design();
    let a = d.add_param("a", "1 mm", None, "").unwrap();
    let b = d.add_param("b", "a", None, "").unwrap();
    let c = d.add_param("c", "a + b", None, "").unwrap();
    assert_eq!(
        d.delete_param(a),
        Err(ParamError::InUse {
            id: a,
            by: vec![b, c]
        })
    );
    d.delete_param(c).unwrap();
    d.delete_param(b).unwrap();
    d.delete_param(a).unwrap();
    assert!(d.params().is_empty());
    assert_eq!(d.delete_param(a), Err(ParamError::NotFound(a)));
    let a2 = d.add_param("a", "2 mm", None, "").unwrap();
    assert_ne!(a2, a, "ids are never reused");
    rt(&d);
}

#[test]
fn meta_edits() {
    let mut d = design();
    let a = d.add_param("a", "1", None, "x").unwrap();
    let rev = d.revision();
    d.set_param_meta(a, Some("x".into()), Some(false)).unwrap();
    assert_eq!(d.revision(), rev, "no-op");
    d.set_param_meta(a, None, Some(true)).unwrap();
    d.set_param_meta(a, Some("note".into()), None).unwrap();
    let p = d.params().get(a).unwrap();
    assert!(p.favorite);
    assert_eq!(p.comment, "note");
    assert!(d.revision() > rev);
    assert_eq!(
        d.set_param_meta(ParamId(50), None, None),
        Err(ParamError::NotFound(ParamId(50)))
    );
    rt(&d);
}

#[test]
fn model_params_from_feature_exprs() {
    let mut d = design();
    let user = d.add_param("depth", "5 mm", None, "").unwrap();
    let f = FeatureId(d.alloc().unwrap());
    let mut e1 = Expr::Inline("20".into());
    let mut e2 = Expr::Inline("depth * 2".into());
    let mut e3 = Expr::Param(user);
    let mut e4 = Expr::Inline("45".into());
    let ids = d
        .add_model_params(
            f,
            vec![
                (&mut e1, L),
                (&mut e2, L),
                (&mut e3, L),
                (&mut e4, Dimension::ANGLE),
            ],
        )
        .unwrap();
    assert_eq!(ids.len(), 3);
    assert_eq!(e1, Expr::Param(ids[0]));
    assert_eq!(e2, Expr::Param(ids[1]));
    assert_eq!(e3, Expr::Param(user));
    assert_eq!(e4, Expr::Param(ids[2]));
    let p = d.params();
    let names: Vec<_> = ids
        .iter()
        .map(|&i| p.get(i).unwrap().name.clone())
        .collect();
    assert_eq!(names, ["d1", "d2", "d3"]);
    assert_eq!(p.get(ids[0]).unwrap().kind, ParamKind::Model { owner: f });
    assert_eq!(p.get(ids[0]).unwrap().unit, Some(Unit::Millimetre));
    assert_eq!(p.get(ids[2]).unwrap().unit, Some(Unit::Degree));
    assert_eq!(p.value(ids[0]), mm(20.0));
    assert_eq!(p.value(ids[1]), mm(10.0));
    assert_eq!(
        p.value(ids[2]),
        Some(Quantity::new(std::f64::consts::FRAC_PI_4, Dimension::ANGLE))
    );
    let prefs = d.meta().default_units.clone();
    assert_eq!(p.resolve(&e2, L, &prefs), Ok(Quantity::new(10.0, L)));
    assert!(p.resolve(&e2, Dimension::ANGLE, &prefs).is_err());
    assert_eq!(
        p.resolve(&Expr::Inline("1 cm".into()), L, &prefs),
        Ok(Quantity::new(10.0, L))
    );
    rt(&d);

    // Numbering continues above the highest d<n>; model params are editable like others.
    d.add_param("d7", "1 mm", None, "").unwrap();
    let mut e5 = Expr::Inline("d1 + d7".into());
    let more = d.add_model_params(f, vec![(&mut e5, L)]).unwrap();
    assert_eq!(d.params().get(more[0]).unwrap().name, "d8");
    assert_eq!(d.params().value(more[0]), mm(21.0));
    d.set_param_expr(ids[0], "30").unwrap();
    assert_eq!(d.params().value(more[0]), mm(31.0));
    rt(&d);
}

#[test]
fn model_params_use_the_document_length_unit() {
    let json = serde_json::to_string(&design())
        .unwrap()
        .replace("\"Millimetre\"", "\"Inch\"");
    let mut d: Design = serde_json::from_str(&json).unwrap();
    assert_eq!(d.meta().default_units.length, LengthUnit::Inch);
    let f = FeatureId(d.alloc().unwrap());
    let mut e = Expr::Inline("2".into());
    let ids = d.add_model_params(f, vec![(&mut e, L)]).unwrap();
    assert_eq!(d.params().get(ids[0]).unwrap().unit, Some(Unit::Inch));
    assert_eq!(d.params().value(ids[0]), mm(50.8));
    rt(&d);
}

#[test]
fn model_params_are_all_or_nothing() {
    let mut d = design();
    let f = FeatureId(d.alloc().unwrap());
    let before = serde_json::to_string(&d).unwrap();
    let rev = d.revision();
    let mut e1 = Expr::Inline("10".into());
    let mut e2 = Expr::Inline("10 deg".into());
    assert!(
        d.add_model_params(f, vec![(&mut e1, L), (&mut e2, L)])
            .is_err()
    );
    assert_eq!(e1, Expr::Inline("10".into()), "fields untouched on error");
    let mut e3 = Expr::Param(ParamId(999));
    assert_eq!(
        d.add_model_params(f, vec![(&mut e3, L)]),
        Err(ParamError::NotFound(ParamId(999)))
    );
    // Bad owners: unallocated, or an id owned by another entity type.
    let mut e4 = Expr::Inline("1".into());
    assert!(matches!(
        d.add_model_params(FeatureId(9999), vec![(&mut e4, L)]),
        Err(ParamError::Model(ModelError::NotFound { .. }))
    ));
    assert!(matches!(
        d.add_model_params(FeatureId(d.root().0), vec![(&mut e4, L)]),
        Err(ParamError::Model(ModelError::IdInUse { .. }))
    ));
    assert_eq!(serde_json::to_string(&d).unwrap(), before);
    assert_eq!(d.revision(), rev);
    // Nothing inline: nothing created, no bump.
    let mut e5 = Expr::Param(d.add_param("p", "1 mm", None, "").unwrap());
    let rev = d.revision();
    assert_eq!(d.add_model_params(f, vec![(&mut e5, L)]), Ok(vec![]));
    assert_eq!(d.revision(), rev);
}

#[test]
fn param_ids_are_owned() {
    let mut d = design();
    let a = d.add_param("a", "1", None, "").unwrap();
    let body = BodyId {
        origin: FeatureId(a.0),
        slot: 0,
    };
    let props = BodyProps {
        name: Some("x".into()),
        ..BodyProps::default()
    };
    assert!(matches!(
        d.set_body_props(body, props),
        Err(ModelError::IdInUse {
            owner: "parameter",
            ..
        })
    ));
    d.add_component(d.root(), "C", Transform::IDENTITY).unwrap();
    rt(&d);
}

#[test]
fn loading_rejects_bad_parameter_tables() {
    let mut d = design();
    let a = d.add_param("a", "1 mm", None, "").unwrap();
    let b = d.add_param("b", "a * 2", None, "").unwrap();
    let good = serde_json::to_string(&d).unwrap();
    let load = |s: &str| serde_json::from_str::<Design>(s);
    assert!(load(&good).is_ok());
    // Older file without the table loads with no parameters.
    let start = good.find(",\"params\"").unwrap();
    let old = format!("{}}}", &good[..start]);
    assert!(load(&old).unwrap().params().is_empty());
    let bad = [
        good.replace("\"a * 2\"", "\"b * 2\""),           // self cycle
        good.replace("\"1 mm\"", "\"b\""),                // a <-> b cycle
        good.replace("\"a * 2\"", "\"zz\""),              // unknown reference
        good.replace("\"name\":\"b\"", "\"name\":\"a\""), // duplicate name
        good.replace("\"name\":\"b\"", "\"name\":\"mm\""), // reserved
        good.replace("\"a * 2\"", "\"a +\""),             // syntax
        good.replace(&format!("[{},", b.0), "[999,"),     // id >= next_id
        good.replace("\"kind\":\"User\"", "\"kind\":{\"Model\":{\"owner\":1}}"), // owner is the root component
        good.replace("\"favorite\":false", "\"favorite\":false,\"extra\":1"),    // unknown field
    ];
    for (i, s) in bad.iter().enumerate() {
        assert_ne!(s, &good, "case {i} didn't change the file");
        assert!(load(s).is_err(), "case {i} loaded: {s}");
    }
    let _ = a;
}

#[test]
fn thousand_chained_params() {
    let mut d = design();
    let first = d.add_param("p0", "1 mm", None, "").unwrap();
    for i in 1..1000 {
        d.add_param(&format!("p{i}"), &format!("p{} + 1", i - 1), None, "")
            .unwrap();
    }
    let last = d.params().by_name("p999").unwrap();
    assert_eq!(d.params().value(last), mm(1000.0));
    d.set_param_expr(first, "2 mm").unwrap();
    assert_eq!(d.params().value(last), mm(1001.0));
    let back = rt(&d);
    assert_eq!(back.params().value(last), mm(1001.0));
}

#[test]
fn errors_are_diagnostics() {
    use amg_core::Diagnostic;
    assert!(ParamError::NameTaken("a".into()).hint().is_some());
    let e: ParamError = ExprError::Cycle(vec![]).into();
    assert!(e.hint().is_some());
}
