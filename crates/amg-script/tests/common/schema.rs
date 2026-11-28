//! Checks report JSON against the snapshots in `tests/schema/`.
//!
//! The snapshots are a small subset of JSON Schema (`type`, `properties`, `required`,
//! `additionalProperties: false`, `items`, `minItems`, `maxItems`, `enum`, `const`), so no
//! validator crate is needed. Any other keyword is refused, so a snapshot cannot silently
//! check less than it says.

use serde_json::Value;

const KEYWORDS: [&str; 11] = [
    "$schema",
    "title",
    "type",
    "properties",
    "required",
    "additionalProperties",
    "items",
    "minItems",
    "maxItems",
    "enum",
    "const",
];

/// The snapshot `tests/schema/<name>.json`.
pub fn load(name: &str) -> Value {
    let path = format!("{}/tests/schema/{name}.json", env!("CARGO_MANIFEST_DIR"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    serde_json::from_str(&text).unwrap_or_else(|e| panic!("{path}: {e}"))
}

/// Every mismatch between `value` and `schema`, with its path.
pub fn check(schema: &Value, value: &Value) -> Vec<String> {
    let mut out = Vec::new();
    walk(schema, value, "$", &mut out);
    out
}

fn type_matches(name: &str, v: &Value) -> bool {
    match name {
        "null" => v.is_null(),
        "boolean" => v.is_boolean(),
        "integer" => v.is_i64() || v.is_u64(),
        "number" => v.is_number(),
        "string" => v.is_string(),
        "array" => v.is_array(),
        "object" => v.is_object(),
        other => panic!("unknown type `{other}` in a schema"),
    }
}

fn walk(schema: &Value, v: &Value, at: &str, out: &mut Vec<String>) {
    let s = schema.as_object().expect("a schema is an object");
    for k in s.keys() {
        assert!(KEYWORDS.contains(&k.as_str()), "unsupported keyword `{k}`");
    }
    if let Some(t) = s.get("type") {
        let ok = match t {
            Value::String(t) => type_matches(t, v),
            Value::Array(ts) => ts.iter().any(|t| type_matches(t.as_str().unwrap(), v)),
            _ => panic!("`type` must be a string or an array"),
        };
        if !ok {
            out.push(format!("{at}: {v} is not of type {t}"));
            return;
        }
    }
    if let Some(c) = s.get("const")
        && c != v
    {
        out.push(format!("{at}: {v} is not {c}"));
    }
    if let Some(e) = s.get("enum")
        && !e.as_array().unwrap().contains(v)
    {
        out.push(format!("{at}: {v} is not one of {e}"));
    }
    if let Some(object) = v.as_object() {
        let props = s.get("properties").and_then(Value::as_object);
        let required = s.get("required").and_then(Value::as_array);
        for r in required.into_iter().flatten() {
            if !object.contains_key(r.as_str().unwrap()) {
                out.push(format!("{at}: missing field {r}"));
            }
        }
        for (k, x) in object {
            match props.and_then(|p| p.get(k)) {
                Some(sub) => walk(sub, x, &format!("{at}.{k}"), out),
                None if s.get("additionalProperties") == Some(&Value::Bool(false)) => {
                    out.push(format!("{at}: unexpected field `{k}`"));
                }
                None => {}
            }
        }
    }
    if let Some(array) = v.as_array() {
        let n = array.len() as u64;
        let min = s.get("minItems").and_then(Value::as_u64);
        let max = s.get("maxItems").and_then(Value::as_u64);
        if min.is_some_and(|m| n < m) || max.is_some_and(|m| n > m) {
            out.push(format!("{at}: {n} items"));
        }
        if let Some(items) = s.get("items") {
            for (i, x) in array.iter().enumerate() {
                walk(items, x, &format!("{at}[{i}]"), out);
            }
        }
    }
}
