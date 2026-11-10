//! BREP-12 Done-when 4: every shim function carries a `// CATEGORY: R|N|C|G` comment (S02 §2,
//! occt-integration.md §4.3), and every function the bridges declare is one of them.
//!
//! Plain text checks, so they run with and without the `occt` feature.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::collections::BTreeMap;
use std::path::Path;

/// Shim headers and the bridge modules whose `extern "C++"` functions they declare.
const SHIMS: [(&str, &str); 2] = [
    ("cpp/occt_shim.hxx", "src/bridge.rs"),
    ("cpp/occt_fixtures.hxx", "src/fixtures.rs"),
];

const CATEGORIES: [char; 4] = ['R', 'N', 'C', 'G'];

/// A function declared in a shim header.
#[derive(Debug)]
struct ShimFn {
    line: usize,
    /// `None` when the comment block above it has no valid `CATEGORY:` tag.
    category: Option<char>,
}

/// Parses the free functions declared inside `namespace amg::occt` of a shim header, with the
/// category tag of the comment block right above each.
fn shim_functions(header: &str) -> BTreeMap<String, ShimFn> {
    let mut out = BTreeMap::new();
    let mut in_namespace = false;
    let mut depth = 0usize;
    let mut comments: Vec<&str> = Vec::new();
    let mut in_declaration = false;
    for (n, raw) in header.lines().enumerate() {
        let line = raw.trim();
        if !in_namespace {
            if line.starts_with("namespace amg::occt") && line.ends_with('{') {
                in_namespace = true;
                depth = 0;
            }
            continue;
        }
        if line.starts_with("//") {
            comments.push(line);
            continue;
        }
        if depth == 0 && line.starts_with('}') {
            // End of the namespace.
            in_namespace = false;
            comments.clear();
            continue;
        }
        let at_top = depth == 0;
        if at_top && !in_declaration && is_function_start(line) {
            let name = function_name(line);
            out.insert(
                name,
                ShimFn {
                    line: n + 1,
                    category: category_of(&comments),
                },
            );
            in_declaration = !line.ends_with(';') && !line.ends_with('{');
        } else if in_declaration && (line.ends_with(';') || line.ends_with('{')) {
            in_declaration = false;
        }
        depth += line.matches('{').count();
        depth = depth.saturating_sub(line.matches('}').count());
        comments.clear();
    }
    out
}

fn is_function_start(line: &str) -> bool {
    const NOT_FUNCTIONS: [&str; 8] = [
        "#",
        "class ",
        "struct ",
        "using ",
        "template",
        "namespace",
        "enum ",
        "public:",
    ];
    !line.is_empty()
        && line.contains('(')
        && !NOT_FUNCTIONS.iter().any(|prefix| line.starts_with(prefix))
}

/// The identifier right before the first `(`.
fn function_name(line: &str) -> String {
    let head = &line[..line.find('(').unwrap()];
    head.rsplit(|c: char| !(c.is_alphanumeric() || c == '_'))
        .next()
        .unwrap()
        .to_owned()
}

fn category_of(comments: &[&str]) -> Option<char> {
    comments.iter().find_map(|comment| {
        let rest = comment.trim_start_matches('/').trim();
        let tag = rest.strip_prefix("CATEGORY:")?.trim_start();
        let category = tag.chars().next()?;
        let reason_ok = tag[category.len_utf8()..]
            .chars()
            .next()
            .is_none_or(|c| c == ' ');
        (CATEGORIES.contains(&category) && reason_ok).then_some(category)
    })
}

/// Names of the functions in the `unsafe extern "C++"` blocks of a bridge module.
fn bridged_functions(bridge: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut in_block = false;
    for line in bridge.lines().map(str::trim) {
        if line.starts_with("unsafe extern \"C++\"") {
            in_block = true;
        } else if in_block && line == "}" {
            in_block = false;
        } else if in_block && let Some(rest) = line.strip_prefix("fn ") {
            out.push(rest[..rest.find('(').unwrap()].trim().to_owned());
        }
    }
    out
}

/// Every lint failure of one header/bridge pair; empty when clean.
fn lint(header: &str, bridge: &str) -> Vec<String> {
    let functions = shim_functions(header);
    let mut errors: Vec<String> = functions
        .iter()
        .filter(|(_, f)| f.category.is_none())
        .map(|(name, f)| {
            format!(
                "line {}: `{name}` has no `// CATEGORY: R|N|C|G <reason>` comment",
                f.line
            )
        })
        .collect();
    for name in bridged_functions(bridge) {
        if !functions.contains_key(&name) {
            errors.push(format!(
                "`{name}` is bridged but not declared in the shim header"
            ));
        }
    }
    errors
}

fn read(relative: &str) -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(relative);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

#[test]
fn every_shim_function_has_a_category() {
    let mut total = 0;
    for (header, bridge) in SHIMS {
        let header_text = read(header);
        let errors = lint(&header_text, &read(bridge));
        assert!(errors.is_empty(), "{header}:\n{}", errors.join("\n"));
        total += shim_functions(&header_text).len();
    }
    // Guards against a parser that silently finds nothing.
    assert!(total >= 30, "only {total} shim functions found");
}

#[test]
fn lint_fails_without_a_category() {
    let header = "\
namespace amg::occt {

// CATEGORY: R
double volume(const OcShape &shape);

// Builds things.
std::unique_ptr<OcShape> make_thing(double a,
                                    double b);

} // namespace amg::occt
";
    let bridge = "unsafe extern \"C++\" {\n fn volume(s: &OcShape) -> Result<f64>;\n}\n";
    let errors = lint(header, bridge);
    assert_eq!(errors.len(), 1, "{errors:?}");
    assert!(errors[0].contains("make_thing"), "{errors:?}");
}

#[test]
fn lint_rejects_unknown_categories_and_missing_declarations() {
    let header = "\
namespace amg::occt {
// CATEGORY: X (not a category)
void a();
// CATEGORY: RN
void b();
// CATEGORY: N (non-destructive)
void c();
class Skipped {
public:
  void member(int x);
};
}
";
    let bridge = "unsafe extern \"C++\" {\n fn c() -> Result<()>;\n fn d() -> Result<()>;\n}\n";
    let errors = lint(header, bridge);
    assert_eq!(errors.len(), 3, "{errors:?}");
    assert!(errors.iter().any(|e| e.contains("`a`")));
    assert!(errors.iter().any(|e| e.contains("`b`")));
    assert!(errors.iter().any(|e| e.contains("`d` is bridged")));
}
