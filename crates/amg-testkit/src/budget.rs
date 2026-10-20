//! The performance budgets of `docs/00-vision.md` §Performance budgets, one [`Budget`] per
//! table row (S00 §5, FND-08).
//!
//! Each budget's criterion bench lives in the crate that owns the scenario and is named
//! `budget/<id>`: a benchmark group [`BENCH_GROUP`] with one function per id. Until an owner
//! writes the real bench, `amg-testkit/benches/budgets.rs` holds a placeholder under that
//! name. When the real bench lands, its owner adds the id to `REAL` in that file (a lane-E
//! file, edited cross-lane and listed in S00's Interface changes); the placeholder then stops
//! running so two benches never share one criterion report.
//!
//! Adding a row to the vision table needs exactly one new [`BUDGETS`] entry whose
//! `description` is the row's *Scenario* text verbatim. A unit test parses the table and
//! fails until the two agree (row count, descriptions and limits).

use std::time::Duration;

/// Criterion group name for budget benches: a bench is `budget/<id>`.
pub const BENCH_GROUP: &str = "budget";

/// One row of the vision's performance budget table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Budget {
    /// Stable snake_case id; the bench is `budget/<id>`.
    pub id: &'static str,
    /// The vision table's *Scenario* text, verbatim.
    pub description: &'static str,
    /// The vision table's *Budget* column.
    pub limit: Limit,
}

/// A budget limit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Limit {
    /// The scenario must finish within this many milliseconds.
    MaxMillis(u64),
    /// The scenario must sustain at least this frame rate.
    MinFps(u32),
}

impl Limit {
    /// The longest allowed time for one run of the scenario: the limit itself for
    /// [`Limit::MaxMillis`], the frame time `1 s / fps` for [`Limit::MinFps`].
    pub fn max_duration(self) -> Duration {
        match self {
            Self::MaxMillis(ms) => Duration::from_millis(ms),
            Self::MinFps(fps) => Duration::from_secs(1) / fps.max(1),
        }
    }

    /// True if one run (a frame, for [`Limit::MinFps`]) taking `measured` meets the limit.
    pub fn is_met_by(self, measured: Duration) -> bool {
        measured <= self.max_duration()
    }
}

impl Budget {
    /// The criterion bench name, `budget/<id>`.
    pub fn bench_name(&self) -> String {
        format!("{BENCH_GROUP}/{}", self.id)
    }
}

/// One entry per row of `docs/00-vision.md` §Performance budgets, in table order.
pub const BUDGETS: &[Budget] = &[
    Budget {
        id: "cold_start",
        description: "Cold start to interactive viewport",
        limit: Limit::MaxMillis(2_000),
    },
    Budget {
        id: "viewport_orbit_5m_tris",
        description: "Viewport, 5 M-triangle mesh, orbit",
        limit: Limit::MinFps(60),
    },
    Budget {
        id: "recompute_50_features",
        description: "Recompute of a typical 50-feature part after editing an early sketch",
        limit: Limit::MaxMillis(500),
    },
    Budget {
        id: "mesh_boolean_2x1m_tris",
        description: "Mesh boolean, two 1 M-triangle meshes",
        limit: Limit::MaxMillis(1_000),
    },
    Budget {
        id: "fea_linear_static_500k_dof",
        description: "Linear static FEA, ~500 k DOF",
        limit: Limit::MaxMillis(30_000),
    },
    Budget {
        id: "topopt_1m_voxels_100_iter",
        description: "Topology optimization, 1 M voxels, 100 iterations",
        limit: Limit::MaxMillis(300_000),
    },
    Budget {
        id: "ui_max_block",
        description: "UI never blocks for more than",
        limit: Limit::MaxMillis(16),
    },
];

/// The budget with this id.
pub fn get(id: &str) -> Option<&'static Budget> {
    BUDGETS.iter().find(|b| b.id == id)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `(scenario, budget)` cells of the vision's budget table.
    fn vision_rows() -> Vec<(String, String)> {
        let path = crate::workspace_root().join("docs").join("00-vision.md");
        let text = std::fs::read_to_string(&path).expect("read docs/00-vision.md");
        let mut lines = text
            .lines()
            .skip_while(|l| !l.starts_with("## Performance budgets"))
            .skip(1)
            .take_while(|l| !l.starts_with("## "))
            .map(str::trim)
            .filter(|l| l.starts_with('|'));
        let header = lines.next().expect("budget table header");
        assert!(header.contains("Scenario"), "unexpected header {header:?}");
        let sep = lines.next().expect("budget table separator");
        assert!(sep.contains("---"), "unexpected separator {sep:?}");
        lines
            .map(|l| {
                let cells: Vec<&str> = l.trim_matches('|').split('|').map(str::trim).collect();
                assert_eq!(cells.len(), 2, "budget row {l:?} must have 2 cells");
                (cells[0].to_owned(), cells[1].to_owned())
            })
            .collect()
    }

    /// Parses `< 2 s`, `≤ 2 s`, `≥ 60 fps`, `< 5 min`, `16 ms (…)`. Panics on anything else:
    /// a wrong comparison direction, a non-positive or fractional value (after unit scaling).
    fn parse_limit(cell: &str) -> Limit {
        let (op, rest) = match cell.chars().next() {
            Some(c @ ('<' | '≤' | '≥' | '>')) => (Some(c), &cell[c.len_utf8()..]),
            _ => (None, cell),
        };
        let mut words = rest.split_whitespace();
        let value: f64 = words
            .next()
            .and_then(|v| v.parse().ok())
            .unwrap_or_else(|| panic!("no number in budget {cell:?}"));
        assert!(
            value.is_finite() && value > 0.0,
            "value must be > 0 in budget {cell:?}"
        );
        let exact = |scale: f64| {
            let v = value * scale;
            assert!(
                (v - v.round()).abs() < 1e-9,
                "budget {cell:?} is not a whole number"
            );
            v.round() as u64
        };
        let unit = words.next();
        match unit {
            Some("fps") => {
                assert!(
                    matches!(op, Some('≥' | '>')),
                    "fps budget {cell:?} must be `≥` or `>`"
                );
                Limit::MinFps(u32::try_from(exact(1.0)).expect("fps fits u32"))
            }
            Some("ms" | "s" | "min") => {
                assert!(
                    matches!(op, None | Some('<' | '≤')),
                    "time budget {cell:?} must be `<`, `≤` or have no operator"
                );
                let scale = match unit {
                    Some("ms") => 1.0,
                    Some("s") => 1_000.0,
                    _ => 60_000.0,
                };
                Limit::MaxMillis(exact(scale))
            }
            unit => panic!("unknown unit {unit:?} in budget {cell:?}"),
        }
    }

    #[test]
    fn budgets_match_vision_table() {
        let rows = vision_rows();
        assert_eq!(
            BUDGETS.len(),
            rows.len(),
            "BUDGETS must have one entry per row of docs/00-vision.md §Performance budgets"
        );
        for (b, (scenario, limit)) in BUDGETS.iter().zip(&rows) {
            assert_eq!(b.description, scenario, "budget {} (table order)", b.id);
            assert_eq!(b.limit, parse_limit(limit), "budget {} limit", b.id);
        }
    }

    #[test]
    fn ids_are_unique_snake_case() {
        for (i, b) in BUDGETS.iter().enumerate() {
            assert!(
                !b.id.is_empty()
                    && b.id
                        .bytes()
                        .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == b'_'),
                "id {:?} must be snake_case",
                b.id
            );
            assert!(
                BUDGETS[..i].iter().all(|o| o.id != b.id),
                "duplicate id {}",
                b.id
            );
            assert_eq!(get(b.id), Some(b));
        }
        assert_eq!(get("nope"), None);
    }

    #[test]
    fn parse_limit_formats() {
        assert_eq!(parse_limit("< 2 s"), Limit::MaxMillis(2_000));
        assert_eq!(parse_limit("≥ 60 fps"), Limit::MinFps(60));
        assert_eq!(parse_limit("< 5 min"), Limit::MaxMillis(300_000));
        assert_eq!(parse_limit("16 ms (heavier work)"), Limit::MaxMillis(16));
        assert_eq!(parse_limit("≤ 1.5 s"), Limit::MaxMillis(1_500));
    }

    #[test]
    #[should_panic(expected = "must be `≥`")]
    fn parse_limit_rejects_wrong_fps_direction() {
        parse_limit("< 60 fps");
    }

    #[test]
    #[should_panic(expected = "must be `<`")]
    fn parse_limit_rejects_wrong_time_direction() {
        parse_limit("≥ 2 s");
    }

    #[test]
    #[should_panic(expected = "whole number")]
    fn parse_limit_rejects_fractional_millis() {
        parse_limit("0.4 ms");
    }

    #[test]
    #[should_panic(expected = "whole number")]
    fn parse_limit_rejects_fractional_fps() {
        parse_limit("≥ 59.94 fps");
    }

    #[test]
    #[should_panic(expected = "> 0")]
    fn parse_limit_rejects_zero() {
        parse_limit("< 0 s");
    }

    #[test]
    fn limits_and_names() {
        assert_eq!(
            Limit::MinFps(60).max_duration(),
            Duration::from_nanos(16_666_666)
        );
        assert!(Limit::MaxMillis(16).is_met_by(Duration::from_millis(16)));
        assert!(!Limit::MaxMillis(16).is_met_by(Duration::from_micros(16_001)));
        assert!(Limit::MinFps(60).is_met_by(Duration::from_millis(16)));
        assert!(!Limit::MinFps(60).is_met_by(Duration::from_millis(17)));
        assert_eq!(BUDGETS[0].bench_name(), "budget/cold_start");
    }
}
