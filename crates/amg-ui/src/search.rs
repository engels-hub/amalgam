//! S-key command search (spec S11 §4).
//!
//! Our own subsequence matcher ([`fuzzy_score`]) with bonuses for word starts and adjacency,
//! plus a recency bonus for the last [`RECENT_CAP`] commands. It runs on every keystroke over a
//! prebuilt [`SearchIndex`] (lower-cased characters and word-start flags per command), so 1 000
//! commands cost well under 1 ms and nothing allocates per command.
//!
//! The popup ([`show`]) lists up to [`MAX_RESULTS`] rows. Enter runs the highlighted row if it
//! is enabled; the highlight starts on the first enabled row, so disabled commands are skipped
//! by default and never run. Disabled rows show their reason inline and as a tooltip.
//!
//! Not yet covered (arrive with their owners): `DialogRegistration` features (UI-05 registers
//! `feature.<kind>` commands, which search already lists) and document parameters (needs the
//! Parameters dialog).

use std::collections::VecDeque;

use egui::{Align2, Button, Key, Modifiers, Pos2, RichText};

use crate::app::CmdEnv;
use crate::command::{CmdId, CommandDef, Enablement, Placement};
use crate::registry::UiRegistry;
use crate::shortcut::ShortcutMap;

/// Rows shown in the popup.
pub const MAX_RESULTS: usize = 12;
/// Commands remembered for the recency bonus.
pub const RECENT_CAP: usize = 20;

const BASE: i32 = 10;
const ADJACENT: i32 = 15;
const WORD_START: i32 = 8;
const FIRST_CHAR: i32 = 10;
const PREFIX: i32 = 30;
const GAP_PENALTY: i32 = 3;
const GAP_CAP: i32 = 15;
const KEYWORD_PENALTY: i32 = 8;
const ID_PENALTY: i32 = 16;
/// Recency bonus of the most recent command; it falls by 2 per rank.
const RECENT_TOP: i32 = 2 * RECENT_CAP as i32;

/// Lower-cased characters of a text plus which of them start a word.
#[derive(Clone, Debug, Default)]
pub struct Prepared {
    chars: Vec<char>,
    starts: Vec<bool>,
}

impl Prepared {
    /// Prepares `text` for matching.
    pub fn new(text: &str) -> Prepared {
        let mut chars = Vec::with_capacity(text.len());
        let mut starts = Vec::with_capacity(text.len());
        let mut prev: Option<char> = None;
        for c in text.chars() {
            let start = match prev {
                None => true,
                Some(p) => !p.is_alphanumeric() || (p.is_lowercase() && c.is_uppercase()),
            };
            // Lower-casing can expand a char; keep the first (matching is per char).
            chars.push(c.to_lowercase().next().unwrap_or(c));
            starts.push(start);
            prev = Some(c);
        }
        Prepared { chars, starts }
    }

    fn len(&self) -> usize {
        self.chars.len()
    }
}

/// Subsequence score of `query` (already lower-cased, no spaces) in `text`, or `None` if
/// `query` is not a subsequence. Higher is better.
///
/// Matching characters score a base value, with bonuses for a word start, for following the
/// previous match directly, for the first character of the text, and for a contiguous prefix.
/// Gaps cost a little, and shorter texts win ties.
pub fn fuzzy_score(query: &[char], text: &Prepared) -> Option<i32> {
    let (n, m) = (query.len(), text.len());
    if n == 0 {
        return Some(0);
    }
    if n > m {
        return None;
    }
    // latest[k]: the last position query[k] may take so that query[k+1..] still fits after it.
    let mut latest = [0usize; 64];
    if n > latest.len() {
        return None;
    }
    let mut pos = m;
    for k in (0..n).rev() {
        loop {
            if pos == 0 {
                return None;
            }
            pos -= 1;
            if text.chars[pos] == query[k] {
                break;
            }
        }
        latest[k] = pos;
    }
    // Forward pass: take a word start inside the feasible window if there is one, else the
    // first occurrence.
    let mut score = 0;
    let mut prev: Option<usize> = None;
    let mut from = 0usize;
    let mut prefix = true;
    for k in 0..n {
        let hi = latest[k];
        let mut pick: Option<usize> = None;
        let mut first: Option<usize> = None;
        for (i, &c) in text.chars.iter().enumerate().take(hi + 1).skip(from) {
            if c != query[k] {
                continue;
            }
            if first.is_none() {
                first = Some(i);
            }
            if text.starts[i] {
                pick = Some(i);
                break;
            }
            // An adjacent continuation beats a later word start.
            if prev.is_some_and(|p| i == p + 1) {
                pick = Some(i);
                break;
            }
        }
        let i = pick.or(first)?;
        score += BASE;
        if text.starts[i] {
            score += WORD_START;
        }
        if i == 0 {
            score += FIRST_CHAR;
        }
        match prev {
            Some(p) if i == p + 1 => score += ADJACENT,
            Some(p) => score -= ((i - p - 1) as i32 * GAP_PENALTY).min(GAP_CAP),
            None => {}
        }
        if i != k {
            prefix = false;
        }
        prev = Some(i);
        from = i + 1;
    }
    if prefix {
        score += PREFIX;
    }
    // Shorter texts first among equal matches.
    score -= (m - n) as i32 / 2;
    Some(score)
}

#[derive(Clone, Debug)]
struct Entry {
    id: CmdId,
    label: Prepared,
    id_text: Prepared,
    keywords: Vec<Prepared>,
}

/// Prebuilt search data for a registry generation.
#[derive(Clone, Debug, Default)]
pub struct SearchIndex {
    generation: u64,
    entries: Vec<Entry>,
    /// Entry indices sorted by label (the empty-query order).
    by_label: Vec<usize>,
}

impl SearchIndex {
    /// Indexes every command of `reg`.
    pub fn build(reg: &UiRegistry) -> SearchIndex {
        let entries: Vec<Entry> = reg
            .commands()
            .map(|c| Entry {
                id: c.id,
                label: Prepared::new(&c.label),
                id_text: Prepared::new(c.id.0),
                keywords: c.keywords.iter().map(|k| Prepared::new(k)).collect(),
            })
            .collect();
        let mut by_label: Vec<usize> = (0..entries.len()).collect();
        by_label.sort_by(|&a, &b| {
            entries[a]
                .label
                .chars
                .cmp(&entries[b].label.chars)
                .then(entries[a].id.cmp(&entries[b].id))
        });
        SearchIndex {
            generation: reg.generation(),
            entries,
            by_label,
        }
    }

    /// The registry generation this index was built for.
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// Number of indexed commands.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// True without commands.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The best [`MAX_RESULTS`] commands for `query`, best first. Every whitespace-separated
    /// token of the query must match the label, a keyword or the id.
    pub fn search(&self, query: &str, recency: &Recency) -> Vec<SearchHit> {
        let tokens: Vec<Vec<char>> = query
            .split_whitespace()
            .map(|t| {
                t.chars()
                    .map(|c| c.to_lowercase().next().unwrap_or(c))
                    .collect()
            })
            .collect();
        if tokens.is_empty() {
            return self.recent_then_alphabetical(recency);
        }
        let mut top: Vec<SearchHit> = Vec::with_capacity(MAX_RESULTS + 1);
        for e in &self.entries {
            let Some(mut score) = score_entry(e, &tokens) else {
                continue;
            };
            if let Some(rank) = recency.rank(e.id) {
                score += RECENT_TOP - 2 * rank as i32;
            }
            let hit = SearchHit { id: e.id, score };
            // Keep `top` sorted best-first; ties by label order is applied via id (stable).
            let at = top.partition_point(|h| h.score > hit.score);
            if at < MAX_RESULTS {
                top.insert(at, hit);
                top.truncate(MAX_RESULTS);
            }
        }
        top
    }

    fn recent_then_alphabetical(&self, recency: &Recency) -> Vec<SearchHit> {
        let mut out: Vec<SearchHit> = Vec::with_capacity(MAX_RESULTS);
        for id in recency.iter() {
            if out.len() == MAX_RESULTS {
                return out;
            }
            if self.entries.iter().any(|e| e.id == id) {
                out.push(SearchHit {
                    id,
                    score: RECENT_TOP,
                });
            }
        }
        for &i in &self.by_label {
            if out.len() == MAX_RESULTS {
                break;
            }
            let id = self.entries[i].id;
            if !out.iter().any(|h| h.id == id) {
                out.push(SearchHit { id, score: 0 });
            }
        }
        out
    }
}

fn score_entry(e: &Entry, tokens: &[Vec<char>]) -> Option<i32> {
    let mut total = 0;
    for t in tokens {
        let mut best = fuzzy_score(t, &e.label);
        for k in &e.keywords {
            if let Some(s) = fuzzy_score(t, k) {
                best = best.max(Some(s - KEYWORD_PENALTY));
            }
        }
        if let Some(s) = fuzzy_score(t, &e.id_text) {
            best = best.max(Some(s - ID_PENALTY));
        }
        total += best?;
    }
    Some(total)
}

/// One search result.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SearchHit {
    /// The command.
    pub id: CmdId,
    /// Match score (recency included).
    pub score: i32,
}

/// The last [`RECENT_CAP`] commands run, most recent first.
#[derive(Clone, Debug, Default)]
pub struct Recency {
    ids: VecDeque<CmdId>,
}

impl Recency {
    /// An empty history.
    pub fn new() -> Recency {
        Recency::default()
    }

    /// Records a run: `id` becomes the most recent.
    pub fn record(&mut self, id: CmdId) {
        self.ids.retain(|i| *i != id);
        self.ids.push_front(id);
        self.ids.truncate(RECENT_CAP);
    }

    /// 0 for the most recent command, `None` if it is not in the history.
    pub fn rank(&self, id: CmdId) -> Option<usize> {
        self.ids.iter().position(|i| *i == id)
    }

    /// The history, most recent first.
    pub fn iter(&self) -> impl Iterator<Item = CmdId> + '_ {
        self.ids.iter().copied()
    }

    /// Number of remembered commands.
    pub fn len(&self) -> usize {
        self.ids.len()
    }

    /// True without history.
    pub fn is_empty(&self) -> bool {
        self.ids.is_empty()
    }
}

/// State of the search popup.
#[derive(Debug, Default)]
pub struct SearchState {
    open: bool,
    query: String,
    selected: usize,
    results: Vec<SearchHit>,
    index: Option<SearchIndex>,
    anchor: Option<Pos2>,
    dirty: bool,
    selection_pinned: bool,
}

impl SearchState {
    /// Opens the popup at `anchor` (the cursor), with an empty query.
    pub fn open(&mut self, anchor: Option<Pos2>) {
        self.open = true;
        self.query.clear();
        self.anchor = anchor;
        self.dirty = true;
        self.selected = 0;
        self.selection_pinned = false;
    }

    /// Closes the popup.
    pub fn close(&mut self) {
        self.open = false;
    }

    /// True while the popup is shown.
    pub fn is_open(&self) -> bool {
        self.open
    }

    /// The query text.
    pub fn query(&self) -> &str {
        &self.query
    }

    /// Replaces the query (tests; the popup edits it directly).
    pub fn set_query(&mut self, q: &str) {
        q.clone_into(&mut self.query);
        self.dirty = true;
        self.selection_pinned = false;
    }

    /// The current results, best first.
    pub fn results(&self) -> &[SearchHit] {
        &self.results
    }

    /// Index of the highlighted row.
    pub fn selected(&self) -> usize {
        self.selected
    }

    /// Recomputes the results if the query or the registry changed.
    pub fn refresh(&mut self, reg: &UiRegistry, recency: &Recency) {
        let stale = self
            .index
            .as_ref()
            .is_none_or(|i| i.generation() != reg.generation());
        if stale {
            self.index = Some(SearchIndex::build(reg));
            self.dirty = true;
        }
        if self.dirty {
            if let Some(index) = &self.index {
                self.results = index.search(&self.query, recency);
            }
            self.dirty = false;
            self.selected = 0;
            self.selection_pinned = false;
        }
    }

    /// The row Enter acts on, given each row's enablement: the highlighted row.
    fn first_enabled(&mut self, enabled: &[bool]) {
        if !self.selection_pinned {
            self.selected = enabled.iter().position(|e| *e).unwrap_or(0);
        }
    }
}

/// The menu/toolbar path of a command, for the search rows ("Mesh ▸ Modify", "File ▸ Export").
pub fn command_path(reg: &UiRegistry, def: &CommandDef) -> String {
    for p in &def.placements {
        match p {
            Placement::Toolbar { group, .. } => {
                let Some(g) = reg.group(*group) else {
                    continue;
                };
                let tab = reg.tab(g.tab).map_or("", |t| t.label);
                return format!("{tab} ▸ {}", g.label);
            }
            Placement::Menu { path, .. } => return path.replace('/', " ▸ "),
            Placement::SearchOnly => {}
        }
    }
    String::new()
}

/// Draws the popup and handles its keys. Returns the command to run, if the user picked one.
///
/// Call once per frame after the panels. Keys are consumed before the text field sees them.
pub(crate) fn show(
    ctx: &egui::Context,
    st: &mut SearchState,
    reg: &UiRegistry,
    recency: &Recency,
    env: &CmdEnv<'_>,
    shortcuts: &ShortcutMap,
) -> Option<CmdId> {
    if !st.open {
        return None;
    }
    st.refresh(reg, recency);

    let (esc, enter, down, up) = ctx.input_mut(|i| {
        (
            i.consume_key(Modifiers::NONE, Key::Escape),
            i.consume_key(Modifiers::NONE, Key::Enter),
            i.consume_key(Modifiers::NONE, Key::ArrowDown),
            i.consume_key(Modifiers::NONE, Key::ArrowUp),
        )
    });
    if esc {
        st.close();
        return None;
    }

    // Enablement of the visible rows only (lazy, spec §4).
    let rows: Vec<(CmdId, Enablement)> = st
        .results
        .iter()
        .filter_map(|h| reg.command(h.id).map(|d| (h.id, d.enablement(env))))
        .collect();
    let enabled: Vec<bool> = rows.iter().map(|(_, e)| e.is_enabled()).collect();
    st.first_enabled(&enabled);
    if !rows.is_empty() {
        if down {
            st.selected = (st.selected + 1).min(rows.len() - 1);
            st.selection_pinned = true;
        }
        if up {
            st.selected = st.selected.saturating_sub(1);
            st.selection_pinned = true;
        }
    }
    // Enter on a disabled row does nothing; the popup stays open.
    if enter
        && let Some((id, en)) = rows.get(st.selected)
        && en.is_enabled()
    {
        st.close();
        return Some(*id);
    }

    let anchor = st.anchor.unwrap_or_else(|| {
        let r = ctx.content_rect();
        Pos2::new(r.center().x - 180.0, r.top() + 90.0)
    });
    let edit_id = egui::Id::new("amg_search_text");
    let mut chosen: Option<CmdId> = None;
    let mut changed = false;
    egui::Area::new(egui::Id::new("amg_search_popup"))
        .order(egui::Order::Foreground)
        .fixed_pos(anchor)
        .pivot(Align2::LEFT_TOP)
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.set_min_width(380.0);
                let resp = ui.add(
                    egui::TextEdit::singleline(&mut st.query)
                        .id(edit_id)
                        .hint_text("Search commands…")
                        .desired_width(f32::INFINITY),
                );
                ui.memory_mut(|m| m.request_focus(edit_id));
                changed = resp.changed();
                ui.separator();
                if rows.is_empty() {
                    ui.weak("No matching command");
                }
                for (i, (id, en)) in rows.iter().enumerate() {
                    let Some(def) = reg.command(*id) else {
                        continue;
                    };
                    ui.horizontal(|ui| {
                        let btn = Button::selectable(i == st.selected, def.label.as_ref());
                        let r = ui.add_enabled(en.is_enabled(), btn);
                        let r = match en.reason() {
                            Some(reason) => r.on_disabled_hover_text(reason),
                            None if !def.tooltip.is_empty() => {
                                r.on_hover_text(def.tooltip.as_ref())
                            }
                            None => r,
                        };
                        if r.clicked() {
                            chosen = Some(*id);
                        }
                        let path = command_path(reg, def);
                        if !path.is_empty() {
                            ui.weak(path);
                        }
                        if let Some(sc) = shortcuts.display_of(*id) {
                            ui.weak(RichText::new(sc).monospace());
                        }
                        if let Some(reason) = en.reason() {
                            ui.weak(RichText::new(format!("({reason})")).italics());
                        }
                    });
                }
            });
        });
    if changed {
        st.dirty = true;
    }
    if let Some(id) = chosen {
        st.close();
        return Some(id);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command::{CmdAction, CommandDef};

    fn score(q: &str, text: &str) -> Option<i32> {
        let q: Vec<char> = q.chars().map(|c| c.to_ascii_lowercase()).collect();
        fuzzy_score(&q, &Prepared::new(text))
    }

    #[test]
    fn subsequence_required() {
        assert!(score("remsh", "Remesh").is_some());
        assert!(score("xyz", "Remesh").is_none());
        assert!(score("remeshh", "Remesh").is_none());
        assert_eq!(score("", "anything"), Some(0));
    }

    #[test]
    fn word_starts_and_adjacency_win() {
        // "fc" = Fillet Chamfer word starts vs a scattered match.
        let word = score("fc", "Fillet Chamfer").unwrap();
        let scattered = score("fc", "Soft Cube").unwrap();
        assert!(word > scattered, "{word} <= {scattered}");
        // contiguous beats gapped
        let adj = score("ext", "Extrude").unwrap();
        let gap = score("ext", "Edge Xylo Cut").unwrap();
        assert!(adj > gap);
        // prefix beats a mid-word match
        assert!(score("rem", "Remesh").unwrap() > score("rem", "Preemption").unwrap());
        // shorter text wins an otherwise equal match
        assert!(score("rem", "Remesh").unwrap() > score("rem", "Remesh Everything Now").unwrap());
    }

    #[test]
    fn camel_case_boundaries_are_word_starts() {
        let p = Prepared::new("RemeshMesh");
        assert!(p.starts[0] && p.starts[6]);
        assert!(!p.starts[1] && !p.starts[7]);
        let p = Prepared::new("mesh.remesh");
        assert!(p.starts[5], "after a dot");
    }

    fn cmd(id: &'static str, label: &'static str, kw: &'static [&'static str]) -> CommandDef {
        CommandDef::new(id, label, CmdAction::Run(|_| {})).with_keywords(kw)
    }

    fn reg_with(cmds: Vec<CommandDef>) -> UiRegistry {
        let mut reg = UiRegistry::default();
        for c in cmds {
            reg.add_command("test", c);
        }
        reg
    }

    #[test]
    fn remsh_finds_remesh_first() {
        let reg = reg_with(vec![
            cmd("t.shell", "Shell", &[]),
            cmd("t.remesh", "Remesh", &["retopo"]),
            cmd("t.render_mesh", "Render Mesh Shading", &[]),
            cmd("t.smooth", "Smooth", &[]),
        ]);
        let idx = SearchIndex::build(&reg);
        let hits = idx.search("remsh", &Recency::new());
        assert_eq!(hits.first().map(|h| h.id), Some(CmdId("t.remesh")));
        // Keywords and ids are searched too.
        let kw = idx.search("retopo", &Recency::new());
        assert_eq!(kw.first().map(|h| h.id), Some(CmdId("t.remesh")));
        let by_id = idx.search("t.smooth", &Recency::new());
        assert_eq!(by_id.first().map(|h| h.id), Some(CmdId("t.smooth")));
    }

    #[test]
    fn multi_token_query_needs_every_token() {
        let reg = reg_with(vec![
            cmd("a.one", "Mesh Remesh", &[]),
            cmd("a.two", "Mesh Smooth", &[]),
        ]);
        let idx = SearchIndex::build(&reg);
        let hits = idx.search("mesh smo", &Recency::new());
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].id, CmdId("a.two"));
    }

    #[test]
    fn recency_promotes_and_caps_at_twenty() {
        let reg = reg_with(vec![
            cmd("a.alpha", "Plane Align", &[]),
            cmd("a.apple", "Plane Alter", &[]),
        ]);
        let idx = SearchIndex::build(&reg);
        let mut rec = Recency::new();
        let cold = idx.search("pla", &rec);
        let first_cold = cold[0].id;
        let other = if first_cold == CmdId("a.alpha") {
            CmdId("a.apple")
        } else {
            CmdId("a.alpha")
        };
        rec.record(other);
        assert_eq!(idx.search("pla", &rec)[0].id, other);

        for i in 0..40 {
            rec.record(CmdId(["x0", "x1", "x2", "x3", "x4"][i % 5]));
        }
        assert_eq!(rec.len(), 6);
        let mut big = Recency::new();
        let ids: Vec<&'static str> = (0..30)
            .map(|i| &*Box::leak(format!("id{i}").into_boxed_str()))
            .collect();
        for id in &ids {
            big.record(CmdId(id));
        }
        assert_eq!(big.len(), RECENT_CAP);
        assert_eq!(big.rank(CmdId(ids[29])), Some(0));
        assert_eq!(big.rank(CmdId(ids[0])), None);
    }

    #[test]
    fn empty_query_lists_recent_first() {
        let reg = reg_with(vec![
            cmd("a.a", "Alpha", &[]),
            cmd("a.b", "Bravo", &[]),
            cmd("a.c", "Charlie", &[]),
        ]);
        let idx = SearchIndex::build(&reg);
        let mut rec = Recency::new();
        rec.record(CmdId("a.c"));
        let hits = idx.search("", &rec);
        let ids: Vec<_> = hits.iter().map(|h| h.id.0).collect();
        assert_eq!(ids, ["a.c", "a.a", "a.b"]);
    }

    #[test]
    fn results_are_capped_at_twelve() {
        let cmds: Vec<CommandDef> = (0..50)
            .map(|i| {
                let id: &'static str = Box::leak(format!("c.cmd{i}").into_boxed_str());
                cmd(id, "Create Thing", &[])
            })
            .collect();
        let idx = SearchIndex::build(&reg_with(cmds));
        assert_eq!(idx.search("thing", &Recency::new()).len(), MAX_RESULTS);
    }

    /// Done-when 5: 1 000 commands, < 1 ms per keystroke. Asserted only with
    /// `AMG_STRICT_TIMING=1` (use `--release`); the criterion bench reports the numbers.
    #[test]
    fn search_1000_commands_budget() {
        let cmds: Vec<CommandDef> = (0..1000)
            .map(|i| {
                let id: &'static str = Box::leak(format!("syn.cmd{i}").into_boxed_str());
                let label: &'static str = Box::leak(
                    format!(
                        "{} {} {}",
                        ["Create", "Modify", "Remesh", "Smooth"][i % 4],
                        i,
                        "Mesh"
                    )
                    .into_boxed_str(),
                );
                cmd(id, label, &["synthetic"])
            })
            .collect();
        let idx = SearchIndex::build(&reg_with(cmds));
        assert_eq!(idx.len(), 1000);
        let rec = Recency::new();
        let queries = ["r", "re", "rem", "remsh", "remsh mesh"];
        let mut worst = std::time::Duration::ZERO;
        for _ in 0..20 {
            for q in queries {
                let t = std::time::Instant::now();
                let hits = idx.search(q, &rec);
                worst = worst.max(t.elapsed());
                assert!(!hits.is_empty());
            }
        }
        eprintln!("worst keystroke: {worst:?}");
        if std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1") {
            assert!(worst < std::time::Duration::from_millis(1), "{worst:?}");
        }
    }
}
