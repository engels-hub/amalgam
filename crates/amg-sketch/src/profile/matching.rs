//! Profile matching across sketch edits (SK-15).
//!
//! A [`ProfileId`] hashes every boundary edge key, so it survives dimension edits and
//! unrelated edits but changes when a region gains a hole, is crossed by a curve or loses a
//! boundary curve. [`ProfileSet::match_previous`] maps the old ids to the new ones in two
//! stages: identical ids first ([`ProfileMatch::Same`]), then a one-to-one greedy assignment
//! of the rest by [`ProfileFingerprint`] similarity ([`ProfileMatch::Fingerprint`]).
//!
//! The score is a weighted sum in `[0, 1]`:
//! outer-loop source-set Jaccard 0.3 (the cap-name key of the persistent-naming note, §5.4),
//! all-source Jaccard 0.1, area ratio 0.3, centroid proximity 0.3 (distance relative to the
//! old bounding-box diagonal). Matches below [`MIN_SCORE`] are rejected. A region redrawn with
//! fresh entities (nothing shared) still scores 0.6 when its area and place are unchanged; a
//! consumer must report any `Fingerprint` match as a warning, never as an exact one.
//!
//! The assignment is deterministic: candidate pairs are processed by descending score, ties by
//! old id then new id.

use std::collections::BTreeMap;

use super::{ProfileFingerprint, ProfileId, ProfileSet};
use crate::ids::EntityId;

/// Lowest accepted fingerprint score.
pub const MIN_SCORE: f64 = 0.5;

/// Result of matching one old profile against a new [`ProfileSet`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ProfileMatch {
    /// The same id exists in the new set (same boundary edge keys).
    Same(ProfileId),
    /// The best fingerprint match; `score` is in `[MIN_SCORE, 1]`.
    Fingerprint {
        /// Successor profile.
        new: ProfileId,
        /// Similarity, higher is better.
        score: f64,
    },
    /// No successor.
    Lost,
}

fn jaccard(a: &[EntityId], b: &[EntityId]) -> f64 {
    let (mut i, mut j, mut common) = (0, 0, 0usize);
    while i < a.len() && j < b.len() {
        match a[i].cmp(&b[j]) {
            std::cmp::Ordering::Less => i += 1,
            std::cmp::Ordering::Greater => j += 1,
            std::cmp::Ordering::Equal => {
                common += 1;
                i += 1;
                j += 1;
            }
        }
    }
    let union = a.len() + b.len() - common;
    if union == 0 {
        1.0
    } else {
        common as f64 / union as f64
    }
}

/// Similarity of two fingerprints in `[0, 1]` (1 = identical data).
pub(crate) fn score(old: &ProfileFingerprint, new: &ProfileFingerprint) -> f64 {
    let outer = jaccard(&old.outer_sources, &new.outer_sources);
    let all = jaccard(&old.sources, &new.sources);
    let (a, b) = (old.area.abs(), new.area.abs());
    let area = if a.max(b) > 0.0 {
        a.min(b) / a.max(b)
    } else {
        1.0
    };
    let diag = (old.bbox.max - old.bbox.min).length().max(1e-9);
    let near = (1.0 - old.centroid.distance(new.centroid) / diag).clamp(0.0, 1.0);
    0.3 * outer + 0.1 * all + 0.3 * area + 0.3 * near
}

impl ProfileSet {
    /// Best match for `fp` without a previous profile set (pure; for `EntityRef::Profile`
    /// resolution by fingerprint). Never returns [`ProfileMatch::Same`]. Ties go to the
    /// smaller id.
    pub fn match_fingerprint(&self, fp: &ProfileFingerprint) -> ProfileMatch {
        let mut best: Option<(f64, ProfileId)> = None;
        for p in &self.profiles {
            let s = score(fp, &p.fingerprint);
            if s >= MIN_SCORE && best.is_none_or(|(bs, _)| s > bs) {
                best = Some((s, p.id));
            }
        }
        match best {
            Some((score, new)) => ProfileMatch::Fingerprint { new, score },
            None => ProfileMatch::Lost,
        }
    }

    /// Maps every profile of `previous` to its successor in `self`: identical ids are
    /// [`ProfileMatch::Same`]; the remaining old profiles are assigned one-to-one to the
    /// remaining new ones by fingerprint score (greedy, best first), else
    /// [`ProfileMatch::Lost`]. A split region maps to its best-matching piece, a merge maps
    /// the best-matching old region to the merged one.
    pub fn match_previous(&self, previous: &ProfileSet) -> BTreeMap<ProfileId, ProfileMatch> {
        let mut out = BTreeMap::new();
        let mut old_left = Vec::new();
        for o in &previous.profiles {
            if self.get(o.id).is_some() {
                out.insert(o.id, ProfileMatch::Same(o.id));
            } else {
                old_left.push(o);
            }
        }
        let new_left: Vec<_> = self
            .profiles
            .iter()
            .filter(|n| previous.get(n.id).is_none())
            .collect();
        let mut pairs: Vec<(f64, ProfileId, ProfileId)> = Vec::new();
        for o in &old_left {
            for n in &new_left {
                let s = score(&o.fingerprint, &n.fingerprint);
                if s >= MIN_SCORE {
                    pairs.push((s, o.id, n.id));
                }
            }
        }
        pairs.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
        let mut taken = std::collections::BTreeSet::new();
        for (s, o, n) in pairs {
            if out.contains_key(&o) || !taken.insert(n) {
                continue;
            }
            out.insert(o, ProfileMatch::Fingerprint { new: n, score: s });
        }
        for o in old_left {
            out.entry(o.id).or_insert(ProfileMatch::Lost);
        }
        out
    }
}
