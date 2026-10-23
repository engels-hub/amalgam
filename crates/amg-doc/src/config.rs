//! Configurations (S01 §10, DOC-10): named variants of one design that override parameter
//! expressions and feature suppression.
//!
//! - A [`Config`] stores *overrides* relative to the base design: a replacement expression for
//!   some parameters (`ParamId` to source text) and a forced suppression state for some
//!   features. Everything else is the base design.
//! - [`ConfigTable`] holds the configurations and which one is active. Activating one is the
//!   command [`Command::SetActiveConfig`](crate::Command::SetActiveConfig); it is an authored
//!   change like any other (undo, save/load, events, crash journal).
//! - Recompute does not look at the table. [`Design::effective`] applies the active
//!   configuration to a copy of the design and the engine runs on that copy. Cache keys are
//!   built from the values features read and from their inputs, so every configuration gets
//!   its own keys where its values differ (and shares entries where they agree), and switching
//!   back to a configuration whose outputs are still in the cache computes nothing.
//! - Overrides replace the expression of a parameter that exists in the base design; the
//!   parameter keeps its name, unit and dimension. Override expressions may reference any
//!   parameter by name and are evaluated together with the others, so overrides can depend on
//!   each other. Overriding a parameter to a value that does not evaluate is refused when the
//!   override is set; the command layer also refuses base edits that would break any
//!   configuration (rename rewrites override expressions like any other expression).
//! - Deleting a parameter or feature drops the overrides that name it.

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::sync::Arc;

use serde::{Deserialize, Serialize};

use crate::expr;
use crate::id::{ConfigId, FeatureId, ParamId};
use crate::model::{Design, ModelError};
use crate::param::{ParamError, rewrite};
use crate::pcoll::{PMap, entries};

/// One configuration: overrides on top of the base design.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// Display name; unique among the configurations.
    pub name: String,
    /// Replacement expression sources, by parameter.
    pub params: BTreeMap<ParamId, String>,
    /// Forced suppression state, by feature (`true`: suppressed, `false`: active), whatever the
    /// base design says.
    pub suppressed: BTreeMap<FeatureId, bool>,
}

/// All configurations of a design and the active one.
///
/// `clone()` is O(1) (persistent map).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConfigTable {
    #[serde(
        serialize_with = "entries::serialize",
        deserialize_with = "entries::deserialize"
    )]
    configs: PMap<ConfigId, Arc<Config>>,
    active: Option<ConfigId>,
}

impl ConfigTable {
    /// The configuration `id`.
    pub fn get(&self, id: ConfigId) -> Option<&Config> {
        self.configs.get(&id).map(|c| &**c)
    }

    /// All configurations in id order.
    pub fn iter(&self) -> impl Iterator<Item = (ConfigId, &Config)> + '_ {
        self.configs.iter().map(|(&id, c)| (id, &**c))
    }

    /// The configuration called `name`.
    pub fn by_name(&self, name: &str) -> Option<ConfigId> {
        self.iter().find(|(_, c)| c.name == name).map(|(id, _)| id)
    }

    /// The active configuration (`None`: the base design).
    pub fn active(&self) -> Option<ConfigId> {
        self.active
    }

    /// Number of configurations.
    pub fn len(&self) -> usize {
        self.configs.len()
    }

    /// True if there are no configurations.
    pub fn is_empty(&self) -> bool {
        self.configs.is_empty()
    }
}

fn not_found(id: ConfigId) -> ModelError {
    ModelError::NotFound {
        what: "configuration",
        id: id.0,
    }
}

fn invalid(field: &'static str, msg: impl Into<String>) -> ModelError {
    ModelError::InvalidValue {
        field,
        msg: msg.into(),
    }
}

impl Design {
    /// The configurations.
    pub fn configs(&self) -> &ConfigTable {
        &self.configs
    }

    /// The active configuration (`None`: the base design).
    pub fn active_config(&self) -> Option<ConfigId> {
        self.configs.active
    }

    /// The active configuration itself.
    pub fn active_config_data(&self) -> Option<&Config> {
        self.configs.active.and_then(|c| self.configs.get(c))
    }

    /// The design as the active configuration defines it: its parameter expressions and
    /// suppression flags applied. This is what recompute evaluates. Borrowed (free) when no
    /// configuration is active. Edit the base design, not this copy.
    pub fn effective(&self) -> Cow<'_, Design> {
        let Some(id) = self.configs.active else {
            return Cow::Borrowed(self);
        };
        match self.with_config(id) {
            Ok(d) => Cow::Owned(d),
            Err(e) => {
                // Every edit path checks configurations, so this means a bug, not user input.
                debug_assert!(false, "active configuration {id} does not apply: {e}");
                Cow::Borrowed(self)
            }
        }
    }

    /// A copy of the design with configuration `id` applied (whether or not it is active).
    /// The copy keeps this design's [`revision`](Design::revision).
    pub fn with_config(&self, id: ConfigId) -> Result<Design, ParamError> {
        let cfg = self.configs.get(id).ok_or_else(|| not_found(id))?;
        let mut d = self.clone();
        if !cfg.params.is_empty() {
            d.params = self
                .params
                .with_exprs(&cfg.params, &self.meta.default_units)?;
        }
        for (&f, &s) in &cfg.suppressed {
            d.timeline.force_suppressed(f, s);
        }
        Ok(d)
    }

    /// Adds an empty configuration.
    pub fn add_config(&mut self, name: &str) -> Result<ConfigId, ModelError> {
        self.check_config_name(name, None)?;
        self.ensure_ids(1)?;
        let id = ConfigId(self.alloc()?);
        let cfg = Config {
            name: name.to_owned(),
            ..Config::default()
        };
        self.configs.configs.insert(id, Arc::new(cfg));
        self.bump();
        Ok(id)
    }

    /// Renames a configuration.
    pub fn rename_config(&mut self, id: ConfigId, name: &str) -> Result<(), ModelError> {
        let cfg = self.configs.get(id).ok_or_else(|| not_found(id))?;
        if cfg.name == name {
            return Ok(());
        }
        self.check_config_name(name, Some(id))?;
        self.edit_config(id, |c| name.clone_into(&mut c.name))
    }

    /// Deletes a configuration; if it was active, the base design becomes active.
    pub fn delete_config(&mut self, id: ConfigId) -> Result<(), ModelError> {
        if self.configs.configs.remove(&id).is_none() {
            return Err(not_found(id));
        }
        if self.configs.active == Some(id) {
            self.configs.active = None;
        }
        self.bump();
        Ok(())
    }

    /// Overrides (`Some(expr)`) or restores (`None`) the expression of a parameter in a
    /// configuration. Fails, changing nothing, if the configuration would no longer evaluate.
    pub fn set_config_param(
        &mut self,
        config: ConfigId,
        param: ParamId,
        expr: Option<&str>,
    ) -> Result<(), ParamError> {
        let cfg = self.configs.get(config).ok_or_else(|| not_found(config))?;
        if self.params.get(param).is_none() {
            return Err(ParamError::NotFound(param));
        }
        if cfg.params.get(&param).map(String::as_str) == expr {
            return Ok(());
        }
        let mut next = cfg.clone();
        match expr {
            Some(e) => next.params.insert(param, e.to_owned()),
            None => next.params.remove(&param),
        };
        // Evaluate the would-be configuration before touching the design.
        self.params
            .with_exprs(&next.params, &self.meta.default_units)?;
        self.configs.configs.insert(config, Arc::new(next));
        self.bump();
        Ok(())
    }

    /// Forces (`Some`) or stops forcing (`None`) the suppression state of a feature in a
    /// configuration.
    pub fn set_config_suppressed(
        &mut self,
        config: ConfigId,
        feature: FeatureId,
        suppressed: Option<bool>,
    ) -> Result<(), ModelError> {
        let cfg = self.configs.get(config).ok_or_else(|| not_found(config))?;
        if !self.timeline.contains(feature) {
            return Err(ModelError::NotFound {
                what: "feature",
                id: feature.0,
            });
        }
        if cfg.suppressed.get(&feature).copied() == suppressed {
            return Ok(());
        }
        self.edit_config(config, |c| {
            match suppressed {
                Some(s) => c.suppressed.insert(feature, s),
                None => c.suppressed.remove(&feature),
            };
        })
    }

    /// Activates a configuration (`None`: the base design).
    pub fn set_active_config(&mut self, config: Option<ConfigId>) -> Result<(), ModelError> {
        if let Some(c) = config
            && self.configs.get(c).is_none()
        {
            return Err(not_found(c));
        }
        if self.configs.active != config {
            self.configs.active = config;
            self.bump();
        }
        Ok(())
    }

    fn edit_config(&mut self, id: ConfigId, f: impl FnOnce(&mut Config)) -> Result<(), ModelError> {
        let c = self
            .configs
            .configs
            .get_mut(&id)
            .ok_or_else(|| not_found(id))?;
        f(Arc::make_mut(c));
        self.bump();
        Ok(())
    }

    fn check_config_name(&self, name: &str, this: Option<ConfigId>) -> Result<(), ModelError> {
        if name.trim().is_empty() {
            return Err(invalid("configuration name", "must not be empty"));
        }
        match self.configs.by_name(name) {
            Some(other) if Some(other) != this => Err(invalid(
                "configuration name",
                format!("`{name}` is already in use"),
            )),
            _ => Ok(()),
        }
    }

    /// Checks that every configuration applies: all of them evaluate. `Err` names the first
    /// that does not.
    pub(crate) fn check_configs(&self) -> Result<(), String> {
        for (id, cfg) in self.configs.iter() {
            if let Err(e) = self.with_config(id) {
                return Err(format!("configuration `{}` ({id}): {e}", cfg.name));
            }
        }
        Ok(())
    }

    /// Drops the overrides of parameters and features that no longer exist (and nothing else).
    /// Does not bump: it runs inside the edit that deleted them.
    pub(crate) fn prune_configs(&mut self) {
        if self.configs.is_empty() {
            return;
        }
        let ids: Vec<ConfigId> = self.configs.configs.keys().copied().collect();
        for id in ids {
            let Some(cfg) = self.configs.configs.get(&id) else {
                continue;
            };
            let stale = cfg.params.keys().any(|p| self.params.get(*p).is_none())
                || cfg.suppressed.keys().any(|f| !self.timeline.contains(*f));
            if stale {
                let mut next = (**cfg).clone();
                next.params.retain(|p, _| self.params.get(*p).is_some());
                next.suppressed.retain(|f, _| self.timeline.contains(*f));
                self.configs.configs.insert(id, Arc::new(next));
            }
        }
    }

    /// Rewrites references to parameter `old` in override expressions after a rename, so
    /// overrides keep their meaning. Expressions that don't parse are left alone (the
    /// configuration check reports them).
    pub(crate) fn rename_in_configs(&mut self, old: &str, new: &str) {
        let ids: Vec<ConfigId> = self.configs.configs.keys().copied().collect();
        for id in ids {
            let Some(cfg) = self.configs.configs.get(&id) else {
                continue;
            };
            let mut next = (**cfg).clone();
            let mut changed = false;
            for src in next.params.values_mut() {
                let Ok(ast) = expr::parse(src) else { continue };
                let out = rewrite(src, &ast, old, new);
                if out != *src {
                    *src = out;
                    changed = true;
                }
            }
            if changed {
                self.configs.configs.insert(id, Arc::new(next));
            }
        }
    }

    /// Structural checks of a loaded table (ids exist, names are sane); evaluation is checked
    /// by [`Design::check_configs`].
    pub(crate) fn validate_configs(&self) -> Result<(), String> {
        let mut names = std::collections::BTreeSet::new();
        for (id, c) in self.configs.iter() {
            if c.name.trim().is_empty() {
                return Err(format!("{id} has an empty name"));
            }
            if !names.insert(c.name.as_str()) {
                return Err(format!("two configurations are called `{}`", c.name));
            }
            if let Some(p) = c.params.keys().find(|p| self.params.get(**p).is_none()) {
                return Err(format!("{id} overrides missing {p}"));
            }
            if let Some(f) = c.suppressed.keys().find(|f| !self.timeline.contains(**f)) {
                return Err(format!("{id} overrides suppression of missing {f}"));
            }
        }
        if let Some(a) = self.configs.active
            && self.configs.get(a).is_none()
        {
            return Err(format!("active configuration {a} does not exist"));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_json_shape_is_stable() {
        let mut d = Design::new("D");
        let p = d.add_param("w", "10 mm", None, "").unwrap();
        let c = d.add_config("Wide").unwrap();
        d.set_config_param(c, p, Some("20 mm")).unwrap();
        d.set_active_config(Some(c)).unwrap();
        let j = serde_json::to_value(d.configs()).unwrap();
        assert_eq!(
            j,
            serde_json::json!({
                "configs": [[c.0, {"name": "Wide", "params": {p.0.to_string(): "20 mm"},
                                   "suppressed": {}}]],
                "active": c.0,
            })
        );
        let back: ConfigTable = serde_json::from_value(j).unwrap();
        assert_eq!(&back, d.configs());
    }
}
