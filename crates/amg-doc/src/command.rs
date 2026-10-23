//! Commands (S01 §6): the only way to mutate a design.
//!
//! [`Command`] is a closed, serializable enum. [`apply_command`] runs one command against a
//! [`Design`] atomically: either it succeeds, or the design is unchanged. Undo/redo and
//! transactions live in [`crate::history`], which calls this module.
//!
//! Domain lanes add feature *types*, never commands.

use amg_core::Transform;
use serde::{Deserialize, Serialize};

use crate::direct::BaseFeature;
use crate::expr::ExprError;
use crate::feature::FeatureBox;
use crate::id::{
    BodyId, ComponentId, ConfigId, ExtKey, FeatureId, GroupId, OccurrenceId, ParamId, Revision,
};
use crate::model::{BodyProps, Design, DesignMode, ExtRecord, ModelError, UnitPrefs};
use crate::param::{Expr, ParamError, UnitSpec};

/// One user-level edit. Serializes as externally tagged JSON (`{"AddParam": {..}}`), the
/// form the crash journal, macros and the Python API share.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Command {
    /// Inserts a feature (`at: None`: at the rollback marker).
    AddFeature {
        /// Insert index.
        at: Option<usize>,
        /// The feature.
        feature: FeatureBox,
        /// Component the feature is created in.
        context: ComponentId,
        /// Display name override.
        name: Option<String>,
    },
    /// Replaces a feature's definition (same kind).
    EditFeature {
        /// The feature.
        id: FeatureId,
        /// New definition.
        feature: FeatureBox,
    },
    /// Deletes features (all or none). Model parameters they own become user parameters.
    DeleteFeatures {
        /// The features.
        ids: Vec<FeatureId>,
    },
    /// Moves a feature to final index `to`.
    MoveFeature {
        /// The feature.
        id: FeatureId,
        /// Final index.
        to: usize,
    },
    /// Moves the rollback marker (number of computed nodes).
    SetMarker {
        /// New marker position.
        index: usize,
    },
    /// Suppresses or unsuppresses features.
    SetSuppressed {
        /// The features.
        ids: Vec<FeatureId>,
        /// New state.
        suppressed: bool,
    },
    /// Sets or clears a feature's display name.
    RenameFeature {
        /// The feature.
        id: FeatureId,
        /// New name (`None`: default).
        name: Option<String>,
    },
    /// Groups adjacent ungrouped features.
    GroupFeatures {
        /// The features.
        ids: Vec<FeatureId>,
        /// Group name.
        name: String,
    },
    /// Dissolves a group.
    Ungroup {
        /// The group.
        group: GroupId,
    },
    /// Renames and/or collapses a group (`None` keeps the current value).
    SetGroup {
        /// The group.
        group: GroupId,
        /// New name.
        name: Option<String>,
        /// New collapsed state.
        collapsed: Option<bool>,
    },
    /// Adds a user parameter.
    AddParam {
        /// Unique name.
        name: String,
        /// Expression source.
        expr: String,
        /// Display unit and dimension.
        unit: Option<UnitSpec>,
        /// Free text.
        comment: String,
    },
    /// Replaces a parameter's expression.
    SetParamExpr {
        /// The parameter.
        id: ParamId,
        /// New source.
        expr: String,
    },
    /// Renames a parameter, rewriting referencing expressions.
    RenameParam {
        /// The parameter.
        id: ParamId,
        /// New name.
        name: String,
    },
    /// Changes comment and/or favourite flag (`None` keeps the current value).
    SetParamMeta {
        /// The parameter.
        id: ParamId,
        /// New comment.
        comment: Option<String>,
        /// New favourite flag.
        favorite: Option<bool>,
    },
    /// Deletes a parameter that nothing references.
    DeleteParam {
        /// The parameter.
        id: ParamId,
    },
    /// Sets the properties of a body.
    SetBodyProps {
        /// The body.
        body: BodyId,
        /// New properties.
        props: BodyProps,
    },
    /// Creates a component and places one occurrence of it.
    NewComponent {
        /// Component to place it in.
        parent: ComponentId,
        /// Name.
        name: String,
        /// Placement.
        transform: Transform,
    },
    /// Renames a component (the root renames the design).
    RenameComponent {
        /// The component.
        id: ComponentId,
        /// New name.
        name: String,
    },
    /// Sets an occurrence's placement.
    SetOccurrenceTransform {
        /// The occurrence.
        occ: OccurrenceId,
        /// New placement.
        transform: Transform,
    },
    /// Shows or hides an occurrence.
    SetOccurrenceVisible {
        /// The occurrence.
        occ: OccurrenceId,
        /// New visibility.
        visible: bool,
    },
    /// Design name and/or default units (`None` keeps the current value).
    SetDesignMeta {
        /// New name.
        name: Option<String>,
        /// New default units; parameters are re-evaluated.
        default_units: Option<UnitPrefs>,
    },
    /// Switches between history and direct modelling (DOC-08, see [`crate::direct`]). To
    /// `Direct` it bakes the evaluated bodies, so only `Document::apply` can do that; to
    /// `History` it adopts the baked bodies in a base feature.
    SetDesignMode {
        /// New mode.
        mode: DesignMode,
    },
    /// Sets or removes an extension record under a key from an earlier [`Command::AddExt`].
    SetExt {
        /// The key.
        key: ExtKey,
        /// The record (`None`: remove).
        record: Option<ExtRecord>,
    },
    /// Adds an extension record under a freshly allocated key of the record's kind.
    AddExt {
        /// The record; its `kind` selects the key's kind.
        record: ExtRecord,
    },
    /// Adds an empty configuration (DOC-10, see [`crate::config`]).
    AddConfig {
        /// Unique, non-empty name.
        name: String,
    },
    /// Renames a configuration.
    RenameConfig {
        /// The configuration.
        config: ConfigId,
        /// New unique name.
        name: String,
    },
    /// Deletes a configuration; if it was active, the base design becomes active.
    DeleteConfig {
        /// The configuration.
        config: ConfigId,
    },
    /// Overrides (`Some`) or restores (`None`) a parameter's expression in a configuration.
    SetConfigParam {
        /// The configuration.
        config: ConfigId,
        /// The parameter.
        param: ParamId,
        /// Replacement expression source.
        expr: Option<String>,
    },
    /// Forces (`Some`) or stops forcing (`None`) a feature's suppression in a configuration.
    SetConfigSuppressed {
        /// The configuration.
        config: ConfigId,
        /// The feature.
        feature: FeatureId,
        /// Forced state.
        suppressed: Option<bool>,
    },
    /// Activates a configuration (`None`: the base design). Recompute then evaluates the
    /// design with that configuration's overrides.
    SetActiveConfig {
        /// The configuration.
        config: Option<ConfigId>,
    },
}

impl Command {
    /// Short undo-menu label ("Add feature", "Move feature", ...).
    pub fn label(&self) -> &'static str {
        match self {
            Command::AddFeature { .. } => "Add feature",
            Command::EditFeature { .. } => "Edit feature",
            Command::DeleteFeatures { .. } => "Delete features",
            Command::MoveFeature { .. } => "Move feature",
            Command::SetMarker { .. } => "Move marker",
            Command::SetSuppressed {
                suppressed: true, ..
            } => "Suppress features",
            Command::SetSuppressed { .. } => "Unsuppress features",
            Command::RenameFeature { .. } => "Rename feature",
            Command::GroupFeatures { .. } => "Group features",
            Command::Ungroup { .. } => "Ungroup",
            Command::SetGroup { .. } => "Edit group",
            Command::AddParam { .. } => "Add parameter",
            Command::SetParamExpr { .. } => "Edit parameter",
            Command::RenameParam { .. } => "Rename parameter",
            Command::SetParamMeta { .. } => "Edit parameter",
            Command::DeleteParam { .. } => "Delete parameter",
            Command::SetBodyProps { .. } => "Edit body",
            Command::NewComponent { .. } => "New component",
            Command::RenameComponent { .. } => "Rename component",
            Command::SetOccurrenceTransform { .. } => "Move occurrence",
            Command::SetOccurrenceVisible { .. } => "Toggle visibility",
            Command::SetDesignMeta { .. } => "Design settings",
            Command::SetDesignMode { .. } => "Change design mode",
            Command::SetExt { .. } | Command::AddExt { .. } => "Edit extension data",
            Command::AddConfig { .. } => "Add configuration",
            Command::RenameConfig { .. } => "Rename configuration",
            Command::DeleteConfig { .. } => "Delete configuration",
            Command::SetConfigParam { .. } | Command::SetConfigSuppressed { .. } => {
                "Edit configuration"
            }
            Command::SetActiveConfig { .. } => "Activate configuration",
        }
    }
}

/// An id created by a command.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CreatedId {
    /// A timeline feature.
    Feature(FeatureId),
    /// A parameter.
    Param(ParamId),
    /// A component.
    Component(ComponentId),
    /// An occurrence.
    Occurrence(OccurrenceId),
    /// A timeline group.
    Group(GroupId),
    /// An extension record key.
    Ext(ExtKey),
    /// A configuration.
    Config(ConfigId),
}

/// What a command did.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CommandOutcome {
    /// Ids the command created, in creation order.
    pub created: Vec<CreatedId>,
    /// The document revision after the command (see `History::revision`).
    pub revision: Revision,
}

/// Why a command was rejected. A rejected command changes nothing.
#[derive(thiserror::Error, Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum CommandError {
    /// A referenced id does not exist.
    #[error("no such {what} {id}")]
    NotFound {
        /// Kind of id.
        what: &'static str,
        /// The raw id.
        id: u64,
    },
    /// The edit would put a feature before one of its inputs.
    #[error("{feature} would come before its input {input}")]
    WouldBreakDependency {
        /// The feature whose input would come later.
        feature: FeatureId,
        /// The input's producer.
        input: FeatureId,
    },
    /// A parameter expression or name is invalid.
    #[error(transparent)]
    Expr(#[from] ExprError),
    /// Another parameter has this name.
    #[error("name `{0}` already in use")]
    NameTaken(String),
    /// Still referenced elsewhere.
    #[error("{what} {id} is still used by {by:?}")]
    InUse {
        /// Kind of entity.
        what: &'static str,
        /// The raw id.
        id: u64,
        /// Raw ids of the users (parameters or features).
        by: Vec<u64>,
    },
    /// Not allowed in this design mode.
    #[error("not allowed in {0:?} mode")]
    WrongMode(DesignMode),
    /// Any other rejected value or state.
    #[error("{0}")]
    Invalid(String),
    /// A low-level model error without a dedicated variant.
    #[error(transparent)]
    Model(ModelError),
}

impl amg_core::Diagnostic for CommandError {}

impl From<ModelError> for CommandError {
    fn from(e: ModelError) -> CommandError {
        match e {
            ModelError::NotFound { what, id } => CommandError::NotFound { what, id },
            ModelError::WouldBreakDependency { feature, input } => {
                CommandError::WouldBreakDependency { feature, input }
            }
            other => CommandError::Model(other),
        }
    }
}

impl From<ParamError> for CommandError {
    fn from(e: ParamError) -> CommandError {
        match e {
            ParamError::Expr(e) => CommandError::Expr(e),
            ParamError::NameTaken(n) => CommandError::NameTaken(n),
            ParamError::NotFound(id) => CommandError::NotFound {
                what: "parameter",
                id: id.0,
            },
            ParamError::InUse { id, by } => CommandError::InUse {
                what: "parameter",
                id: id.0,
                by: by.into_iter().map(|p| p.0).collect(),
            },
            ParamError::Model(m) => m.into(),
        }
    }
}

type Created = Vec<CreatedId>;

/// Applies `cmd` to `design` and returns the ids it created. On `Err` the design is unchanged.
///
/// In direct mode the timeline commands are refused with [`CommandError::WrongMode`]; switching
/// from history to direct mode needs the evaluated bodies and goes through `Document::apply`.
pub fn apply_command(design: &mut Design, cmd: Command) -> Result<Vec<CreatedId>, CommandError> {
    if design.meta().mode == DesignMode::Direct && edits_timeline(&cmd) {
        return Err(CommandError::WrongMode(DesignMode::Direct));
    }
    apply_ungated(design, cmd)
}

/// True for the commands that only make sense with a timeline.
fn edits_timeline(cmd: &Command) -> bool {
    matches!(
        cmd,
        Command::AddFeature { .. }
            | Command::EditFeature { .. }
            | Command::DeleteFeatures { .. }
            | Command::MoveFeature { .. }
            | Command::SetMarker { .. }
            | Command::SetSuppressed { .. }
            | Command::RenameFeature { .. }
            | Command::GroupFeatures { .. }
            | Command::Ungroup { .. }
            | Command::SetGroup { .. }
    )
}

/// [`apply_command`] without the design-mode check (direct-mode baking adds its scratch
/// feature this way).
pub(crate) fn apply_ungated(
    design: &mut Design,
    cmd: Command,
) -> Result<Vec<CreatedId>, CommandError> {
    // Commands touch several structures (a delete converts parameters, a group edits nodes),
    // so run on a scratch copy and swap in on success. Copies are O(1) (persistent maps).
    let mut work = design.clone();
    let check = affects_configs(&cmd);
    let created = run(&mut work, cmd)?;
    if !work.configs().is_empty() {
        // Deleted parameters and features take their overrides with them; edits that could
        // leave a configuration unevaluable (DOC-10) are refused as a whole.
        work.prune_configs();
        if check && let Err(m) = work.check_configs() {
            return Err(CommandError::Invalid(format!("would break {m}")));
        }
    }
    *design = work;
    Ok(created)
}

/// Commands after which every configuration must still evaluate.
fn affects_configs(cmd: &Command) -> bool {
    matches!(
        cmd,
        Command::SetParamExpr { .. }
            | Command::RenameParam { .. }
            | Command::DeleteParam { .. }
            | Command::SetDesignMeta { .. }
            | Command::AddParam { .. }
    )
}

fn require_feature(d: &Design, id: FeatureId) -> Result<(), CommandError> {
    if d.timeline().contains(id) {
        Ok(())
    } else {
        Err(CommandError::NotFound {
            what: "feature",
            id: id.0,
        })
    }
}

/// Parameters a feature field points to must exist.
fn check_feature_params(d: &Design, f: &FeatureBox) -> Result<(), CommandError> {
    for e in f.exprs() {
        if let Expr::Param(p) = e
            && d.params().get(*p).is_none()
        {
            return Err(CommandError::NotFound {
                what: "parameter",
                id: p.0,
            });
        }
    }
    Ok(())
}

fn run(d: &mut Design, cmd: Command) -> Result<Created, CommandError> {
    let mut created = Vec::new();
    match cmd {
        Command::AddFeature {
            at,
            feature,
            context,
            name,
        } => {
            check_feature_params(d, &feature)?;
            let id = d.add_feature(at, feature, context, name)?;
            created.push(CreatedId::Feature(id));
        }
        Command::EditFeature { id, feature } => {
            if d.timeline()
                .node(id)
                .is_some_and(|(_, n)| n.feature.downcast_ref::<BaseFeature>().is_some())
            {
                return Err(CommandError::Invalid(
                    "the base feature can't be edited".to_owned(),
                ));
            }
            check_feature_params(d, &feature)?;
            d.replace_feature(id, feature)?;
        }
        Command::DeleteFeatures { mut ids } => {
            if ids.is_empty() {
                return Err(CommandError::Invalid("no features given".to_owned()));
            }
            ids.sort_unstable();
            ids.dedup();
            for &id in &ids {
                require_feature(d, id)?;
            }
            for &id in &ids {
                d.remove_feature(id)?;
            }
            d.convert_model_params(&ids);
        }
        Command::MoveFeature { id, to } => d.move_feature(id, to)?,
        Command::SetMarker { index } => d.set_marker(index)?,
        Command::SetSuppressed { ids, suppressed } => {
            for &id in &ids {
                require_feature(d, id)?;
            }
            for id in ids {
                d.set_suppressed(id, suppressed)?;
            }
        }
        Command::RenameFeature { id, name } => d.rename_feature(id, name)?,
        Command::GroupFeatures { ids, name } => {
            let g = d.group_features(&ids, &name)?;
            created.push(CreatedId::Group(g));
        }
        Command::Ungroup { group } => d.ungroup(group)?,
        Command::SetGroup {
            group,
            name,
            collapsed,
        } => {
            let mut g = d
                .timeline()
                .group(group)
                .cloned()
                .ok_or(CommandError::NotFound {
                    what: "group",
                    id: group.0,
                })?;
            if let Some(n) = name {
                g.name = n;
            }
            if let Some(c) = collapsed {
                g.collapsed = c;
            }
            d.set_group(group, g)?;
        }
        Command::AddParam {
            name,
            expr,
            unit,
            comment,
        } => {
            let id = d.add_param(&name, &expr, unit, &comment)?;
            created.push(CreatedId::Param(id));
        }
        Command::SetParamExpr { id, expr } => d.set_param_expr(id, &expr)?,
        Command::RenameParam { id, name } => {
            let old = d.params().get(id).map(|p| p.name.clone());
            d.rename_param(id, &name)?;
            if let Some(old) = old
                && old != name
            {
                d.rename_in_configs(&old, &name);
            }
        }
        Command::SetParamMeta {
            id,
            comment,
            favorite,
        } => d.set_param_meta(id, comment, favorite)?,
        Command::DeleteParam { id } => {
            let users: Vec<u64> = d
                .timeline()
                .nodes()
                .filter(|n| n.feature.exprs().iter().any(|e| **e == Expr::Param(id)))
                .map(|n| n.id.0)
                .collect();
            if !users.is_empty() {
                return Err(CommandError::InUse {
                    what: "parameter",
                    id: id.0,
                    by: users,
                });
            }
            d.delete_param(id)?;
        }
        Command::SetBodyProps { body, props } => d.set_body_props(body, props)?,
        Command::NewComponent {
            parent,
            name,
            transform,
        } => {
            let (c, o) = d.add_component(parent, &name, transform)?;
            created.push(CreatedId::Component(c));
            created.push(CreatedId::Occurrence(o));
        }
        Command::RenameComponent { id, name } => d.rename_component(id, &name)?,
        Command::SetOccurrenceTransform { occ, transform } => {
            d.set_occurrence_transform(occ, transform)?;
        }
        Command::SetOccurrenceVisible { occ, visible } => {
            d.set_occurrence_visible(occ, visible)?;
        }
        Command::SetDesignMeta {
            name,
            default_units,
        } => {
            if let Some(u) = default_units {
                d.set_default_units(u)?;
            }
            if let Some(n) = name {
                d.set_name(&n);
            }
        }
        Command::SetDesignMode { mode } => match (d.meta().mode, mode) {
            (a, b) if a == b => {}
            (DesignMode::Direct, _) => {
                if let Some(f) = d.leave_direct_mode()? {
                    created.push(CreatedId::Feature(f));
                }
            }
            (_, _) => {
                return Err(CommandError::Invalid(
                    "switching to direct mode bakes the evaluated bodies: apply it \
                         through Document::apply"
                        .to_owned(),
                ));
            }
        },
        Command::SetExt { key, record } => d.set_ext(key, record)?,
        Command::AddExt { record } => {
            let key = d.new_ext_key(&record.kind)?;
            d.set_ext(key.clone(), Some(record))?;
            created.push(CreatedId::Ext(key));
        }
        Command::AddConfig { name } => {
            if d.configs().by_name(&name).is_some() {
                return Err(CommandError::NameTaken(name));
            }
            let id = d.add_config(&name)?;
            created.push(CreatedId::Config(id));
        }
        Command::RenameConfig { config, name } => {
            if d.configs().by_name(&name).is_some_and(|c| c != config) {
                return Err(CommandError::NameTaken(name));
            }
            d.rename_config(config, &name)?;
        }
        Command::DeleteConfig { config } => d.delete_config(config)?,
        Command::SetConfigParam {
            config,
            param,
            expr,
        } => d.set_config_param(config, param, expr.as_deref())?,
        Command::SetConfigSuppressed {
            config,
            feature,
            suppressed,
        } => d.set_config_suppressed(config, feature, suppressed)?,
        Command::SetActiveConfig { config } => d.set_active_config(config)?,
    }
    Ok(created)
}
