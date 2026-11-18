//! Dialog widgets (spec §9, Interfaces §A `amg_ui::widgets`, UI-15).
//!
//! Public framework API for hand-built dialogs (MESH-23 and the other domain tools) and for the
//! generic feature dialog renderer (UI-05):
//! - [`ExprInput`]: an expression field. It parses on every change with
//!   [`ParamTable::try_eval`](amg_doc::param::ParamTable::try_eval), underlines the error span,
//!   shows the evaluated value per the document [`UnitPrefs`](amg_doc::UnitPrefs)
//!   ([`format_quantity`]), autocompletes parameter, function and unit names, nudges with
//!   Up/Down and shows the bound model parameter (`d3 = 20.00 mm`). `Enter` or focus loss
//!   commits, `Esc` reverts.
//! - [`SelectionField`]: "N selected", a clear button and the active state. Only one field is
//!   active at a time (app-wide), and the active one sets the tool's pick filter: the router
//!   intersects the active tool's filter with it ([`active_selection_filter`]).
//! - [`ChoiceField`] (dropdown or a row of buttons), [`toggle`], [`IntField`] and [`flip`].
//! - [`DialogFrame`]: help link, error/warning banner (preview health), the body, collapsible
//!   [`group`]s, and OK / Cancel with OK disabled (and the reason shown) while any field is
//!   invalid.
//!
//! Every widget carries its label into AccessKit (`labelled_by`, or the widget's own text), so
//! kittest and screen readers find it by label.

mod expr;
mod fields;
mod frame;
mod selection;

pub use expr::{
    Completion, CompletionKind, ExprCheck, ExprInput, ExprResponse, check_expr, completions,
};
pub use fields::{ChoiceField, IntField, flip, toggle};
pub use frame::{DialogAction, DialogFrame, group};
pub use selection::{SelectionField, active_selection_filter, selection_count_valid};

use amg_core::{Dimension, FormatSpec, FormatStyle, Quantity, Unit};
use amg_doc::UnitPrefs;

/// Decimals shown for evaluated values (`15.00 mm`, Fusion style).
pub const DISPLAY_PRECISION: u8 = 2;

/// Default nudge step for Up/Down, in the field's display unit (1 mm, 1 deg, ...). UI-11 makes
/// it a preference.
pub const DEFAULT_NUDGE_STEP: f64 = 1.0;

/// The unit a value of dimension `dim` is shown in under `prefs`: the document length unit
/// for lengths (and its square / cube for areas and volumes), degrees for angles, kilograms
/// for masses, otherwise
/// the canonical unit of the dimension if the unit table has one. `None` for dimensionless
/// values and dimensions without a table unit.
pub fn display_unit(dim: Dimension, prefs: &UnitPrefs) -> Option<Unit> {
    let len: Unit = prefs.length.into();
    if dim == Dimension::LENGTH {
        return Some(len);
    }
    if dim == Dimension::AREA || dim == Dimension::VOLUME {
        let exp = if dim == Dimension::AREA { 2 } else { 3 };
        if let Some(u) = Unit::from_symbol(&format!("{}^{exp}", len.symbol())) {
            return Some(u);
        }
    }
    if dim == Dimension::ANGLE {
        return Some(Unit::Degree);
    }
    if dim == Dimension::MASS {
        return Some(Unit::Kilogram);
    }
    if dim == Dimension::NONE {
        return None;
    }
    Unit::all()
        .iter()
        .copied()
        .find(|u| u.dimension() == dim && u.to_canonical(1.0).to_bits() == 1f64.to_bits())
}

/// Formats an evaluated value for display: [`DISPLAY_PRECISION`] decimals in its
/// [`display_unit`] (`15.00 mm`, `0.59 in`, `90.00 deg`). Dimensionless values print the bare
/// number; dimensions without a table unit print the canonical value with the dimension's
/// canonical unit name (`mm*t/s^3`).
pub fn format_quantity(q: Quantity, prefs: &UnitPrefs) -> String {
    let spec = FormatSpec {
        style: FormatStyle::Decimal,
        precision: DISPLAY_PRECISION,
        trim_zeros: false,
        show_unit: true,
    };
    let prec = usize::from(DISPLAY_PRECISION);
    match display_unit(q.dim, prefs) {
        Some(u) => amg_core::format_quantity(q, u, &spec)
            .unwrap_or_else(|_| format!("{:.prec$} {}", q.value, q.dim)),
        None if q.dim == Dimension::NONE => format!("{:.prec$}", q.value),
        None => format!("{:.prec$} {}", q.value, q.dim),
    }
}

#[cfg(test)]
mod tests {
    use amg_core::LengthUnit;

    use super::*;

    #[test]
    fn values_format_with_two_decimals_in_the_document_unit() {
        let mm = UnitPrefs::default();
        let inch = UnitPrefs {
            length: LengthUnit::Inch,
        };
        let l = Quantity::new(15.0, Dimension::LENGTH);
        assert_eq!(format_quantity(l, &mm), "15.00 mm");
        assert_eq!(
            format_quantity(Quantity::new(25.4, Dimension::LENGTH), &inch),
            "1.00 in"
        );
        assert_eq!(
            format_quantity(
                Quantity::new(std::f64::consts::FRAC_PI_2, Dimension::ANGLE),
                &mm
            ),
            "90.00 deg"
        );
        assert_eq!(format_quantity(Quantity::dimensionless(2.5), &mm), "2.50");
        assert_eq!(
            format_quantity(Quantity::new(100.0, Dimension::AREA), &mm),
            "100.00 mm^2"
        );
        assert_eq!(display_unit(Dimension::MASS, &mm), Some(Unit::Kilogram));
        assert_eq!(display_unit(Dimension::TIME, &mm), Some(Unit::Second));
    }
}
