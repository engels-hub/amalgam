//! Notifications and toasts (spec §14, UI-10).
//!
//! A [`Notification`] has a severity, a title, a body, an optional hint and an optional
//! `details` section. [`Notification::from_diagnostic`] builds one from any
//! [`amg_core::Diagnostic`] and is `None` for a cancellation: *cancelled is not an error*.
//!
//! [`Notifications`] is the queue the app owns. Errors stay until dismissed; info and
//! warnings hide after [`AUTO_HIDE`]. The queue has its own clock (real time plus a skew a
//! test can advance), so the 6 s rule is testable without sleeping.

use std::time::{Duration, Instant};

use amg_core::{Diagnostic, Severity};

use crate::theme::ThemeTokens;

/// How long info and warning toasts stay.
pub const AUTO_HIDE: Duration = Duration::from_secs(6);

/// One message for the user.
#[derive(Clone, Debug)]
pub struct Notification {
    /// How serious it is.
    pub severity: Severity,
    /// Short heading.
    pub title: String,
    /// The user-facing message.
    pub body: String,
    /// What the user can do about it.
    pub hint: Option<String>,
    /// Developer context (the `report` chain), shown under "Details" and copied by
    /// "Copy details".
    pub details: Option<String>,
    /// Stays until dismissed (errors by default).
    pub sticky: bool,
}

impl Notification {
    /// A notification; errors are sticky, the others auto-hide.
    pub fn new(
        severity: Severity,
        title: impl Into<String>,
        body: impl Into<String>,
    ) -> Notification {
        Notification {
            severity,
            title: title.into(),
            body: body.into(),
            hint: None,
            details: None,
            sticky: severity == Severity::Error,
        }
    }

    /// An info toast.
    pub fn info(title: impl Into<String>, body: impl Into<String>) -> Notification {
        Notification::new(Severity::Info, title, body)
    }

    /// A warning toast.
    pub fn warning(title: impl Into<String>, body: impl Into<String>) -> Notification {
        Notification::new(Severity::Warning, title, body)
    }

    /// An error toast.
    pub fn error(title: impl Into<String>, body: impl Into<String>) -> Notification {
        Notification::new(Severity::Error, title, body)
    }

    /// Adds a hint.
    #[must_use]
    pub fn with_hint(mut self, hint: impl Into<String>) -> Notification {
        self.hint = Some(hint.into());
        self
    }

    /// Adds the details section.
    #[must_use]
    pub fn with_details(mut self, details: impl Into<String>) -> Notification {
        self.details = Some(details.into());
        self
    }

    /// Makes the toast stay until dismissed.
    #[must_use]
    pub fn sticky(mut self) -> Notification {
        self.sticky = true;
        self
    }

    /// The notification for a diagnostic: its user message, hint and severity, and the
    /// [`amg_core::report`] chain as details. `None` if the diagnostic only means "cancelled".
    pub fn from_diagnostic(e: &dyn Diagnostic) -> Option<Notification> {
        if e.is_cancelled() {
            return None;
        }
        let severity = e.severity();
        let title = match severity {
            Severity::Info => "Info",
            Severity::Warning => "Warning",
            Severity::Error => "Error",
        };
        Some(Notification {
            severity,
            title: title.to_owned(),
            body: e.user_message(),
            hint: e.hint(),
            details: Some(amg_core::report(e)),
            sticky: severity == Severity::Error,
        })
    }
}

/// A shown notification.
#[derive(Clone, Debug)]
pub struct Toast {
    /// Unique per queue.
    pub id: u64,
    /// The message.
    pub notification: Notification,
    created: Instant,
}

/// The toast queue.
#[derive(Debug, Default)]
pub struct Notifications {
    items: Vec<Toast>,
    next_id: u64,
    skew: Duration,
}

impl Notifications {
    /// An empty queue.
    pub fn new() -> Notifications {
        Notifications::default()
    }

    fn now(&self) -> Instant {
        Instant::now() + self.skew
    }

    /// Shows a notification; returns its toast id.
    pub fn push(&mut self, n: Notification) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        let created = self.now();
        self.items.push(Toast {
            id,
            notification: n,
            created,
        });
        id
    }

    /// The toasts, oldest first.
    pub fn items(&self) -> &[Toast] {
        &self.items
    }

    /// True if nothing is shown.
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Removes a toast.
    pub fn dismiss(&mut self, id: u64) {
        self.items.retain(|t| t.id != id);
    }

    /// Removes every toast.
    pub fn clear(&mut self) {
        self.items.clear();
    }

    /// Moves this queue's clock forward (tests: "6 s later" without sleeping).
    pub fn advance_clock(&mut self, by: Duration) {
        self.skew += by;
    }

    /// Removes the auto-hide toasts that are due. Returns the time until the next one is due
    /// (for `request_repaint_after`), or `None` if only sticky toasts (or none) remain.
    pub fn expire(&mut self) -> Option<Duration> {
        let now = self.now();
        self.items
            .retain(|t| t.notification.sticky || now.duration_since(t.created) < AUTO_HIDE);
        self.items
            .iter()
            .filter(|t| !t.notification.sticky)
            .map(|t| AUTO_HIDE.saturating_sub(now.duration_since(t.created)))
            .min()
    }
}

/// What the user did to the toasts this frame.
#[derive(Debug, Default)]
pub(crate) struct ToastActions {
    pub dismissed: Vec<u64>,
    pub open_log: bool,
}

fn severity_color(tokens: &ThemeTokens, s: Severity) -> egui::Color32 {
    match s {
        Severity::Error => tokens.error,
        Severity::Warning => tokens.warning,
        Severity::Info => tokens.accent,
    }
}

/// Draws the toasts at the top right. Copying happens here; dismiss and "Open log folder"
/// are returned for the app to apply.
pub(crate) fn show(
    ctx: &egui::Context,
    notifications: &Notifications,
    tokens: &ThemeTokens,
) -> ToastActions {
    let mut actions = ToastActions::default();
    if notifications.is_empty() {
        return actions;
    }
    egui::Area::new(egui::Id::new("amg_toasts"))
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-12.0, 96.0))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            ui.set_max_width(380.0);
            for toast in notifications.items() {
                ui.push_id(toast.id, |ui| toast_ui(ui, toast, tokens, &mut actions));
            }
        });
    actions
}

fn toast_ui(ui: &mut egui::Ui, toast: &Toast, tokens: &ThemeTokens, actions: &mut ToastActions) {
    let n = &toast.notification;
    egui::Frame::popup(ui.style()).show(ui, |ui| {
        ui.set_min_width(300.0);
        ui.label(
            egui::RichText::new(n.title.as_str())
                .strong()
                .color(severity_color(tokens, n.severity)),
        );
        ui.label(n.body.as_str());
        if let Some(hint) = &n.hint {
            ui.label(format!("Hint: {hint}"));
        }
        if let Some(details) = &n.details {
            egui::CollapsingHeader::new("Details")
                .id_salt("details")
                .show(ui, |ui| {
                    ui.label(egui::RichText::new(details.as_str()).monospace());
                });
        }
        ui.horizontal(|ui| {
            if let Some(details) = &n.details
                && ui.button("Copy details").clicked()
            {
                ui.ctx().copy_text(details.clone());
            }
            if ui.button("Open log folder").clicked() {
                actions.open_log = true;
            }
            if ui.button("Dismiss").clicked() {
                actions.dismissed.push(toast.id);
            }
        });
    });
}

#[cfg(test)]
#[allow(clippy::expect_used, reason = "test code")]
mod tests {
    use super::*;

    #[test]
    fn cancellation_is_not_a_notification() {
        assert!(Notification::from_diagnostic(&amg_core::Cancelled).is_none());
        let e = amg_core::Error::Cancelled(amg_core::Cancelled);
        assert!(Notification::from_diagnostic(&e).is_none());
    }

    #[test]
    fn diagnostic_fields_carry_over() {
        let e = amg_core::Error::Degenerate { what: "edge" };
        let n = Notification::from_diagnostic(&e).expect("notification");
        assert_eq!(n.severity, Severity::Error);
        assert!(n.sticky);
        assert_eq!(n.body, "degenerate geometry: edge");
        assert!(n.hint.is_some());
        assert_eq!(n.details.as_deref(), Some(amg_core::report(&e).as_str()));
    }

    #[test]
    fn info_and_warning_auto_hide_errors_stay() {
        let mut q = Notifications::new();
        q.push(Notification::info("i", "info"));
        q.push(Notification::warning("w", "warn"));
        let err = q.push(Notification::error("e", "boom"));
        assert!(q.expire().is_some());
        assert_eq!(q.items().len(), 3);
        q.advance_clock(Duration::from_millis(5900));
        q.expire();
        assert_eq!(q.items().len(), 3, "still within 6 s");
        q.advance_clock(Duration::from_millis(200));
        assert_eq!(q.expire(), None);
        assert_eq!(q.items().len(), 1);
        assert_eq!(q.items()[0].id, err);
        q.dismiss(err);
        assert!(q.is_empty());
    }
}
