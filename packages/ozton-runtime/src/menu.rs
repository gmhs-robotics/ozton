//! Competition-loop adapter around the shared selector, with Disable first.
use vexide::display::Display;

use crate::selector::{Selector, SelectorItem, StatusHandle};

#[derive(Clone)]
struct Choice<A> {
    label: String,
    action: A,
}
impl<A: Copy> SelectorItem for Choice<A> {
    fn label(&self) -> &str {
        &self.label
    }
}

/// Reuses the same rendering and status handling as recording and programmed menus.
/// Call `poll` every tick to lock selection while enabled. No routine is registered
/// implicitly; construction provides only the caller's disabled action.
pub struct RouteMenu<A: Copy + 'static> {
    _selector: Selector<Choice<A>>,
    status: StatusHandle<Choice<A>>,
    last_selection: usize,
    last_status: String,
}
impl<A: Copy + 'static> RouteMenu<A> {
    pub fn new(display: Display, disabled: A) -> Self {
        Self::with_choices(display, disabled, std::iter::empty::<(&str, A)>())
    }

    /// Build all choices before starting the selector, always with Disable first.
    pub fn with_choices(
        display: Display,
        disabled: A,
        routes: impl IntoIterator<Item = (impl Into<String>, A)>,
    ) -> Self {
        let mut choices = vec![Choice {
            label: "Disable".into(),
            action: disabled,
        }];
        choices.extend(routes.into_iter().map(|(label, action)| Choice {
            label: label.into(),
            action,
        }));
        let selector = Selector::new(display, choices, 0);
        let status = selector.status_handle();
        status.set_selection_enabled(
            vexide::competition::mode() == vexide::competition::CompetitionMode::Disabled,
        );
        Self {
            _selector: selector,
            status,
            last_selection: 0,
            last_status: String::new(),
        }
    }
    pub fn add(&mut self, label: impl Into<String>, action: A) {
        self.status.add_option(Choice {
            label: label.into(),
            action,
        });
    }
    pub fn action(&self) -> A {
        self.status.with_selection(|choice| choice.action)
    }
    pub fn poll(&mut self, disabled: bool) -> Option<A> {
        self.status.set_selection_enabled(disabled);
        let selected = self.status.selection_index();
        if disabled && selected != self.last_selection {
            self.last_selection = selected;
            self.last_status.clear();
            Some(self.action())
        } else {
            None
        }
    }
    pub fn status(&mut self, message: &str) {
        if self.last_status != message {
            self.last_status.clear();
            self.last_status.push_str(message);
            self.status.show_status_persistent(message);
        }
    }
}
