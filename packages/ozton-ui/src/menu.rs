//! Competition-loop adapter around the shared selector.
use vexide::display::Display;

use crate::selector::{
    Background, Selector, SelectorItem, SimpleSelectTheme, StatusHandle, THEME_DARK,
};

/// A selector owns presentation while the competition runtime owns route dispatch.
pub trait RouteSelector<A> {
    fn action(&self) -> A;
    fn poll(&mut self, selection_enabled: bool) -> Option<A>;
    fn status(&mut self, message: &str);
}

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

/// Call `poll` every tick to lock selection while enabled.
pub struct RouteMenu<A: Copy + 'static> {
    _selector: Selector<Choice<A>>,
    status: StatusHandle<Choice<A>>,
    last_selection: usize,
    last_status: String,
}
impl<A: Copy + 'static> RouteMenu<A> {
    /// Build the choices in route order and select the first.
    ///
    /// Panics if `routes` is empty.
    pub fn with_choices(
        display: Display,
        routes: impl IntoIterator<Item = (impl Into<String>, A)>,
    ) -> Self {
        Self::with_style(display, routes, THEME_DARK, None)
    }

    pub(crate) fn with_style(
        display: Display,
        routes: impl IntoIterator<Item = (impl Into<String>, A)>,
        theme: SimpleSelectTheme,
        background: Option<Background>,
    ) -> Self {
        let choices = routes
            .into_iter()
            .map(|(label, action)| Choice {
                label: label.into(),
                action,
            })
            .collect();
        let selector = Selector::new_with_background(display, choices, 0, theme, background);
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

impl<A: Copy + 'static> RouteSelector<A> for RouteMenu<A> {
    fn action(&self) -> A {
        RouteMenu::action(self)
    }
    fn poll(&mut self, selection_enabled: bool) -> Option<A> {
        RouteMenu::poll(self, selection_enabled)
    }
    fn status(&mut self, message: &str) {
        RouteMenu::status(self, message);
    }
}
