//! Competition-loop adapter around the shared selector, with Disable first.
use vexide::display::Display;

use crate::selector::{Background, Selector, SelectorItem, StatusHandle, THEME_DARK};

/// A selector owns presentation while the competition runtime owns route dispatch.
pub trait RouteSelector<A> {
    fn action(&self) -> A;
    fn poll(&mut self, selection_enabled: bool) -> Option<A>;
    fn status(&mut self, message: &str);
}

type RouteSelectorFactory<A> = dyn FnOnce(Vec<(String, A)>) -> Box<dyn RouteSelector<A>>;

/// Build a stock Brain menu or give the route list to a custom UI factory.
pub enum Ui<A> {
    Brain(Display),
    /// Stock route menu with an application-defined background decoration.
    BrainWithBackground(Display, Background),
    Custom(Box<RouteSelectorFactory<A>>),
}

impl<A: Copy + 'static> Ui<A> {
    pub fn custom<S: RouteSelector<A> + 'static>(
        make: impl FnOnce(Vec<(String, A)>) -> S + 'static,
    ) -> Self {
        Self::Custom(Box::new(|routes| Box::new(make(routes))))
    }

    pub fn build(self, disabled: A, routes: Vec<(String, A)>) -> Box<dyn RouteSelector<A>> {
        self.build_with_default(disabled, routes, None)
    }

    /// Select a named initial route on the stock Brain menu; custom UIs own their selection.
    pub fn build_with_default(
        self,
        disabled: A,
        routes: Vec<(String, A)>,
        default: Option<&str>,
    ) -> Box<dyn RouteSelector<A>> {
        match self {
            Self::Brain(display) => {
                Box::new(RouteMenu::with_default(display, disabled, routes, default))
            }
            Self::BrainWithBackground(display, background) => {
                Box::new(RouteMenu::with_default_and_background(
                    display,
                    disabled,
                    routes,
                    default,
                    Some(background),
                ))
            }
            Self::Custom(build) => {
                let mut choices = vec![("Disable".into(), disabled)];
                choices.extend(routes);
                build(choices)
            }
        }
    }
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
        Self::with_default(display, disabled, routes, None)
    }

    /// Keep Disable first while optionally selecting a named route at startup.
    pub fn with_default(
        display: Display,
        disabled: A,
        routes: impl IntoIterator<Item = (impl Into<String>, A)>,
        default: Option<&str>,
    ) -> Self {
        Self::with_default_and_background(display, disabled, routes, default, None)
    }

    fn with_default_and_background(
        display: Display,
        disabled: A,
        routes: impl IntoIterator<Item = (impl Into<String>, A)>,
        default: Option<&str>,
        background: Option<Background>,
    ) -> Self {
        let mut choices = vec![Choice {
            label: "Disable".into(),
            action: disabled,
        }];
        choices.extend(routes.into_iter().map(|(label, action)| Choice {
            label: label.into(),
            action,
        }));
        let selection = default
            .and_then(|label| choices.iter().position(|choice| choice.label == label))
            .unwrap_or_default();
        let selector =
            Selector::new_with_background(display, choices, selection, THEME_DARK, background);
        let status = selector.status_handle();
        status.set_selection_enabled(
            vexide::competition::mode() == vexide::competition::CompetitionMode::Disabled,
        );
        Self {
            _selector: selector,
            status,
            last_selection: selection,
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
