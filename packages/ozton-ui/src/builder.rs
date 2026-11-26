use vexide::display::Display;

use crate::{
    RouteMenu, RouteSelector,
    selector::{Background, SimpleSelectTheme, THEME_DARK},
};

/// A competition UI component that turns the runtime's routes into a selector.
pub trait RouteUi<A> {
    /// `default` indexes `routes`, which excludes the Disable choice.
    fn into_selector(
        self,
        disabled: A,
        routes: impl Iterator<Item = (&'static str, A)>,
        default: Option<usize>,
    ) -> impl RouteSelector<A> + 'static;
}

/// A configured Brain UI, ready to attach to the competition runtime.
pub struct Ui(UiBuilder);

impl Ui {
    pub fn builder(display: Display) -> UiBuilder {
        UiBuilder {
            display,
            theme: THEME_DARK,
            background: None,
        }
    }
}

/// Configures the stock Brain route selector.
#[must_use]
pub struct UiBuilder {
    display: Display,
    theme: SimpleSelectTheme,
    background: Option<Background>,
}

impl UiBuilder {
    pub fn with_theme(mut self, theme: SimpleSelectTheme) -> Self {
        self.theme = theme;
        self
    }

    pub fn with_background(mut self, draw: Background) -> Self {
        self.background = Some(draw);
        self
    }

    pub fn build(self) -> Ui {
        Ui(self)
    }
}

impl<A: Copy + 'static> RouteUi<A> for Ui {
    fn into_selector(
        self,
        disabled: A,
        routes: impl Iterator<Item = (&'static str, A)>,
        default: Option<usize>,
    ) -> impl RouteSelector<A> + 'static {
        let UiBuilder {
            display,
            theme,
            background,
        } = self.0;
        RouteMenu::with_style(display, disabled, routes, default, theme, background)
    }
}

/// A custom selector factory receives all choices in menu order, with Disable first.
/// The factory controls its initial selection.
impl<A, F, S> RouteUi<A> for F
where
    F: FnOnce(Vec<(String, A)>) -> S,
    S: RouteSelector<A> + 'static,
{
    fn into_selector(
        self,
        disabled: A,
        routes: impl Iterator<Item = (&'static str, A)>,
        _default: Option<usize>,
    ) -> impl RouteSelector<A> + 'static {
        let choices = std::iter::once(("Disable", disabled))
            .chain(routes)
            .map(|(label, action)| (label.into(), action))
            .collect();
        self(choices)
    }
}
