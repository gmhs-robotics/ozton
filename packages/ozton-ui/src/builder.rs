use vexide::display::Display;

use crate::{
    RouteMenu, RouteSelector,
    selector::{Background, SimpleSelectTheme, THEME_DARK},
};

/// A competition UI component that turns the runtime's routes into a selector.
pub trait RouteUi<A> {
    /// Present the supplied nonempty route list in order and select the first route.
    fn into_selector(
        self,
        routes: impl Iterator<Item = (&'static str, A)>,
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
        routes: impl Iterator<Item = (&'static str, A)>,
    ) -> impl RouteSelector<A> + 'static {
        let UiBuilder {
            display,
            theme,
            background,
        } = self.0;
        RouteMenu::with_style(display, routes, theme, background)
    }
}

/// A custom selector factory receives exactly the registered routes in order.
/// The first choice is the default.
impl<A, F, S> RouteUi<A> for F
where
    F: FnOnce(Vec<(String, A)>) -> S,
    S: RouteSelector<A> + 'static,
{
    fn into_selector(
        self,
        routes: impl Iterator<Item = (&'static str, A)>,
    ) -> impl RouteSelector<A> + 'static {
        let choices = routes
            .map(|(label, action)| (label.into(), action))
            .collect();
        self(choices)
    }
}
