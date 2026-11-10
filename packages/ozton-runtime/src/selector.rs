use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use vexide::{
    color::Color,
    display::{Display, Font, FontFamily, FontSize, Line, Rect, Text, TouchState},
    task::{self, Task},
    time::sleep,
};

#[derive(Debug, Clone, Copy)]
pub struct SimpleSelectTheme {
    pub background_default: Color,
    pub background_active: Color,
    pub background_selected: Color,
    pub background_selected_active: Color,
    pub text_default: Color,
    pub text_selected: Color,
    pub text_active: Color,
    pub text_selected_active: Color,
    pub border: Color,
}

pub const THEME_DARK: SimpleSelectTheme = SimpleSelectTheme {
    background_default: Color::new(25, 25, 25),
    background_active: Color::new(102, 102, 102),
    background_selected: Color::new(67, 189, 224),
    background_selected_active: Color::new(123, 209, 233),
    text_default: Color::new(187, 187, 187),
    text_selected: Color::new(255, 255, 255),
    text_active: Color::new(187, 187, 187),
    text_selected_active: Color::new(255, 255, 255),
    border: Color::new(153, 153, 153),
};

pub trait SelectorItem: Clone {
    fn label(&self) -> &str;
}

#[derive(Debug, Clone)]
struct StatusMessage {
    text: String,
    set_at: Option<Instant>,
}

#[derive(Debug, Clone)]
struct SelectorState<I: SelectorItem + 'static> {
    options: Vec<I>,
    selection: usize,
    active_row: Option<usize>,
    status: Option<StatusMessage>,
    status_dirty: bool,
    options_dirty: bool,
}

pub struct Selector<I: SelectorItem + 'static> {
    state: Rc<RefCell<SelectorState<I>>>,
    _task: Task<()>,
}

#[derive(Debug, Clone)]
pub struct StatusHandle<I: SelectorItem + 'static> {
    state: Rc<RefCell<SelectorState<I>>>,
}

#[derive(Debug)]
pub struct SelectionController<I: SelectorItem + Clone + 'static> {
    status: StatusHandle<I>,
    last_selection: Option<usize>,
}

impl<I: SelectorItem + 'static> Selector<I> {
    const STATUS_HEIGHT: i16 = 24;
    const ROW_HEIGHT: i16 = 36;
    const PAGE_SIZE: usize = 5;

    pub fn new(display: Display, options: Vec<I>, default_selection: usize) -> Self {
        Self::new_with_theme(display, options, default_selection, THEME_DARK)
    }

    pub fn new_with_theme(
        mut display: Display,
        options: Vec<I>,
        default_selection: usize,
        theme: SimpleSelectTheme,
    ) -> Self {
        assert!(
            !options.is_empty(),
            "Selector requires at least one option."
        );

        let selection = default_selection.min(options.len() - 1);

        let state = Rc::new(RefCell::new(SelectorState {
            options,
            selection,
            active_row: None,
            status: None,
            status_dirty: true,
            options_dirty: false,
        }));

        display.set_render_mode(vexide::display::RenderMode::DoubleBuffered);

        Self {
            state: state.clone(),
            _task: task::spawn(async move {
                let mut page = selection / Self::PAGE_SIZE;
                display.fill(
                    &Rect::new(
                        [0, 0],
                        [Display::HORIZONTAL_RESOLUTION, Display::VERTICAL_RESOLUTION],
                    ),
                    theme.background_default,
                );

                {
                    let state = state.borrow();
                    Self::draw_page(&mut display, &theme, &state, page);
                }

                {
                    let state = state.borrow();
                    Self::draw_status(
                        &mut display,
                        &theme,
                        &format!("Selected: {}", state.options[state.selection].label()),
                    );
                }
                display.render();
                let mut last_press_count = display.touch_status().press_count;
                let mut last_mode = vexide::competition::mode();

                loop {
                    let touch = display.touch_status();
                    let new_press = touch.press_count != last_press_count;
                    last_press_count = touch.press_count;
                    let (redraw_page, redraw_status, status_text) = {
                        let mut state = state.borrow_mut();
                        let mode = vexide::competition::mode();
                        if mode != last_mode {
                            last_mode = mode;
                            state.status_dirty = true;
                        }
                        let prev_selection = state.selection;
                        let prev_active = state.active_row;
                        let prev_page = page;
                        let options_dirty = core::mem::take(&mut state.options_dirty);
                        if options_dirty {
                            page = state.selection / Self::PAGE_SIZE;
                        }
                        state.active_row = None;

                        if matches!(touch.state, TouchState::Pressed | TouchState::Held)
                            && (0..Display::HORIZONTAL_RESOLUTION).contains(&touch.point.x)
                            && (0..Self::list_height()).contains(&touch.point.y)
                        {
                            let row = usize::try_from(touch.point.y / Self::ROW_HEIGHT)
                                .unwrap_or_default();
                            let page_start = page * Self::PAGE_SIZE;
                            let option_index = page_start + row;
                            if row < Self::PAGE_SIZE && option_index < state.options.len() {
                                state.selection = option_index;
                                state.active_row = Some(option_index);
                            } else if new_press
                                && state.options.len() > Self::PAGE_SIZE
                                && touch.point.y >= Self::PAGE_SIZE as i16 * Self::ROW_HEIGHT
                            {
                                let last_page = (state.options.len() - 1) / Self::PAGE_SIZE;
                                if touch.point.x < Display::HORIZONTAL_RESOLUTION / 2 {
                                    page = page.saturating_sub(1);
                                } else {
                                    page = (page + 1).min(last_page);
                                }
                            }
                        }

                        if prev_selection != state.selection {
                            state.status = None;
                            state.status_dirty = true;
                        }

                        if let Some(status) = &state.status
                            && status.set_at.is_some_and(|set_at| {
                                Instant::now().saturating_duration_since(set_at)
                                    >= Self::status_duration()
                            })
                        {
                            state.status = None;
                            state.status_dirty = true;
                        }

                        let redraw_status = core::mem::take(&mut state.status_dirty);
                        let status_text = redraw_status.then(|| {
                            state
                                .status
                                .as_ref()
                                .map(|status| status.text.clone())
                                .unwrap_or_else(|| {
                                    format!("Selected: {}", state.options[state.selection].label())
                                })
                        });
                        (
                            options_dirty
                                || prev_selection != state.selection
                                || prev_active != state.active_row
                                || prev_page != page,
                            redraw_status,
                            status_text,
                        )
                    };

                    if redraw_page {
                        let state = state.borrow();
                        Self::draw_page(&mut display, &theme, &state, page);
                    }

                    if let Some(status_text) = status_text {
                        Self::draw_status(
                            &mut display,
                            &theme,
                            &format!("{last_mode:?} | {status_text}"),
                        );
                    }

                    if redraw_page || redraw_status {
                        display.render();
                    }
                    sleep(Display::REFRESH_INTERVAL).await;
                }
            }),
        }
    }

    pub fn status_handle(&self) -> StatusHandle<I> {
        StatusHandle {
            state: self.state.clone(),
        }
    }

    fn draw_page(
        display: &mut Display,
        theme: &SimpleSelectTheme,
        state: &SelectorState<I>,
        page: usize,
    ) {
        display.fill(
            &Rect::from_dimensions(
                [0, 0],
                Display::HORIZONTAL_RESOLUTION as u16,
                Self::list_height() as u16,
            ),
            theme.background_default,
        );

        let start = page * Self::PAGE_SIZE;
        for row in 0..Self::PAGE_SIZE {
            let index = start + row;
            if let Some(item) = state.options.get(index) {
                Self::draw_item(
                    display,
                    theme,
                    item.label(),
                    row,
                    index == state.selection,
                    state.active_row == Some(index),
                );
            }
        }
        let visible_rows = state
            .options
            .len()
            .saturating_sub(start)
            .min(Self::PAGE_SIZE);
        for row in 1..visible_rows {
            let y = row as i16 * Self::ROW_HEIGHT - 1;
            display.fill(
                &Line::new([0, y], [Display::HORIZONTAL_RESOLUTION, y]),
                theme.border,
            );
        }

        if state.options.len() > Self::PAGE_SIZE {
            let y = Self::ROW_HEIGHT * Self::PAGE_SIZE as i16;
            let width = Display::HORIZONTAL_RESOLUTION;
            display.fill(&Line::new([0, y], [width, y]), theme.border);
            display.fill(
                &Line::new([width / 2, y], [width / 2, Self::list_height()]),
                theme.border,
            );
            let last_page = (state.options.len() - 1) / Self::PAGE_SIZE;
            let navigation = [
                ("Prev", page > 0, 8),
                ("Next", page < last_page, width / 2 + 8),
            ];
            for (label, enabled, x) in navigation {
                display.draw_text(
                    &Text::from_string(
                        label,
                        Font::new(FontSize::MEDIUM, FontFamily::Proportional),
                        [x, y + 6],
                    ),
                    if enabled {
                        theme.text_selected
                    } else {
                        theme.text_default
                    },
                    None,
                );
            }
        }
    }

    fn draw_item(
        display: &mut Display,
        theme: &SimpleSelectTheme,
        label: &str,
        row: usize,
        selected: bool,
        active: bool,
    ) {
        let (background_color, text_color) = match (selected, active) {
            (false, false) => (theme.background_default, theme.text_default),
            (false, true) => (theme.background_active, theme.text_active),
            (true, false) => (theme.background_selected, theme.text_selected),
            (true, true) => (theme.background_selected_active, theme.text_selected_active),
        };

        let width: u16 = (Display::HORIZONTAL_RESOLUTION - 2)
            .try_into()
            .unwrap_or_default();
        let height: u16 = Self::ROW_HEIGHT
            .saturating_sub(2)
            .try_into()
            .unwrap_or_default();
        let y = row as i16 * Self::ROW_HEIGHT;

        display.fill(
            &Rect::from_dimensions([0, y], width, height),
            background_color,
        );

        let label = Self::short_label(label, 28);
        display.draw_text(
            &Text::from_string(
                &label,
                Font::new(FontSize::MEDIUM, FontFamily::Proportional),
                [8, y + 6],
            ),
            text_color,
            None,
        );
    }

    fn short_label(label: &str, max_chars: usize) -> String {
        let mut chars = label.chars();
        let mut shortened: String = chars.by_ref().take(max_chars).collect();
        if chars.next().is_some() {
            shortened.push('…');
        }
        shortened
    }

    fn draw_status(display: &mut Display, theme: &SimpleSelectTheme, text: &str) {
        let y_start = Self::list_height();
        let height: u16 = Self::STATUS_HEIGHT.try_into().unwrap_or_default();

        display.fill(
            &Rect::from_dimensions([0, y_start], Display::HORIZONTAL_RESOLUTION as u16, height),
            theme.background_default,
        );

        let text = Self::short_label(text, 32);
        display.draw_text(
            &Text::from_string(
                &text,
                Font::new(FontSize::MEDIUM, FontFamily::Proportional),
                [8, y_start + 4],
            ),
            theme.text_default,
            None,
        );
    }

    fn list_height() -> i16 {
        Display::VERTICAL_RESOLUTION - Self::STATUS_HEIGHT
    }

    fn status_duration() -> Duration {
        Duration::from_secs(2)
    }
}

impl<I: SelectorItem + 'static> StatusHandle<I> {
    /// Replaces menu options after a route is added or a save can be retried.
    pub fn replace_options(&self, options: Vec<I>, selected: usize) {
        if options.is_empty() {
            return;
        }
        let mut state = self.state.borrow_mut();
        state.selection = selected.min(options.len() - 1);
        state.options = options;
        state.active_row = None;
        state.options_dirty = true;
        state.status_dirty = true;
    }

    pub fn show_status(&self, text: impl Into<String>) {
        let text = text.into();
        let mut state = self.state.borrow_mut();
        state.status = Some(StatusMessage {
            text,
            set_at: Some(Instant::now()),
        });
        state.status_dirty = true;
    }

    /// Keeps status visible until another status or route selection replaces it.
    pub fn show_status_persistent(&self, text: impl Into<String>) {
        let mut state = self.state.borrow_mut();
        state.status = Some(StatusMessage {
            text: text.into(),
            set_at: None,
        });
        state.status_dirty = true;
    }

    pub fn selection_index(&self) -> usize {
        self.state.borrow().selection
    }

    pub fn with_selection<T>(&self, read: impl FnOnce(&I) -> T) -> T {
        let state = self.state.borrow();
        read(&state.options[state.selection])
    }

    pub fn selection(&self) -> I
    where
        I: Clone,
    {
        self.with_selection(Clone::clone)
    }
}

impl<I: SelectorItem + Clone + 'static> SelectionController<I> {
    pub fn new(status: StatusHandle<I>) -> Self {
        Self {
            status,
            last_selection: None,
        }
    }

    pub fn status(&self) -> &StatusHandle<I> {
        &self.status
    }

    pub fn consume_selection_change(&mut self) -> Option<I> {
        let selection_index = self.status.selection_index();

        if self.last_selection == Some(selection_index) {
            return None;
        }

        self.last_selection = Some(selection_index);
        let selection = self.status.selection();
        Some(selection)
    }
}
