//! Touch selector, status and shared control handles.

mod drawing;
mod handles;

use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use vexide::{
    color::Color,
    display::{Display, Rect, TouchState},
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

/// Draw an optional decoration over a filled background region, before text.
/// The callback must stay inside the supplied region.
pub type Background = fn(&mut Display, Rect, Color);

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
    options: Vec<Rc<I>>,
    selection: usize,
    selection_revision: u64,
    active_row: Option<usize>,
    status: Option<StatusMessage>,
    status_dirty: bool,
    options_dirty: bool,
    selection_enabled: bool,
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
    last_selection: Option<u64>,
}

impl<I: SelectorItem + 'static> Selector<I> {
    const STATUS_HEIGHT: i16 = 24;
    const ROW_HEIGHT: i16 = 36;
    const PAGE_SIZE: usize = 5;

    pub fn new(display: Display, options: Vec<I>, default_selection: usize) -> Self {
        Self::new_with_theme(display, options, default_selection, THEME_DARK)
    }

    pub fn new_with_theme(
        display: Display,
        options: Vec<I>,
        default_selection: usize,
        theme: SimpleSelectTheme,
    ) -> Self {
        Self::new_with_background(display, options, default_selection, theme, None)
    }

    pub fn new_with_background(
        mut display: Display,
        options: Vec<I>,
        default_selection: usize,
        theme: SimpleSelectTheme,
        background: Option<Background>,
    ) -> Self {
        assert!(
            !options.is_empty(),
            "Selector requires at least one option."
        );

        let selection = default_selection.min(options.len() - 1);

        let state = Rc::new(RefCell::new(SelectorState {
            options: options.into_iter().map(Rc::new).collect(),
            selection,
            selection_revision: 0,
            active_row: None,
            status: None,
            status_dirty: true,
            options_dirty: false,
            selection_enabled: true,
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
                    Self::draw_page(&mut display, &theme, &state, page, background);
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

                        if state.selection_enabled
                            && matches!(touch.state, TouchState::Pressed | TouchState::Held)
                            && (0..Display::HORIZONTAL_RESOLUTION).contains(&touch.point.x)
                            && (0..Self::list_height()).contains(&touch.point.y)
                        {
                            let row = usize::try_from(touch.point.y / Self::ROW_HEIGHT)
                                .unwrap_or_default();
                            let page_start = page * Self::PAGE_SIZE;
                            let option_index = page_start + row;
                            if row < Self::PAGE_SIZE && option_index < state.options.len() {
                                if state.selection != option_index {
                                    state.selection = option_index;
                                    state.selection_revision =
                                        state.selection_revision.wrapping_add(1);
                                }
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
                        Self::draw_page(&mut display, &theme, &state, page, background);
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

    fn status_duration() -> Duration {
        Duration::from_secs(2)
    }
}
