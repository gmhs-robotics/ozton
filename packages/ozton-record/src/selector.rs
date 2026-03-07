use core::{future::Future, pin::Pin};
use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use autons::{
    Selector,
    simple::{SimpleSelectTheme, THEME_DARK},
};
use vexide::{
    display::{Display, Font, FontFamily, FontSize, Line, Rect, Text, TouchState},
    task::{self, Task},
    time::sleep,
};

pub trait SelectorItem: Clone {
    fn label(&self) -> &str;
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RecordTarget {
    #[default]
    Off,
    New,
    Overwrite(u32),
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct RecordOption {
    pub label: String,
    pub target: RecordTarget,
}

impl SelectorItem for RecordOption {
    fn label(&self) -> &str {
        &self.label
    }
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct PlaybackChoice {
    pub label: String,
    pub route_id: Option<u32>,
}

impl SelectorItem for PlaybackChoice {
    fn label(&self) -> &str {
        &self.label
    }
}

pub type SelectorCallback<R, I> =
    for<'a> fn(&'a mut R, I) -> Pin<Box<dyn Future<Output = ()> + 'a>>;

#[derive(Debug, Clone)]
struct StatusMessage {
    text: String,
    set_at: Instant,
}

#[derive(Debug, Clone)]
struct SelectorState<I: SelectorItem + 'static> {
    options: Vec<I>,
    selection: usize,
    active_row: Option<usize>,
    dirty_rows: Vec<usize>,
    status: Option<StatusMessage>,
    status_dirty: bool,
}

pub struct RecorderSelect<R: 'static, I: SelectorItem + 'static> {
    state: Rc<RefCell<SelectorState<I>>>,
    callback: SelectorCallback<R, I>,
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

impl<R, I> RecorderSelect<R, I>
where
    R: 'static,
    I: SelectorItem + 'static,
{
    const STATUS_HEIGHT: i16 = 24;

    pub fn new(
        display: Display,
        options: Vec<I>,
        default_selection: usize,
        callback: SelectorCallback<R, I>,
    ) -> Self {
        crate::log!(
            "selector.new: options={} default_selection={}",
            options.len(),
            default_selection
        );
        Self::new_with_theme(display, options, default_selection, callback, THEME_DARK)
    }

    #[allow(clippy::await_holding_refcell_ref)]
    pub fn new_with_theme(
        mut display: Display,
        options: Vec<I>,
        default_selection: usize,
        callback: SelectorCallback<R, I>,
        theme: SimpleSelectTheme,
    ) -> Self {
        assert!(
            !options.is_empty(),
            "RecorderSelect requires at least one option."
        );

        let rows = options.len().max(1);
        let cell_height = Self::cell_height(rows);
        let selection = default_selection.min(options.len() - 1);
        crate::log!(
            "selector.new_with_theme: rows={} cell_height={} initial_selection={}",
            rows,
            cell_height,
            selection
        );

        let state = Rc::new(RefCell::new(SelectorState {
            options,
            selection,
            active_row: None,
            dirty_rows: Vec::new(),
            status: None,
            status_dirty: true,
        }));

        display.set_render_mode(vexide::display::RenderMode::DoubleBuffered);

        Self {
            state: state.clone(),
            callback,
            _task: task::spawn(async move {
                display.fill(
                    &Rect::new(
                        [0, 0],
                        [Display::HORIZONTAL_RESOLUTION, Display::VERTICAL_RESOLUTION],
                    ),
                    theme.background_default,
                );

                Self::draw_borders(&mut display, &theme, cell_height);

                {
                    let state = state.borrow();
                    for row_index in 0..state.options.len() {
                        Self::draw_item(
                            &mut display,
                            &theme,
                            state.options[row_index].label(),
                            row_index,
                            row_index == state.selection,
                            false,
                            cell_height,
                        );
                    }
                }

                Self::draw_status(&mut display, &theme, None);

                loop {
                    let touch = display.touch_status();
                    let touched_index = match touch.state {
                        TouchState::Pressed | TouchState::Held
                            if touch.point.y < Self::list_height() =>
                        {
                            let row_index: usize =
                                (touch.point.y / cell_height).try_into().unwrap_or_default();
                            Some(row_index)
                        }
                        _ => None,
                    };

                    let mut state = state.borrow_mut();

                    let prev_selection = state.selection;
                    let prev_active = state.active_row;

                    if let Some(row_index) = touched_index
                        && row_index < state.options.len()
                    {
                        state.selection = row_index;
                    }

                    state.active_row = touched_index.filter(|row| *row < state.options.len());

                    let current_selection = state.selection;

                    if prev_selection != current_selection {
                        crate::log!(
                            "selector.touch: selection {} -> {}",
                            prev_selection,
                            current_selection
                        );
                        state.dirty_rows.extend([prev_selection, current_selection]);
                    }

                    if prev_active != state.active_row {
                        if let Some(prev) = prev_active {
                            state.dirty_rows.push(prev);
                        }

                        if let Some(current) = state.active_row {
                            state.dirty_rows.push(current);
                        }
                    }

                    let mut status_text = state.status.as_ref().map(|s| s.text.clone());
                    if let Some(status) = &state.status
                        && Instant::now().saturating_duration_since(status.set_at)
                            >= Self::status_duration()
                    {
                        state.status = None;
                        status_text = None;
                        state.status_dirty = true;
                    }

                    let redraw_status = if state.status_dirty {
                        state.status_dirty = false;
                        true
                    } else {
                        false
                    };

                    let selection = state.selection;
                    let active_row = state.active_row;
                    let dirty_rows = core::mem::take(&mut state.dirty_rows);
                    let redraw_rows: Vec<(usize, String)> = dirty_rows
                        .into_iter()
                        .filter_map(|index| {
                            state
                                .options
                                .get(index)
                                .map(|item| (index, item.label().to_string()))
                        })
                        .collect();

                    drop(state);

                    for (row_index, label) in redraw_rows
                        .into_iter()
                        .filter(|(row_index, _)| *row_index < rows)
                    {
                        Self::draw_item(
                            &mut display,
                            &theme,
                            &label,
                            row_index,
                            row_index == selection,
                            active_row == Some(row_index),
                            cell_height,
                        );
                    }

                    if redraw_status {
                        Self::draw_status(&mut display, &theme, status_text.as_deref());
                    }

                    display.render();
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

    fn draw_item(
        display: &mut Display,
        theme: &SimpleSelectTheme,
        label: &str,
        row: usize,
        selected: bool,
        active: bool,
        cell_height: i16,
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
        let height: u16 = cell_height.saturating_sub(2).try_into().unwrap_or_default();

        display.fill(
            &Rect::from_dimensions([0, row as i16 * cell_height], width, height),
            background_color,
        );

        display.draw_text(
            &Text::from_string(
                label,
                Font::new(FontSize::MEDIUM, FontFamily::Proportional),
                [8, row as i16 * cell_height + 6],
            ),
            text_color,
            None,
        );
    }

    fn draw_borders(display: &mut Display, theme: &SimpleSelectTheme, cell_height: i16) {
        let rows = Self::list_height() / cell_height;
        for n in 1..=rows {
            display.fill(
                &Line::new(
                    [0, n * cell_height - 1],
                    [Display::HORIZONTAL_RESOLUTION, n * cell_height - 1],
                ),
                theme.border,
            );
        }
    }

    fn draw_status(display: &mut Display, theme: &SimpleSelectTheme, text: Option<&str>) {
        let y_start = Self::list_height();
        let height: u16 = Self::STATUS_HEIGHT.try_into().unwrap_or_default();

        display.fill(
            &Rect::from_dimensions([0, y_start], Display::HORIZONTAL_RESOLUTION as u16, height),
            theme.background_default,
        );

        if let Some(text) = text {
            display.draw_text(
                &Text::from_string(
                    text,
                    Font::new(FontSize::MEDIUM, FontFamily::Proportional),
                    [8, y_start + 4],
                ),
                theme.text_default,
                None,
            );
        }
    }

    fn list_height() -> i16 {
        Display::VERTICAL_RESOLUTION - Self::STATUS_HEIGHT
    }

    fn cell_height(rows: usize) -> i16 {
        (Self::list_height() / rows.max(1) as i16).max(1)
    }

    fn status_duration() -> Duration {
        Duration::from_secs(2)
    }
}

impl<R, I> Selector<R> for RecorderSelect<R, I>
where
    R: 'static,
    I: SelectorItem + 'static,
{
    async fn run(&self, robot: &mut R) {
        let (callback, selection) = {
            let state = self.state.borrow();
            crate::log!(
                "selector.run: invoking callback for selection={} label={}",
                state.selection,
                state.options[state.selection].label()
            );
            (self.callback, state.options[state.selection].clone())
        };

        {
            let mut state = self.state.borrow_mut();
            let selection_index = state.selection;
            state.dirty_rows.push(selection_index);
        }

        (callback)(robot, selection).await;
    }
}

impl<I: SelectorItem + 'static> StatusHandle<I> {
    pub fn show_status(&self, text: impl Into<String>) {
        let text = text.into();
        crate::log!("selector.status: {}", text);
        let mut state = self.state.borrow_mut();
        state.status = Some(StatusMessage {
            text,
            set_at: Instant::now(),
        });
        state.status_dirty = true;
    }

    pub fn selection_index(&self) -> usize {
        self.state.borrow().selection
    }

    pub fn selection(&self) -> I
    where
        I: Clone,
    {
        self.state.borrow().options[self.state.borrow().selection].clone()
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
        crate::log!(
            "selector.consume_selection_change: selection_index={} label={}",
            selection_index,
            selection.label()
        );
        Some(selection)
    }
}
