use std::borrow::Cow;

use vexide::display::{Font, FontFamily, FontSize, Line, Rect, Text};

use super::*;

impl<I: SelectorItem + 'static> Selector<I> {
    pub(super) fn draw_page(
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
                label.as_ref(),
                Font::new(FontSize::MEDIUM, FontFamily::Proportional),
                [8, y + 6],
            ),
            text_color,
            None,
        );
    }

    fn short_label(label: &str, max_chars: usize) -> Cow<'_, str> {
        let Some((end, _)) = label.char_indices().nth(max_chars) else {
            return Cow::Borrowed(label);
        };
        let mut shortened = String::with_capacity(end + '…'.len_utf8());
        shortened.push_str(&label[..end]);
        shortened.push('…');
        Cow::Owned(shortened)
    }

    pub(super) fn draw_status(display: &mut Display, theme: &SimpleSelectTheme, text: &str) {
        let y_start = Self::list_height();
        let height: u16 = Self::STATUS_HEIGHT.try_into().unwrap_or_default();

        display.fill(
            &Rect::from_dimensions([0, y_start], Display::HORIZONTAL_RESOLUTION as u16, height),
            theme.background_default,
        );

        let text = Self::short_label(text, 32);
        display.draw_text(
            &Text::from_string(
                text.as_ref(),
                Font::new(FontSize::MEDIUM, FontFamily::Proportional),
                [8, y_start + 4],
            ),
            theme.text_default,
            None,
        );
    }

    pub(super) fn list_height() -> i16 {
        Display::VERTICAL_RESOLUTION - Self::STATUS_HEIGHT
    }
}
