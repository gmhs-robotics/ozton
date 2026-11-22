use super::*;

impl<I: SelectorItem + 'static> StatusHandle<I> {
    /// Replaces menu options after a route is added or a save can be retried.
    /// Lock touch selection without stopping drawing or persistent status updates.
    pub fn set_selection_enabled(&self, enabled: bool) {
        self.state.borrow_mut().selection_enabled = enabled;
    }

    /// Replaces the choices. Returns false if the incoming list is empty.
    pub fn replace_options(&self, options: Vec<I>, selected: usize) -> bool {
        if options.is_empty() {
            return false;
        }
        let options: Vec<_> = options.into_iter().map(Rc::new).collect();
        let mut state = self.state.borrow_mut();
        state.selection = selected.min(options.len() - 1);
        state.selection_revision = state.selection_revision.wrapping_add(1);
        let old = std::mem::replace(&mut state.options, options);
        state.active_row = None;
        state.options_dirty = true;
        state.status_dirty = true;
        drop(state);
        drop(old);
        true
    }

    pub(crate) fn add_option(&self, option: I) {
        let mut state = self.state.borrow_mut();
        state.options.push(Rc::new(option));
        state.options_dirty = true;
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
        let option = {
            let state = self.state.borrow();
            Rc::clone(&state.options[state.selection])
        };
        read(option.as_ref())
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
        let revision = self.status.state.borrow().selection_revision;

        if self.last_selection == Some(revision) {
            return None;
        }

        self.last_selection = Some(revision);
        let selection = self.status.selection();
        Some(selection)
    }
}
