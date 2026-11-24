use std::{cell::RefCell, rc::Rc};

use ozton_ui::RouteSelector;
use vexide::{
    competition::{self, CompetitionMode},
    task::{Task, spawn},
    time::sleep,
};

pub(super) struct TaskSelector<A>(Rc<RefCell<Box<dyn RouteSelector<A>>>>);

impl<A: 'static> TaskSelector<A> {
    pub(super) fn new(selector: Box<dyn RouteSelector<A>>) -> Self {
        Self(Rc::new(RefCell::new(selector)))
    }

    pub(super) fn start(&self) -> Task<()> {
        let selector = Rc::clone(&self.0);
        spawn(async move {
            loop {
                selector
                    .borrow_mut()
                    .poll(competition::mode() == CompetitionMode::Disabled);
                sleep(crate::clock::CONTROL_PERIOD).await;
            }
        })
    }
}

impl<A> RouteSelector<A> for TaskSelector<A> {
    fn action(&self) -> A {
        self.0.borrow().action()
    }

    fn poll(&mut self, selection_enabled: bool) -> Option<A> {
        self.0.borrow_mut().poll(selection_enabled)
    }

    fn status(&mut self, message: &str) {
        self.0.borrow_mut().status(message);
    }
}
