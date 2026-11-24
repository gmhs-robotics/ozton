//! Shared handles for actuator and encoder access to the same motor objects.

use std::{
    cell::{BorrowError, BorrowMutError, Ref, RefCell, RefMut},
    rc::Rc,
};

use vexide::prelude::Motor;

trait MotorStorage {
    fn borrow(&self) -> Ref<'_, [Motor]>;
    fn borrow_mut(&self) -> RefMut<'_, [Motor]>;
    fn try_borrow(&self) -> Result<Ref<'_, [Motor]>, BorrowError>;
    fn try_borrow_mut(&self) -> Result<RefMut<'_, [Motor]>, BorrowMutError>;
}

impl<T: AsRef<[Motor]> + AsMut<[Motor]>> MotorStorage for RefCell<T> {
    fn borrow(&self) -> Ref<'_, [Motor]> {
        Ref::map(self.borrow(), AsRef::as_ref)
    }

    fn borrow_mut(&self) -> RefMut<'_, [Motor]> {
        RefMut::map(self.borrow_mut(), AsMut::as_mut)
    }

    fn try_borrow(&self) -> Result<Ref<'_, [Motor]>, BorrowError> {
        RefCell::try_borrow(self).map(|motors| Ref::map(motors, AsRef::as_ref))
    }

    fn try_borrow_mut(&self) -> Result<RefMut<'_, [Motor]>, BorrowMutError> {
        RefCell::try_borrow_mut(self).map(|motors| RefMut::map(motors, AsMut::as_mut))
    }
}

/// A shared group of motors in wiring order.
#[derive(Clone)]
pub struct SharedMotors(Rc<dyn MotorStorage>);

impl SharedMotors {
    pub fn borrow(&self) -> Ref<'_, [Motor]> {
        self.0.borrow()
    }

    pub fn borrow_mut(&self) -> RefMut<'_, [Motor]> {
        self.0.borrow_mut()
    }

    /// Background readers must skip a sample rather than panic on a busy group.
    pub fn try_borrow(&self) -> Result<Ref<'_, [Motor]>, BorrowError> {
        self.0.try_borrow()
    }

    pub fn try_borrow_mut(&self) -> Result<RefMut<'_, [Motor]>, BorrowMutError> {
        self.0.try_borrow_mut()
    }
}

impl<const N: usize> From<Rc<RefCell<[Motor; N]>>> for SharedMotors {
    fn from(motors: Rc<RefCell<[Motor; N]>>) -> Self {
        Self(motors)
    }
}

impl FromIterator<Motor> for SharedMotors {
    fn from_iter<I: IntoIterator<Item = Motor>>(motors: I) -> Self {
        Self(Rc::new(RefCell::new(
            motors.into_iter().collect::<Box<[_]>>(),
        )))
    }
}

/// Creates a shared `Rc<RefCell<[Motor; N]>>` from a list of motors.
#[macro_export]
macro_rules! shared_motors {
    ($($motor:expr),* $(,)?) => {
        ::std::rc::Rc::new(::std::cell::RefCell::new([$($motor),*]))
    };
}
