//! Shared handles for actuator and encoder access to the same motor objects.

use std::{cell::RefCell, rc::Rc};

use vexide::prelude::Motor;

/// Shared motor storage in wiring order. An array can coerce to a slice
/// without copying its motors or allocating a second storage object.
pub type SharedMotors = Rc<RefCell<[Motor]>>;

/// Register an existing group for diagnostic sampling without taking ownership.
#[doc(hidden)]
pub fn register_motor_group(_motors: &SharedMotors) {
    #[cfg(feature = "logging")]
    register(_motors);
}

/// Creates a shared `Rc<RefCell<[Motor; N]>>` from a list of motors.
#[macro_export]
macro_rules! shared_motors {
    ($($motor:expr),* $(,)?) => {
        ::std::rc::Rc::new(::std::cell::RefCell::new([$($motor),*]))
    };
}

#[cfg(feature = "logging")]
std::thread_local! {
    static GROUPS: RefCell<Vec<std::rc::Weak<RefCell<[Motor]>>>> = const { RefCell::new(Vec::new()) };
}

#[cfg(feature = "logging")]
fn register(storage: &SharedMotors) {
    let weak = Rc::downgrade(storage);
    GROUPS.with(|groups| {
        let mut groups = groups.borrow_mut();
        groups.retain(|entry| entry.strong_count() != 0);
        if !groups.iter().any(|entry| entry.ptr_eq(&weak)) {
            groups.push(weak);
        }
    });
}

/// Snapshot software command state without constructing another Motor or
/// reading a borrowed group. The registry never keeps hardware alive.
#[cfg(feature = "logging")]
pub(crate) fn command_targets() -> [Option<vexide::smart::motor::MotorControl>; 21] {
    use vexide::smart::SmartDevice;
    let mut targets = [None; 21];
    GROUPS.with(|groups| {
        let mut groups = groups.borrow_mut();
        groups.retain(|entry| {
            let Some(storage) = entry.upgrade() else {
                return false;
            };
            if let Ok(motors) = storage.try_borrow() {
                for motor in motors.iter() {
                    if let Some(target) =
                        targets.get_mut(usize::from(motor.port_number().saturating_sub(1)))
                    {
                        *target = Some(motor.target());
                    }
                }
            }
            true
        });
    });
    targets
}
