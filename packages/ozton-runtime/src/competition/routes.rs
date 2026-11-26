/// A named autonomous choice dispatched by the robot's `OztonCompete` implementation.
#[derive(Clone, Copy, Debug)]
pub struct RobotRoute<A> {
    pub label: &'static str,
    pub id: A,
}

impl<A> RobotRoute<A> {
    pub const fn new(label: &'static str, id: A) -> Self {
        Self { label, id }
    }
}
