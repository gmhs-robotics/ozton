//! Recording choices for the shared competition selector.

pub use ozton_runtime::selector::{
    SelectionController, Selector as RecorderSelect, SelectorItem, SimpleSelectTheme, StatusHandle,
    THEME_DARK,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RecordTarget {
    #[default]
    Off,
    New,
    Overwrite(u32),
}
