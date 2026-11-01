#![deny(unused_variables)]

use std::cell::Cell;

#[test]
fn debug_log_checks_captured_arguments_in_every_profile() {
    let context = "device failure";
    ozton_record::log!("context={context}");
}

#[test]
fn debug_log_evaluates_arguments_only_in_debug_builds() {
    let evaluated = Cell::new(false);
    ozton_record::log!("evaluated={}", {
        evaluated.set(true);
        true
    });
    assert_eq!(evaluated.get(), cfg!(debug_assertions));
}
