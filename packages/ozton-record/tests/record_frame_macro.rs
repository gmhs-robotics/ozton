use ozton_record::{Interpolate, record_frame};

record_frame! {
    struct NestedFrame {
        x: f64,
        enabled: bool,
    }
}

record_frame! {
    struct OuterFrame {
        nested: NestedFrame,
        power: f64,
    }
}

#[test]
fn record_frame_macro_generates_interpolation() {
    let from = OuterFrame {
        nested: NestedFrame {
            x: 0.0,
            enabled: false,
        },
        power: 0.0,
    };
    let to = OuterFrame {
        nested: NestedFrame {
            x: 10.0,
            enabled: true,
        },
        power: 1.0,
    };

    let mid = OuterFrame::interpolate(&from, &to, 0.25);
    assert!((mid.nested.x - 2.5).abs() < 1e-9);
    assert!(!mid.nested.enabled);
    assert!((mid.power - 0.25).abs() < 1e-9);
}
