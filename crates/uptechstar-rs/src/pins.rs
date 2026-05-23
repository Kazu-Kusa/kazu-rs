//! Pin setter/getter/mode-setter constructor utilities.
//!
//! Port of `pyuptech/modules/pins.py`.

use std::rc::Rc;

/// A function that sets a pin level given a value.
pub type PinSetter = Rc<dyn Fn(i32)>;
/// A function that gets a pin level, returning the value.
pub type PinGetter = Rc<dyn Fn() -> i32>;
/// A function that sets a pin mode given a value.
pub type PinModeSetter = Rc<dyn Fn(i32)>;

/// Construct a `PinSetter` from an indexed setter and a specific pin.
///
/// The returned closure calls `indexed_setter(pin, level)`.
pub fn pin_setter_constructor<F>(indexed_setter: F, pin: i32) -> PinSetter
where
    F: Fn(i32, i32) + 'static,
{
    let setter = Rc::new(indexed_setter);
    Rc::new(move |level: i32| {
        setter(pin, level);
    })
}

/// Construct a `PinGetter` from an indexed getter and a specific pin.
///
/// The returned closure calls `indexed_getter(pin)`.
pub fn pin_getter_constructor<F>(indexed_getter: F, pin: i32) -> PinGetter
where
    F: Fn(i32) -> i32 + 'static,
{
    let getter = Rc::new(indexed_getter);
    Rc::new(move || getter(pin))
}

/// Construct a `PinModeSetter` from an indexed mode setter and a specific pin.
///
/// The returned closure calls `indexed_mode_setter(pin, mode)`.
pub fn pin_mode_setter_constructor<F>(indexed_mode_setter: F, pin: i32) -> PinModeSetter
where
    F: Fn(i32, i32) + 'static,
{
    let setter = Rc::new(indexed_mode_setter);
    Rc::new(move |mode: i32| {
        setter(pin, mode);
    })
}

/// Construct a `PinModeSetter` that sets the mode for multiple pins at once.
///
/// The returned closure calls `indexed_mode_setter(p, mode)` for each pin in `pins`.
pub fn multiple_pin_mode_setter_constructor<F>(
    indexed_mode_setter: F,
    pins: &[i32],
) -> PinModeSetter
where
    F: Fn(i32, i32) + 'static,
{
    let setter = Rc::new(indexed_mode_setter);
    let owned_pins = pins.to_vec();
    Rc::new(move |mode: i32| {
        for &pin in &owned_pins {
            setter(pin, mode);
        }
    })
}
