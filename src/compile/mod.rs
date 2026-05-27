//! Robot behavior handlers — compiles run config into Botix state graphs.
//!
//! Each handler function corresponds to a robot behavior (edge detection,
//! search, battle, navigation, etc.) and returns the transitions that define
//! the state machine for that behavior.
//!
//! Port of `kazu/compile.py`.

mod battle;
mod edge;
mod fence;
mod navigation;
mod scan;
mod stage;
mod surrounding;

pub use battle::{
    make_always_off_stage_battle_handler, make_always_on_stage_battle_handler,
    make_std_battle_handler,
};
pub use edge::make_edge_handler;
pub use fence::make_fence_handler;
pub use navigation::{
    make_align_direction_handler, make_back_to_stage_handler, make_rand_walk_handler,
    make_reboot_handler,
};
pub use scan::{make_scan_handler, make_search_handler};
pub use stage::make_on_stage_handler;
pub use surrounding::make_surrounding_handler;

use crate::config::{AppConfig, RunConfig};
use mentabotix_rs::{
    state::{FixedAxis, MovingState, TurnDirection},
    transition::{BreakerResult, MovingTransition},
};

/// Result from a handler: the transitions defining the behavior graph
/// Result from a handler: the transitions defining the behavior graph
/// plus the start and normal-exit anchors.
pub struct HandlerOutput {
    pub start_state: MovingState,
    pub normal_exit: MovingState,
    pub transitions: Vec<MovingTransition>,
}

// ── Helpers ───────────────────────────────────────────────────

pub(crate) fn halt_state() -> MovingState {
    MovingState::halt()
}
pub(crate) fn continues_state() -> MovingState {
    MovingState::straight(0)
}

pub(crate) fn make_straight(speed: i32) -> MovingState {
    MovingState::straight(speed)
}
pub(crate) fn make_turn_l(speed: i32) -> MovingState {
    MovingState::turn(TurnDirection::Left, speed)
}
pub(crate) fn make_turn_r(speed: i32) -> MovingState {
    MovingState::turn(TurnDirection::Right, speed)
}
pub(crate) fn make_drift_rl(speed: i32) -> MovingState {
    MovingState::drift(FixedAxis::RearLeft, speed)
}
pub(crate) fn make_drift_rr(speed: i32) -> MovingState {
    MovingState::drift(FixedAxis::RearRight, speed)
}

pub(crate) fn make_trans(
    dur: f64,
    breaker: Option<std::sync::Arc<dyn Fn() -> BreakerResult + Send + Sync>>,
) -> MovingTransition {
    let mut t = MovingTransition::new(dur).unwrap();
    if let Some(b) = breaker {
        t = t.with_arc_breaker(b);
    }
    t
}

pub(crate) fn make_trans_no_breaker(dur: f64) -> MovingTransition {
    MovingTransition::new(dur).unwrap()
}

// ── Dispatch table ────────────────────────────────────────────

/// All available handler names.
pub fn all_handler_names() -> &'static [&'static str] {
    &[
        "edge", "surr", "scan", "search", "fence", "boot", "bkstage", "rdwalk", "align", "stdbat",
        "onstage", "angbat", "afgbat",
    ]
}
/// Full dispatch table: returns complete `HandlerOutput` with state objects
/// (used by viz to extract speed labels).
pub type HandlerFullFn = fn(&AppConfig, &RunConfig) -> HandlerOutput;

/// Dispatch table mapping pack names to handler functions returning `HandlerOutput`.
pub fn get_handler_full(name: &str) -> Option<HandlerFullFn> {
    match name {
        "edge" => Some(|ac, rc| make_edge_handler(ac, rc, None, None, None)),
        "surr" => Some(|ac, rc| make_surrounding_handler(ac, rc, None, None, None)),
        "scan" => Some(|ac, rc| make_scan_handler(ac, rc, None)),
        "search" => Some(|ac, rc| make_search_handler(ac, rc, None, None, None)),
        "fence" => Some(|ac, rc| make_fence_handler(ac, rc, None, None)),
        "boot" => Some(|ac, rc| make_reboot_handler(ac, rc, None)),
        "bkstage" => Some(|ac, rc| make_back_to_stage_handler(ac, rc, None)),
        "rdwalk" => Some(|ac, rc| make_rand_walk_handler(ac, rc, None)),
        "stdbat" => Some(make_std_battle_handler),
        "onstage" => Some(make_always_on_stage_battle_handler),
        "angbat" => Some(make_always_on_stage_battle_handler),
        "afgbat" => Some(make_always_off_stage_battle_handler),
        "align" => Some(|ac, rc| make_align_direction_handler(ac, rc, None)),
        _ => None,
    }
}
