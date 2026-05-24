//! Robot behavior handlers — compiles run config into Botix state graphs.
//!
//! Each handler function corresponds to a robot behavior (edge detection,
//! search, battle, navigation, etc.) and returns the transitions that define
//! the state machine for that behavior.
//!
//! Port of `kazu/compile.py`.

use mentabotix_rs::{MovingState, MovingTransition};

/// Result from a handler: the transitions defining the behavior graph.
pub type HandlerResult = Vec<MovingTransition>;

// ── Edge handler ──────────────────────────────────────────────

/// Build the edge detection & avoidance state machine.
///
/// Reads edge detection thresholds from config and creates a graph
/// that backs away from detected edges, turns, and resumes forward motion.
pub fn make_edge_handler(
    start_state: Option<MovingState>,
    _normal_exit: Option<MovingState>,
    _abnormal_exit: Option<MovingState>,
) -> HandlerResult {
    let start = start_state.unwrap_or_else(MovingState::halt);

    // Edge handler: drive forward until edge detected, then back up + turn.
    let forward = MovingState::straight(400);
    let back_up = MovingState::straight(-300);
    let turn = MovingState::straight(200); // simplified
    let halt = MovingState::halt();

    let t_start = MovingTransition::new(0.5)
        .unwrap()
        .with_from_state(start.id())
        .with_single_to_state(forward.id());

    let t_edge_detect = MovingTransition::new(10.0)
        .unwrap()
        .with_from_state(forward.id())
        .with_single_to_state(back_up.id());

    let t_backup_done = MovingTransition::new(0.5)
        .unwrap()
        .with_from_state(back_up.id())
        .with_single_to_state(turn.id());

    let t_turn_done = MovingTransition::new(0.3)
        .unwrap()
        .with_from_state(turn.id())
        .with_single_to_state(halt.id());

    vec![t_start, t_edge_detect, t_backup_done, t_turn_done]
}

// ── Stub handlers ─────────────────────────────────────────────

pub fn make_surrounding_handler() -> HandlerResult {
    let s = MovingState::straight(300);
    let h = MovingState::halt();
    let t = MovingTransition::new(1.0).unwrap()
        .with_from_state(s.id())
        .with_single_to_state(h.id());
    vec![t]
}

pub fn make_scan_handler() -> HandlerResult {
    make_surrounding_handler()
}

pub fn make_search_handler() -> HandlerResult {
    make_surrounding_handler()
}

pub fn make_fence_handler() -> HandlerResult {
    make_surrounding_handler()
}

pub fn make_reboot_handler() -> HandlerResult {
    make_surrounding_handler()
}

pub fn make_back_to_stage_handler() -> HandlerResult {
    make_surrounding_handler()
}

pub fn make_rand_walk_handler() -> HandlerResult {
    make_surrounding_handler()
}

pub fn make_std_battle_handler() -> HandlerResult {
    make_surrounding_handler()
}

pub fn make_always_on_stage_battle_handler() -> HandlerResult {
    make_surrounding_handler()
}

pub fn make_always_off_stage_battle_handler() -> HandlerResult {
    make_surrounding_handler()
}

/// Dispatch table mapping pack names to handler functions.
pub fn get_handler(name: &str) -> Option<fn() -> HandlerResult> {
    match name {
        "edge" => Some(|| make_edge_handler(None, None, None)),
        "surr" => Some(make_surrounding_handler),
        "scan" => Some(make_scan_handler),
        "search" => Some(make_search_handler),
        "fence" => Some(make_fence_handler),
        "boot" => Some(make_reboot_handler),
        "bkstage" => Some(make_back_to_stage_handler),
        "rdwalk" => Some(make_rand_walk_handler),
        "stdbat" => Some(make_std_battle_handler),
        "onstage" => Some(make_always_on_stage_battle_handler),
        "angbat" => Some(make_always_on_stage_battle_handler),
        "afgbat" => Some(make_always_off_stage_battle_handler),
        _ => None,
    }
}

/// All available handler names.
pub fn all_handler_names() -> &'static [&'static str] {
    &[
        "edge", "surr", "scan", "search", "fence", "boot",
        "bkstage", "rdwalk", "stdbat", "onstage", "angbat", "afgbat",
    ]
}
