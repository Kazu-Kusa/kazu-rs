//! Sensor weights, code signs, and run-mode constants.
//!
//! Port of `kazu/constant.py` — frozen dataclasses become unit structs with
//! associated constants; `IntEnum` types become `#[repr(i32)]` C-like enums.
//!
//! Variant names preserve Python O_X_O_O conventions — not Rust camelCase.
#![allow(non_camel_case_types, dead_code, clippy::upper_case_acronyms)]

// ── Top-level constant ──────────────────────────────────────────

/// Quit sentinel used in interactive loops.
pub const QUIT: &str = "q";

// ── Activation ─────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum Activation {
    Activate = 1,
    Deactivate = 0,
}

impl Activation {
    pub fn as_bool(self) -> bool {
        matches!(self, Activation::Activate)
    }
}

// ── Attitude ───────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Attitude;

impl Attitude {
    pub const PITCH: i32 = 0;
    pub const ROLL: i32 = 1;
    pub const YAW: i32 = 2;
}

// ── Axis ───────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Axis;

impl Axis {
    pub const X: i32 = 0;
    pub const Y: i32 = 1;
    pub const Z: i32 = 2;
}

// ── RunMode ────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RunMode;

impl RunMode {
    pub const AFG: &str = "AFG";
    pub const ANG: &str = "ANG";
    pub const NGS: &str = "NGS";
    pub const FGS: &str = "FGS";
    pub const FGDL: &str = "FGDL";

    pub fn export() -> &'static [&'static str] {
        &[Self::AFG, Self::ANG, Self::NGS, Self::FGS, Self::FGDL]
    }
}

// ── EdgeWeights ────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeWeights;

impl EdgeWeights {
    pub const FL: i32 = 1;
    pub const FR: i32 = 2;
    pub const RL: i32 = 4;
    pub const RR: i32 = 8;

    /// Return the standard weight sequence: (FL, RL, RR, FR).
    pub fn export_std_weight_seq() -> (i32, i32, i32, i32) {
        (Self::FL, Self::RL, Self::RR, Self::FR)
    }
}

// ── FenceWeights ───────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FenceWeights;

impl FenceWeights {
    pub const FRONT: i32 = 1;
    pub const REAR: i32 = 2;
    pub const LEFT: i32 = 4;
    pub const RIGHT: i32 = 8;
}

// ── ScanWeights ────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScanWeights;

impl ScanWeights {
    pub const FRONT: i32 = 1;
    pub const REAR: i32 = 2;
    pub const LEFT: i32 = 4;
    pub const RIGHT: i32 = 8;
}

// ── SurroundingWeights ─────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SurroundingWeights;

impl SurroundingWeights {
    pub const LEFT_OBJECT: i32 = 1;
    pub const RIGHT_OBJECT: i32 = 2;
    pub const BEHIND_OBJECT: i32 = 4;
    pub const FRONT_ENEMY_CAR: i32 = 400;
    pub const FRONT_ENEMY_BOX: i32 = 300;
    pub const FRONT_NEUTRAL_BOX: i32 = 200;
    pub const FRONT_ALLY_BOX: i32 = 100;
    pub const NOTHING: i32 = 0;
}

// ── StageWeight ────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StageWeight;

impl StageWeight {
    pub const STAGE: i32 = 1;
    pub const REBOOT: i32 = 2;
    pub const UNCLEAR: i32 = 4;
}

// ── EdgeCodeSign ───────────────────────────────────────────────
/// Bitmask enum for edge-detection sensor triggers.
///
/// Each variant corresponds to a combination of the four edge-weight regions
/// (FL=1, FR=2, RL=4, RR=8).  Variant name `X_O_O_O` reads "FL triggered,
/// others clear", etc.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum EdgeCodeSign {
    O_O_O_O = 0,
    X_O_O_O = 1,  // FL
    O_O_O_X = 2,  // FR
    O_X_O_O = 4,  // RL
    O_O_X_O = 8,  // RR
    X_X_O_O = 5,  // FL + RL
    O_O_X_X = 10, // RR + FR
    X_O_X_O = 9,  // FL + RR
    X_O_O_X = 3,  // FR + FL
    O_X_X_O = 12, // RL + RR
    O_X_O_X = 6,  // RL + FR
    X_X_X_O = 13, // FL + RL + RR
    X_X_O_X = 7,  // FL + FR + RL
    X_O_X_X = 11, // FL + RR + FR
    O_X_X_X = 14, // RL + RR + FR
    X_X_X_X = 15, // all
}

// ── SurroundingCodeSign ────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum SurroundingCodeSign {
    // ── base ──
    NOTHING = 0,
    LEFT_OBJECT = 1,
    RIGHT_OBJECT = 2,
    LEFT_RIGHT_OBJECTS = 3,
    BEHIND_OBJECT = 4,
    LEFT_BEHIND_OBJECTS = 5,
    RIGHT_BEHIND_OBJECTS = 6,
    LEFT_RIGHT_BEHIND_OBJECTS = 7,

    // ── ally box ──
    FRONT_ALLY_BOX = 100,
    FRONT_ALLY_BOX_LEFT_OBJECT = 101,
    FRONT_ALLY_BOX_RIGHT_OBJECT = 102,
    FRONT_ALLY_BOX_LEFT_RIGHT_OBJECTS = 103,
    FRONT_ALLY_BOX_BEHIND_OBJECT = 104,
    FRONT_ALLY_BOX_LEFT_BEHIND_OBJECTS = 105,
    FRONT_ALLY_BOX_RIGHT_BEHIND_OBJECTS = 106,
    FRONT_ALLY_BOX_LEFT_RIGHT_BEHIND_OBJECTS = 107,

    // ── neutral box ──
    FRONT_NEUTRAL_BOX = 200,
    FRONT_NEUTRAL_BOX_LEFT_OBJECT = 201,
    FRONT_NEUTRAL_BOX_RIGHT_OBJECT = 202,
    FRONT_NEUTRAL_BOX_LEFT_RIGHT_OBJECTS = 203,
    FRONT_NEUTRAL_BOX_BEHIND_OBJECT = 204,
    FRONT_NEUTRAL_BOX_LEFT_BEHIND_OBJECTS = 205,
    FRONT_NEUTRAL_BOX_RIGHT_BEHIND_OBJECTS = 206,
    FRONT_NEUTRAL_BOX_LEFT_RIGHT_BEHIND_OBJECTS = 207,

    // ── enemy box ──
    FRONT_ENEMY_BOX = 300,
    FRONT_ENEMY_BOX_LEFT_OBJECT = 301,
    FRONT_ENEMY_BOX_RIGHT_OBJECT = 302,
    FRONT_ENEMY_BOX_LEFT_RIGHT_OBJECTS = 303,
    FRONT_ENEMY_BOX_BEHIND_OBJECT = 304,
    FRONT_ENEMY_BOX_LEFT_BEHIND_OBJECTS = 305,
    FRONT_ENEMY_BOX_RIGHT_BEHIND_OBJECTS = 306,
    FRONT_ENEMY_BOX_LEFT_RIGHT_BEHIND_OBJECTS = 307,

    // ── enemy car ──
    FRONT_ENEMY_CAR = 400,
    FRONT_ENEMY_CAR_LEFT_OBJECT = 401,
    FRONT_ENEMY_CAR_RIGHT_OBJECT = 402,
    FRONT_ENEMY_CAR_LEFT_RIGHT_OBJECTS = 403,
    FRONT_ENEMY_CAR_BEHIND_OBJECT = 404,
    FRONT_ENEMY_CAR_LEFT_BEHIND_OBJECTS = 405,
    FRONT_ENEMY_CAR_RIGHT_BEHIND_OBJECTS = 406,
    FRONT_ENEMY_CAR_LEFT_RIGHT_BEHIND_OBJECTS = 407,
}

// ── StageCodeSign ──────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum StageCodeSign {
    ON_STAGE = 0,
    OFF_STAGE = 1,
    ON_STAGE_REBOOT = 2,
    OFF_STAGE_REBOOT = 3,
    UNCLEAR_ZONE = 4,
    UNCLEAR_ZONE_REBOOT = 6,
}

// ── FenceCodeSign ──────────────────────────────────────────────
/// Bitmask enum for fence-sensor triggers.
///
/// Weights: Front=1, Rear=2, Left=4, Right=8.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum FenceCodeSign {
    O_O_O_O = 0,
    X_O_O_O = 1,  // Front
    O_X_O_O = 2,  // Rear
    O_O_X_O = 4,  // Left
    O_O_O_X = 8,  // Right
    X_X_O_O = 3,
    X_O_X_O = 5,
    X_O_O_X = 9,
    O_X_X_O = 6,
    O_X_O_X = 10,
    O_O_X_X = 12,
    X_X_X_O = 7,
    X_X_O_X = 11,
    X_O_X_X = 13,
    O_X_X_X = 14,
    X_X_X_X = 15,
}

// ── ScanCodesign ───────────────────────────────────────────────
/// Bitmask enum for scan-sensor triggers.
///
/// Same weight/layout as [`FenceCodeSign`]: Front=1, Rear=2, Left=4, Right=8.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum ScanCodesign {
    O_O_O_O = 0,
    X_O_O_O = 1,  // Front
    O_X_O_O = 2,  // Rear
    O_O_X_O = 4,  // Left
    O_O_O_X = 8,  // Right
    X_X_O_O = 3,
    X_O_X_O = 5,
    X_O_O_X = 9,
    O_X_X_O = 6,
    O_X_O_X = 10,
    O_O_X_X = 12,
    X_X_X_O = 7,
    X_X_O_X = 11,
    X_O_X_X = 13,
    O_X_X_X = 14,
    X_X_X_X = 15,
}
