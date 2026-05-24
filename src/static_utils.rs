//! Utility functions — port of `kazu/static.py`.
#![allow(dead_code)]

use crate::config::TagGroup;
use std::collections::HashMap;

/// Build a tag query table for vision-based object classification.
///
/// Maps (tag_id, is_enemy) pairs to action codes used by the surrounding handler.
pub fn make_query_table(tag_group: &TagGroup) -> HashMap<(i32, bool), i32> {
    let mut table = HashMap::new();

    // Allied tags → not enemy
    table.insert((tag_group.ally_tag, false), 0);
    // Enemy tags → enemy
    table.insert((tag_group.enemy_tag, true), 1);
    // Neutral tags → not enemy
    table.insert((tag_group.neutral_tag, false), 2);
    // Default tag → not enemy (safe default)
    table.insert((tag_group.default_tag, false), 3);

    table
}

/// Get a formatted timestamp string (YYYY-MM-DD-HH-MM-SS-ms).
pub fn get_timestamp() -> String {
    // Simple stub — in production, use chrono crate
    "0000-00-00-00-00-00-000".to_string()
}
