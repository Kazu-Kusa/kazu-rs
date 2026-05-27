use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TagGroup {
    pub team_color: String,
    #[serde(default = "default_enemy_tag")]
    pub enemy_tag: i32,
    #[serde(default = "default_ally_tag")]
    pub ally_tag: i32,
    #[serde(default)]
    pub neutral_tag: i32,
    #[serde(default = "default_tag_id")]
    pub default_tag: i32,
}

fn default_enemy_tag() -> i32 {
    1
}
fn default_ally_tag() -> i32 {
    0
}
fn default_tag_id() -> i32 {
    0
}

impl Default for TagGroup {
    fn default() -> Self {
        Self {
            team_color: "blue".into(),
            enemy_tag: 1,
            ally_tag: 0,
            neutral_tag: 0,
            default_tag: 0,
        }
    }
}

impl TagGroup {
    pub fn new(team_color: &str) -> Self {
        let (enemy, ally) = match team_color {
            "online" => (1, 0),
            "yellow" => (1, 2),
            "blue" => (2, 1),
            _ => (1, 0),
        };
        Self {
            team_color: team_color.to_string(),
            enemy_tag: enemy,
            ally_tag: ally,
            neutral_tag: 0,
            default_tag: 0,
        }
    }
}
