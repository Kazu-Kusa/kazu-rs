use std::collections::{HashMap, HashSet};
use std::fs;
use std::path::Path;
use crate::state::ArrowStyle;
use crate::transition::MovingTransition;

/// Export a graph structure as a PlantUML file matching Python `Botix.export_structure()`.
pub fn export_structure(
    save_path: &Path,
    transitions: &[MovingTransition],
    arrow_style: ArrowStyle,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut lines: Vec<String> = Vec::new();
    lines.push("@startuml".to_string());

    let mut all_ids: HashSet<usize> = HashSet::new();
    let mut indegree: HashMap<usize, usize> = HashMap::new();
    let mut outdegree: HashMap<usize, usize> = HashMap::new();

    for t in transitions {
        for &sid in &t.from_states {
            all_ids.insert(sid);
            *outdegree.entry(sid).or_insert(0) += 1;
            indegree.entry(sid).or_insert(0);
        }
        for &sid in t.to_states.values() {
            all_ids.insert(sid);
            *indegree.entry(sid).or_insert(0) += 1;
            outdegree.entry(sid).or_insert(0);
        }
    }

    let start_states: HashSet<usize> = indegree
        .iter()
        .filter(|(_, deg)| **deg == 0)
        .map(|(&id, _)| id)
        .collect();
    let end_states: HashSet<usize> = outdegree
        .iter()
        .filter(|(_, deg)| **deg == 0)
        .map(|(&id, _)| id)
        .collect();

    let arrow = arrow_style.as_str();

    // Start markers.
    for &sid in &start_states {
        lines.push(format!("[*] {} s{}", arrow, sid));
    }
    lines.push(String::new());

    // State declarations.
    for &sid in &all_ids {
        lines.push(format!("state \"State({})\" as s{}", sid, sid));
    }
    lines.push(String::new());

    // Transitions.
    for t in transitions {
        if t.to_states.len() <= 1 {
            for &from_id in &t.from_states {
                if let Some(&to_id) = t.to_states.values().next() {
                    let label = if t.breaker.is_some() {
                        format!("{:.3}s [breaker]", t.duration)
                    } else {
                        format!("{:.3}s", t.duration)
                    };
                    lines.push(format!("s{} {} s{} : {}", from_id, arrow, to_id, label));
                }
            }
        } else {
            let break_node = format!("choice_t{}", t.id());
            lines.push(format!("state {} <<choice>>", break_node));
            for &from_id in &t.from_states {
                lines.push(format!(
                    "s{} {} {} : {:.3}s",
                    from_id, arrow, break_node, t.duration
                ));
            }
            for (key, &to_id) in &t.to_states {
                lines.push(format!("{} {} s{} : {:?}", break_node, arrow, to_id, key));
            }
        }
    }

    lines.push(String::new());

    // End markers.
    for &sid in &end_states {
        lines.push(format!("s{} {} [*]", sid, arrow));
    }

    lines.push("@enduml".to_string());
    fs::write(save_path, lines.join("\n"))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transition::BreakerResult;
    use std::path::PathBuf;

    #[test]
    fn test_export_simple() {
        let t = MovingTransition::new(1.0)
            .unwrap()
            .with_from_state(0)
            .with_single_to_state(1);

        let path = PathBuf::from("test_export.puml");
        export_structure(&path, &[t], ArrowStyle::Down).unwrap();

        let content = fs::read_to_string(&path).unwrap();
        assert!(content.contains("@startuml"));
        assert!(content.contains("[*] --> s0"));
        assert!(content.contains("s1 --> [*]"));
        assert!(content.contains("@enduml"));

        let _ = fs::remove_file(&path);
    }

    #[test]
    fn test_export_branching_demo() {
        // Python test_compile_with_branches structure:
        //   s100 -> breaker{1,2,3} -> s200|s300|s400
        //   s400 -> breaker{1,2}   -> s500|s600

        let t_a_bcd = MovingTransition::new(1.0)
            .unwrap()
            .with_from_state(100)
            .with_to_state(BreakerResult::Int(1), 200)
            .with_to_state(BreakerResult::Int(2), 300)
            .with_to_state(BreakerResult::Int(3), 400);

        let t_d_ef = MovingTransition::new(0.5)
            .unwrap()
            .with_from_state(400)
            .with_to_state(BreakerResult::Int(1), 500)
            .with_to_state(BreakerResult::Int(2), 600);

        let path = PathBuf::from("demo_branching.puml");
        export_structure(&path, &[t_a_bcd, t_d_ef], ArrowStyle::Down).unwrap();

        let content = fs::read_to_string(&path).unwrap();
        println!("=== Generated ===\n{}\n=== End ===", content);

        assert!(content.contains("[*] --> s100"));
        assert!(content.contains("choice_t"));
        assert!(content.contains("s600 --> [*]"));
    }
}
