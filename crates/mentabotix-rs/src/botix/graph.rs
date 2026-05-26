use std::collections::{HashMap, HashSet, VecDeque};

use crate::state::MovingState;
use crate::transition::MovingTransition;

use super::Botix;

impl Botix {
    /// Compute the set of states reachable from `start` via forward edges.
    pub(crate) fn compute_reachable_set(
        states: &HashMap<usize, MovingState>,
        forward_edge: &HashMap<usize, usize>,
        transitions: &HashMap<usize, MovingTransition>,
        start: usize,
    ) -> HashSet<usize> {
        let mut visited = HashSet::new();
        let mut queue = VecDeque::new();
        queue.push_back(start);

        while let Some(current) = queue.pop_front() {
            if !visited.insert(current) {
                continue;
            }
            // Follow forward edge.
            if let Some(trans) = forward_edge.get(&current)
                .and_then(|&tid| transitions.get(&tid))
            {
                for &next_id in trans.to_states.values() {
                    if states.contains_key(&next_id) && !visited.contains(&next_id) {
                        queue.push_back(next_id);
                    }
                }
            }
        }

        visited
    }

    /// Validate the graph structure.
    pub fn validate(&self) -> Result<(), String> {
        if self.states.is_empty() {
            return Err("No states in graph".into());
        }
        if self.transitions.is_empty() {
            return Err("No transitions in graph".into());
        }
        Ok(())
    }

    /// Find loops in the graph via DFS.
    pub fn find_loops(&self) -> Vec<Vec<usize>> {
        let mut loops = Vec::new();
        let mut visited = HashSet::new();
        let mut path_stack: Vec<usize> = Vec::new();
        let mut path_set: HashSet<usize> = HashSet::new();

        // Use an explicit stack-based DFS to avoid recursion limits.
        // Each stack frame: (state_id, iterator_position).
        type Frame = (usize, Vec<usize>, usize); // (state, neighbors, next_index)
        let mut stack: Vec<Frame> = Vec::new();

        // Initialize with start state's neighbors.
        let init_neighbors: Vec<usize> = self
            .forward_edge
            .get(&self.start_state)
            .and_then(|&tid| self.transitions.get(&tid))
            .map(|t| t.to_states.values().copied().collect())
            .unwrap_or_default();

        path_stack.push(self.start_state);
        path_set.insert(self.start_state);
        visited.insert(self.start_state);
        stack.push((self.start_state, init_neighbors, 0));

        'outer: while let Some((current, ref neighbors, mut idx)) = stack.pop() {
            // Restore path to this point.
            while path_stack.last() != Some(&current) {
                let popped = path_stack.pop().unwrap();
                path_set.remove(&popped);
            }

            while idx < neighbors.len() {
                let next = neighbors[idx];
                idx += 1;

                if path_set.contains(&next) {
                    // Loop detected.
                    let loop_start = path_stack.iter().position(|&s| s == next).unwrap();
                    loops.push(path_stack[loop_start..].to_vec());
                    continue;
                }

                if visited.contains(&next) {
                    continue;
                }

                // Push current frame with updated index.
                stack.push((current, neighbors.clone(), idx));
                visited.insert(next);
                path_stack.push(next);
                path_set.insert(next);

                // Get next state's neighbors.
                let next_neighbors: Vec<usize> = self
                    .forward_edge
                    .get(&next)
                    .and_then(|&tid| self.transitions.get(&tid))
                    .map(|t| t.to_states.values().copied().collect())
                    .unwrap_or_default();

                stack.push((next, next_neighbors, 0));
                continue 'outer;
            }

            // All neighbors explored — backtrack.
            path_stack.pop();
            path_set.remove(&current);
        }

        loops
    }

    /// Get the IDs of start states (states with indegree 0).
    pub fn start_states(&self) -> HashSet<usize> {
        self.states
            .keys()
            .filter(|id| self.incoming_edges.get(id).is_none_or(|v| v.is_empty()))
            .copied()
            .collect()
    }

    /// Get the IDs of end states (states with no forward edge).
    pub fn end_states(&self) -> HashSet<usize> {
        self.states
            .keys()
            .filter(|id| !self.forward_edge.contains_key(id))
            .copied()
            .collect()
    }
}
