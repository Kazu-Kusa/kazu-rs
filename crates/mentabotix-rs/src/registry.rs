use std::collections::HashMap;
use std::hash::Hash;

/// Maps enum discriminants (or any hashable key) to state IDs for branching transitions.
pub struct CaseRegistry<K: Hash + Eq> {
    cases: HashMap<K, usize>,
}

impl<K: Hash + Eq> CaseRegistry<K> {
    /// Create a new empty CaseRegistry.
    pub fn new() -> Self {
        Self {
            cases: HashMap::new(),
        }
    }

    /// Register a case key → state ID mapping.
    /// Returns error if the case is already registered.
    pub fn register(&mut self, case: K, state_id: usize) -> Result<&mut Self, String> {
        if self.cases.contains_key(&case) {
            return Err(format!(
                "Case already registered: {:?}",
                std::any::type_name::<K>()
            ));
        }
        self.cases.insert(case, state_id);
        Ok(self)
    }

    /// Batch register multiple cases to the same state ID.
    pub fn batch_register(
        &mut self,
        cases: impl IntoIterator<Item = K>,
        state_id: usize,
    ) -> Result<&mut Self, String> {
        for case in cases {
            self.register(case, state_id)?;
        }
        Ok(self)
    }

    /// Unregister a case.
    pub fn unregister(&mut self, case: &K) -> Result<&mut Self, String> {
        if self.cases.remove(case).is_none() {
            return Err("Case not registered".into());
        }
        Ok(self)
    }

    /// Clear all registered cases.
    pub fn init_container(&mut self) -> &mut Self {
        self.cases.clear();
        self
    }

    /// Export the cases map and reset the registry.
    pub fn export(self) -> HashMap<K, usize> {
        self.cases
    }

    /// Get the number of registered cases.
    pub fn len(&self) -> usize {
        self.cases.len()
    }

    /// Check if the registry is empty.
    pub fn is_empty(&self) -> bool {
        self.cases.is_empty()
    }

    /// Get a reference to the cases map.
    pub fn cases(&self) -> &HashMap<K, usize> {
        &self.cases
    }
}

impl<K: Hash + Eq> Default for CaseRegistry<K> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_and_export() {
        let mut reg = CaseRegistry::new();
        reg.register("case_a".to_string(), 1).unwrap();
        reg.register("case_b".to_string(), 2).unwrap();

        let cases = reg.export();
        assert_eq!(cases.len(), 2);
        assert_eq!(cases.get("case_a"), Some(&1));
        assert_eq!(cases.get("case_b"), Some(&2));
    }

    #[test]
    fn test_duplicate_register() {
        let mut reg = CaseRegistry::new();
        reg.register(1, 100).unwrap();
        assert!(reg.register(1, 200).is_err());
    }

    #[test]
    fn test_batch_register() {
        let mut reg = CaseRegistry::new();
        reg.batch_register(vec![0, 1, 2], 42).unwrap();
        assert_eq!(reg.len(), 3);
    }
}
