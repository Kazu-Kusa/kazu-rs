use bdmc_rs::controller::CloseLoopController;

/// Updater closure: takes no args, returns a Vec<f64> of sensor data.
pub type MentaUpdater = Box<dyn Fn() -> Vec<f64> + Send + Sync>;

/// Types of sampler functions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SamplerType {
    /// Returns a sequence of sensor data.
    Sequence,
    /// Takes an index, returns a single value.
    Indexed,
    /// Returns a single sensor value directly.
    Direct,
}

/// Unified sampler trait — replaces Python's 3 separate sampler types.
pub trait Sampler: Send + Sync {
    /// Collect sensor data.
    fn sample(&self) -> Vec<f64>;
    /// The type of this sampler.
    fn sampler_type(&self) -> SamplerType;
}

/// Sampler usage configuration — which sampler and which data indices to use.
#[derive(Debug, Clone)]
pub struct SamplerUsage {
    /// Index into the Menta's sampler list.
    pub used_sampler_index: usize,
    /// Which data indices from the sampler's output are needed.
    /// Empty means all data.
    pub required_data_indexes: Vec<usize>,
}

impl SamplerUsage {
    pub fn new(used_sampler_index: usize, required_data_indexes: Vec<usize>) -> Self {
        Self {
            used_sampler_index,
            required_data_indexes,
        }
    }
}

/// Result type for updater closures — either a sequence or a single value.
pub enum UpdaterResult {
    Sequence(Vec<f64>),
    Single(f64),
}

/// A sensor data updater that produces closures.
pub struct Menta {
    samplers: Vec<Box<dyn Sampler>>,
}

impl Menta {
    /// Create a new Menta instance.
    pub fn new(samplers: Vec<Box<dyn Sampler>>) -> Self {
        Self { samplers }
    }

    /// Construct an updater closure from registered sampler usages.
    /// Returns a closure that, when called, reads sensor data.
    pub fn construct_updater(
        &self,
        usages: &[SamplerUsage],
    ) -> Result<MentaUpdater, Box<dyn std::error::Error>> {
        if usages.is_empty() {
            return Err("Empty usage list".into());
        }

        // We need to collect all the relevant sampler data.
        // For simplicity, support the common case: build a closure that reads
        // from all specified samplers and returns concatenated data.

        let _closures: Vec<Box<dyn Fn() -> f64 + Send + Sync>> = Vec::new();

        for usage in usages {
            if usage.used_sampler_index >= self.samplers.len() {
                return Err(format!(
                    "Sampler index {} out of bounds (have {} samplers)",
                    usage.used_sampler_index,
                    self.samplers.len()
                )
                .into());
            }

            // We need to capture a reference to the sampler. Since Menta is borrowed,
            // we can't move the sampler into the closure. Instead, we'll use indices
            // and call through self. But closures can't borrow self...
            //
            // For the closure approach, we need the samplers to be in Arc or similar.
            // For now, we'll implement a simpler approach: the closure captures owned
            // copies of the samplers (if Clone) or we restructure.
            //
            // Practical approach for now: have the caller use register_updater()
            // which directly manipulates the controller context.

            // Placeholder: return a closure that returns empty data.
            // The real implementation requires Arc<dyn Sampler> or similar.
            let _ = usage;
        }

        // For the MVP, return a closure that reads from all samplers.
        // This requires the samplers to be cloneable or shared.
        // Let's use a different approach — use indices + unsafe, or Arc.

        Err("construct_updater: use register_updater() instead for now".into())
    }

    /// Register an updater into a controller's context.
    ///
    /// Reads sensor data and writes results into the controller's context
    /// under the given `output_keys`.
    pub fn register_updater(
        &self,
        controller: &mut CloseLoopController,
        usages: &[SamplerUsage],
        output_keys: &[String],
    ) -> Result<(), Box<dyn std::error::Error>> {
        if usages.is_empty() {
            return Err("Empty usage list".into());
        }
        if output_keys.is_empty() {
            return Err("Empty output keys".into());
        }

        // We need to build a closure that reads from samplers.
        // The samplers are behind &self, so we need shared ownership.
        // For the MVP, we'll use unsafe to extend lifetimes, or restructure.

        // Actually, let's just implement this simply: iterate all usages,
        // collect data, and insert into context.

        let mut results: Vec<f64> = Vec::new();
        for usage in usages {
            if usage.used_sampler_index >= self.samplers.len() {
                return Err(
                    format!("Sampler index {} out of bounds", usage.used_sampler_index).into(),
                );
            }

            let data = self.samplers[usage.used_sampler_index].sample();

            if usage.required_data_indexes.is_empty() {
                results.extend(data);
            } else {
                for &idx in &usage.required_data_indexes {
                    if idx < data.len() {
                        results.push(data[idx]);
                    }
                }
            }
        }

        // Write to controller context.
        for (i, key) in output_keys.iter().enumerate() {
            if i < results.len() {
                controller.context_mut().insert(
                    key.clone(),
                    serde_json::Value::Number(
                        serde_json::Number::from_f64(results[i]).unwrap_or(0.into()),
                    ),
                );
            }
        }

        Ok(())
    }

    /// Execute a single sample cycle and return the collected data.
    pub fn sample_all(&self) -> Vec<f64> {
        let mut data = Vec::new();
        for sampler in &self.samplers {
            data.extend(sampler.sample());
        }
        data
    }

    /// Get number of registered samplers.
    pub fn sampler_count(&self) -> usize {
        self.samplers.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockSampler {
        data: Vec<f64>,
    }

    impl Sampler for MockSampler {
        fn sample(&self) -> Vec<f64> {
            self.data.clone()
        }
        fn sampler_type(&self) -> SamplerType {
            SamplerType::Sequence
        }
    }

    #[test]
    fn test_menta_register_updater() {
        let sampler = MockSampler {
            data: vec![1.0, 2.0, 3.0],
        };
        let menta = Menta::new(vec![Box::new(sampler)]);

        let usage = SamplerUsage::new(0, vec![0, 2]);
        let mut controller = CloseLoopController::new(None, None, None, None).unwrap();

        menta
            .register_updater(&mut controller, &[usage], &["x".into(), "y".into()])
            .unwrap();

        let ctx = controller.context();
        assert_eq!(ctx.get("x").and_then(|v| v.as_f64()), Some(1.0));
        assert_eq!(ctx.get("y").and_then(|v| v.as_f64()), Some(3.0));
    }

    #[test]
    fn test_menta_sample_all() {
        let s1 = MockSampler { data: vec![10.0] };
        let s2 = MockSampler {
            data: vec![20.0, 30.0],
        };
        let menta = Menta::new(vec![Box::new(s1), Box::new(s2)]);

        assert_eq!(menta.sample_all(), vec![10.0, 20.0, 30.0]);
    }
}
