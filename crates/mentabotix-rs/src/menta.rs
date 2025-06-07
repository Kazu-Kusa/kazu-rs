// Type aliases for Rust equivalents
pub type SensorData = f64; // Using f64 as the primary numeric type
pub type SensorDataSequence = Vec<SensorData>;
pub type SequenceClosure = Box<dyn Fn() -> SensorDataSequence>;
pub type SingleClosure = Box<dyn Fn() -> SensorData>;


// Union type for different sampler types
pub enum Sampler<SeqF, IdxF, DrcF>
where
    SeqF: Fn() -> SensorDataSequence,
    IdxF: Fn(usize) -> SensorData,
    DrcF: Fn() -> SensorData,
{
    Sequence(SeqF),
    Indexed(IdxF),
    Direct(DrcF),
}


// UpdaterClosure equivalent
pub enum UpdaterClosure {
    Sequence(SequenceClosure),
    Single(SingleClosure),
}

/// Sampler usage configuration
#[derive(Debug, Clone)]
pub struct SamplerUsage {
    pub used_sampler_index: usize,
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

/// A unified sensor data updater constructor.
pub struct Menta<SeqF, IdxF, DrcF>
where
    SeqF: Fn() -> SensorDataSequence,
    IdxF: Fn(usize) -> SensorData,
    DrcF: Fn() -> SensorData,
{
    samplers: Vec<Sampler<SeqF, IdxF, DrcF>>,
}

impl<SeqF, IdxF, DrcF> Menta<SeqF, IdxF, DrcF>
where
    SeqF: Fn() -> SensorDataSequence + Clone + 'static,
    IdxF: Fn(usize) -> SensorData + Clone + 'static,
    DrcF: Fn() -> SensorData + Clone + 'static,
{
    /// Create a new Menta instance
    pub fn new(samplers: Option<Vec<Sampler<SeqF, IdxF, DrcF>>>) -> Self {
        let samplers = samplers.unwrap_or_default();
        Self { samplers }
    }

    /// Construct an updater function based on the given list of sampler usages
    pub fn construct_updater(&self, usages: &[SamplerUsage]) -> Result<UpdaterClosure, Box<dyn std::error::Error>> {
        if usages.is_empty() {
            return Err("Can't resolve the empty usage list".into());
        }

        // For simplicity, we'll return the first usage resolver
        // In a full implementation, you'd need to handle combining multiple updaters
        let usage = &usages[0];

        if usage.used_sampler_index >= self.samplers.len() {
            return Err("Sampler index out of bounds".into());
        }

        match &self.samplers[usage.used_sampler_index] {
            Sampler::Sequence(sampler) => {
                Self::resolve_seq_sampler(sampler, &usage.required_data_indexes)
            }
            Sampler::Indexed(sampler) => {
                Self::resolve_idx_sampler(sampler, &usage.required_data_indexes)
            }
            Sampler::Direct(sampler) => {
                Self::resolve_drc_sampler(sampler, &usage.required_data_indexes)
            }
        }
    }

    /// Resolve sequence sampler based on required data indexes
    fn resolve_seq_sampler(
        sampler: &SeqF,
        required_data_indexes: &[usize],
    ) -> Result<UpdaterClosure, Box<dyn std::error::Error>> {
        match required_data_indexes.len() {
            0 => {
                // Return all data
                let sampler_clone = sampler.clone();
                Ok(UpdaterClosure::Sequence(Box::new(move || sampler_clone())))
            }
            1 => {
                // Return specific data
                let unique_index = required_data_indexes[0];
                let sampler_clone = sampler.clone();
                Ok(UpdaterClosure::Single(Box::new(move || {
                    let data = sampler_clone();
                    data[unique_index]
                })))
            }
            _ => {
                // Return multiple specific data
                let required_indexes = required_data_indexes.to_vec();
                let sampler_clone = sampler.clone();
                Ok(UpdaterClosure::Sequence(Box::new(move || {
                    let data = sampler_clone();
                    required_indexes.iter().map(|&i| data[i]).collect()
                })))
            }
        }
    }

    /// Resolve indexed sampler based on required data indexes
    fn resolve_idx_sampler(
        sampler: &IdxF,
        required_data_indexes: &[usize],
    ) -> Result<UpdaterClosure, Box<dyn std::error::Error>> {
        match required_data_indexes.len() {
            0 => Err("Must specify at least one required data index".into()),
            1 => {
                let unique_index = required_data_indexes[0];
                let sampler_clone = sampler.clone();
                Ok(UpdaterClosure::Single(Box::new(move || sampler_clone(unique_index))))
            }
            _ => {
                let required_indexes = required_data_indexes.to_vec();
                let sampler_clone = sampler.clone();
                Ok(UpdaterClosure::Sequence(Box::new(move || {
                    required_indexes.iter().map(|&i| sampler_clone(i)).collect()
                })))
            }
        }
    }

    /// Resolve direct sampler based on required data indexes
    fn resolve_drc_sampler(
        sampler: &DrcF,
        required_data_indexes: &[usize],
    ) -> Result<UpdaterClosure, Box<dyn std::error::Error>> {
        match required_data_indexes.len() {
            0 => {
                let sampler_clone = sampler.clone();
                Ok(UpdaterClosure::Single(Box::new(move || sampler_clone())))
            }
            1 => {
                let unique_index = required_data_indexes[0];
                let sampler_clone = sampler.clone();
                Ok(UpdaterClosure::Single(Box::new(move || {
                    let data = sampler_clone() as i64;
                    ((data >> unique_index) & 1) as f64
                })))
            }
            _ => {
                let required_indexes = required_data_indexes.to_vec();
                let sampler_clone = sampler.clone();
                Ok(UpdaterClosure::Sequence(Box::new(move || {
                    let temp_seq = sampler_clone() as i64;
                    required_indexes.iter().map(|&i| ((temp_seq >> i) & 1) as f64).collect()
                })))
            }
        }
    }
}