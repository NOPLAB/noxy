// Execution mode definitions and switching logic

use crate::physics::simulation::{PhysicsSimulation, SimulationConfig};
use anyhow::Result;
use std::path::PathBuf;

/// Application execution modes
#[derive(Debug, Clone, PartialEq)]
pub enum ExecutionMode {
    /// Interactive mode with 3D visualization and user input
    Interactive {
        /// Enable real-time physics visualization
        show_physics: bool,
        /// Target frame rate for rendering
        target_fps: u32,
    },
    /// Headless mode for compute-only simulation
    Headless {
        /// Number of simulation steps to run
        steps: u32,
        /// Output directory for results
        output_path: Option<PathBuf>,
        /// File format for output data
        output_format: OutputFormat,
    },
}

/// Output formats for headless mode
#[derive(Debug, Clone, PartialEq)]
pub enum OutputFormat {
    /// JSON format with position/velocity data
    Json,
    /// CSV format for analysis
    Csv,
    /// Binary format for performance
    Binary,
}

impl Default for ExecutionMode {
    fn default() -> Self {
        Self::Interactive {
            show_physics: true,
            target_fps: 60,
        }
    }
}

/// Mode-specific configuration
#[derive(Debug, Clone)]
pub struct ModeConfig {
    pub execution_mode: ExecutionMode,
    pub simulation_config: SimulationConfig,
}

impl Default for ModeConfig {
    fn default() -> Self {
        Self {
            execution_mode: ExecutionMode::default(),
            simulation_config: SimulationConfig::default(),
        }
    }
}

/// Mode manager handles switching between execution modes
pub struct ModeManager {
    current_mode: ExecutionMode,
    simulation: Option<PhysicsSimulation>,
}

impl ModeManager {
    /// Create a new mode manager
    pub fn new(mode: ExecutionMode) -> Self {
        Self {
            current_mode: mode,
            simulation: None,
        }
    }

    /// Initialize the mode with simulation
    pub fn initialize(&mut self, config: ModeConfig) -> Result<()> {
        self.current_mode = config.execution_mode;
        self.simulation = Some(PhysicsSimulation::with_config(config.simulation_config)?);
        Ok(())
    }

    /// Get the current execution mode
    pub fn current_mode(&self) -> &ExecutionMode {
        &self.current_mode
    }

    /// Check if running in interactive mode
    pub fn is_interactive(&self) -> bool {
        matches!(self.current_mode, ExecutionMode::Interactive { .. })
    }

    /// Check if running in headless mode
    pub fn is_headless(&self) -> bool {
        matches!(self.current_mode, ExecutionMode::Headless { .. })
    }

    /// Get mutable reference to the simulation
    pub fn simulation_mut(&mut self) -> Option<&mut PhysicsSimulation> {
        self.simulation.as_mut()
    }

    /// Get reference to the simulation
    pub fn simulation(&self) -> Option<&PhysicsSimulation> {
        self.simulation.as_ref()
    }

    /// Switch to a different execution mode
    pub fn switch_mode(&mut self, new_mode: ExecutionMode, config: SimulationConfig) -> Result<()> {
        // Save current simulation state if needed
        let current_stats = self.simulation.as_ref().map(|sim| sim.statistics());

        // Create new simulation with the same configuration
        self.simulation = Some(PhysicsSimulation::with_config(config)?);
        self.current_mode = new_mode;

        if let Some(stats) = current_stats {
            log::info!(
                "Switched modes, previous simulation had {} rigid bodies using {} backend",
                stats.rigidbody_count,
                stats.backend_name
            );
        }

        Ok(())
    }

    /// Run the simulation in the current mode
    pub fn run(&mut self) -> Result<ModeResult> {
        match &self.current_mode {
            ExecutionMode::Interactive {
                show_physics,
                target_fps,
            } => self.run_interactive(*show_physics, *target_fps),
            ExecutionMode::Headless {
                steps,
                output_path,
                output_format,
            } => self.run_headless(*steps, output_path.clone(), output_format.clone()),
        }
    }

    /// Run in interactive mode
    fn run_interactive(&mut self, _show_physics: bool, _target_fps: u32) -> Result<ModeResult> {
        // This would integrate with the existing winit/wgpu rendering system
        // For now, just indicate that interactive mode is ready
        log::info!("Interactive mode initialized and ready");

        Ok(ModeResult::Interactive {
            frames_rendered: 0,
            simulation_steps: 0,
        })
    }

    /// Run in headless mode
    fn run_headless(
        &mut self,
        steps: u32,
        output_path: Option<PathBuf>,
        output_format: OutputFormat,
    ) -> Result<ModeResult> {
        let simulation = self
            .simulation
            .as_mut()
            .ok_or_else(|| anyhow::anyhow!("Simulation not initialized"))?;

        log::info!("Starting headless simulation for {} steps", steps);

        let dt = 1.0 / 60.0; // 60 FPS equivalent time step
        let mut positions_over_time = Vec::new();

        // Run simulation steps
        for step in 0..steps {
            simulation.step(dt)?;

            // Collect data every 10 steps to avoid excessive memory usage
            if step % 10 == 0 {
                let ids = simulation.rigidbody_ids();
                let mut step_data = Vec::new();

                for id in ids {
                    if let (Ok(pos), Ok(vel)) = (
                        simulation.get_rigidbody_position(id),
                        simulation.get_rigidbody_velocity(id),
                    ) {
                        step_data.push(RigidBodySnapshot {
                            id: id.id(),
                            position: pos,
                            velocity: vel,
                            time: step as f32 * dt,
                        });
                    }
                }
                positions_over_time.push(step_data);
            }

            if step % 1000 == 0 {
                log::info!("Completed {} / {} steps", step, steps);
            }
        }

        let stats = simulation.statistics();

        // Save results if output path is specified
        if let Some(path) = output_path {
            self.save_results(&positions_over_time, &path, &output_format)?;
        }
        log::info!(
            "Headless simulation completed. {} rigid bodies using {} backend",
            stats.rigidbody_count,
            stats.backend_name
        );

        Ok(ModeResult::Headless {
            steps_completed: steps,
            rigidbodies_simulated: stats.rigidbody_count,
            backend_used: stats.backend_name,
            data_points: positions_over_time.len(),
        })
    }

    /// Save simulation results to file
    fn save_results(
        &self,
        data: &[Vec<RigidBodySnapshot>],
        path: &PathBuf,
        format: &OutputFormat,
    ) -> Result<()> {
        std::fs::create_dir_all(path.parent().unwrap_or(path))?;

        match format {
            OutputFormat::Json => {
                let json_data = serde_json::to_string_pretty(data)?;
                std::fs::write(path.with_extension("json"), json_data)?;
                log::info!(
                    "Saved simulation data to {}",
                    path.with_extension("json").display()
                );
            }
            OutputFormat::Csv => {
                self.save_csv(data, &path.with_extension("csv"))?;
                log::info!(
                    "Saved simulation data to {}",
                    path.with_extension("csv").display()
                );
            }
            OutputFormat::Binary => {
                let binary_data = bincode::serialize(data)?;
                std::fs::write(path.with_extension("bin"), binary_data)?;
                log::info!(
                    "Saved simulation data to {}",
                    path.with_extension("bin").display()
                );
            }
        }

        Ok(())
    }

    /// Save data in CSV format
    fn save_csv(&self, data: &[Vec<RigidBodySnapshot>], path: &PathBuf) -> Result<()> {
        use std::io::Write;

        let mut file = std::fs::File::create(path)?;
        writeln!(
            file,
            "time,rigidbody_id,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z"
        )?;

        for step_data in data {
            for snapshot in step_data {
                writeln!(
                    file,
                    "{},{},{},{},{},{},{},{}",
                    snapshot.time,
                    snapshot.id,
                    snapshot.position.x,
                    snapshot.position.y,
                    snapshot.position.z,
                    snapshot.velocity.x,
                    snapshot.velocity.y,
                    snapshot.velocity.z
                )?;
            }
        }

        Ok(())
    }
}

/// Result of running a mode
#[derive(Debug)]
pub enum ModeResult {
    Interactive {
        frames_rendered: u64,
        simulation_steps: u64,
    },
    Headless {
        steps_completed: u32,
        rigidbodies_simulated: usize,
        backend_used: String,
        data_points: usize,
    },
}

/// Snapshot of rigid body state at a specific time
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RigidBodySnapshot {
    pub id: u64,
    pub position: glam::Vec3,
    pub velocity: glam::Vec3,
    pub time: f32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::backends::traits::BackendSelection;

    #[test]
    fn test_mode_creation() {
        let mode = ExecutionMode::Interactive {
            show_physics: true,
            target_fps: 60,
        };

        let manager = ModeManager::new(mode);
        assert!(manager.is_interactive());
        assert!(!manager.is_headless());
    }

    #[test]
    fn test_mode_switching() {
        let interactive_mode = ExecutionMode::Interactive {
            show_physics: true,
            target_fps: 60,
        };

        let mut manager = ModeManager::new(interactive_mode);

        let config = SimulationConfig {
            backend_selection: BackendSelection::Cpu,
            max_rigidbodies: 100,
            gravity: glam::Vec3::new(0.0, -9.81, 0.0),
        };

        manager
            .initialize(ModeConfig {
                execution_mode: ExecutionMode::Interactive {
                    show_physics: true,
                    target_fps: 60,
                },
                simulation_config: config.clone(),
            })
            .unwrap();

        assert!(manager.is_interactive());

        // Switch to headless mode
        let headless_mode = ExecutionMode::Headless {
            steps: 100,
            output_path: None,
            output_format: OutputFormat::Json,
        };

        manager.switch_mode(headless_mode, config).unwrap();
        assert!(manager.is_headless());
    }

    #[test]
    fn test_headless_simulation() {
        let temp_dir = std::env::temp_dir();
        let output_path = temp_dir.join("test_output");

        let headless_mode = ExecutionMode::Headless {
            steps: 50,
            output_path: Some(output_path.clone()),
            output_format: OutputFormat::Json,
        };

        let mut manager = ModeManager::new(headless_mode);

        let config = ModeConfig {
            execution_mode: ExecutionMode::Headless {
                steps: 50,
                output_path: Some(output_path),
                output_format: OutputFormat::Json,
            },
            simulation_config: SimulationConfig::default(),
        };

        manager.initialize(config).unwrap();

        // Add some rigid bodies for testing
        if let Some(sim) = manager.simulation_mut() {
            use crate::physics::simulation::RigidBodyProperties;

            for i in 0..5 {
                let props = RigidBodyProperties {
                    position: glam::Vec3::new(i as f32, 10.0, 0.0),
                    velocity: glam::Vec3::ZERO,
                    mass: 1.0,
                };
                sim.add_rigidbody(props).unwrap();
            }
        }

        let result = manager.run().unwrap();

        if let ModeResult::Headless {
            steps_completed,
            rigidbodies_simulated,
            ..
        } = result
        {
            assert_eq!(steps_completed, 50);
            assert_eq!(rigidbodies_simulated, 5);
        } else {
            panic!("Expected headless result");
        }
    }
}
