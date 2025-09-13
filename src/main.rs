mod app;
mod physics;
mod utils;

use std::env;

use app::Application;
use winit::event_loop::EventLoop;


use clap::{Parser, Subcommand, ValueEnum};
use physics::backends::traits::BackendSelection;

/// noxy - GPU-accelerated physics simulation
#[derive(Parser)]
#[command(name = "noxy")]
#[command(version = env!("CARGO_PKG_VERSION"))]
#[command(about = "GPU-accelerated physics simulation")]
#[command(long_about = "A high-performance physics simulator with CPU and GPU backend support.\n\
                       Supports rigidbody dynamics, robotics simulations, and machine learning environments.")]
pub struct Cli {
    /// Physics backend to use
    #[arg(long, short = 'b', default_value_t = BackendType::Auto)]
    pub backend: BackendType,

    /// Number of rigidbodies to simulate
    #[arg(long, short = 'r', default_value_t = 10)]
    pub rigidbodies: u32,

    /// Number of simulation steps (headless mode only)
    #[arg(long, short = 's', default_value_t = 1000)]
    pub steps: u32,

    /// Output file for results
    #[arg(long, short = 'o')]
    pub output: Option<String>,

    /// Simulation timestep in seconds
    #[arg(long, default_value_t = 0.016)]
    pub timestep: f32,

    /// Gravity magnitude (m/s^2)
    #[arg(long, default_value_t = 9.81)]
    pub gravity: f32,

    /// Enable performance profiling
    #[arg(long)]
    pub profile: bool,

    /// Verbosity level (0-3)
    #[arg(short, long, action = clap::ArgAction::Count)]
    pub verbose: u8,

    /// Configuration file path
    #[arg(long, short = 'c')]
    pub config: Option<String>,

    /// Seed for reproducible simulations
    #[arg(long)]
    pub seed: Option<u64>,

    /// Execution mode
    #[command(subcommand)]
    pub mode: Option<ModeCommand>,
}

impl Cli {
    /// Get the mode, defaulting to Auto if no subcommand is provided
    pub fn get_mode(&self) -> Mode {
        match &self.mode {
            Some(ModeCommand::Interactive { .. }) => Mode::Interactive,
            Some(ModeCommand::Headless { .. }) => Mode::Headless,
            None => Mode::Auto,
        }
    }
}

/// Execution modes
#[derive(Subcommand)]
pub enum ModeCommand {
    /// Run in interactive mode with 3D visualization
    Interactive {
        /// Enable debug UI
        #[arg(long)]
        debug: bool,
    },
    /// Run in headless mode for high-performance computation
    Headless {
        /// Export format
        #[arg(long, default_value_t = ExportFormat::Json)]
        format: ExportFormat,
    },
}

/// Simplified mode enum for internal use
#[derive(Debug, Clone)]
pub enum Mode {
    Interactive,
    Headless,
    Auto,
}

/// Physics backend types
#[derive(Debug, Clone, ValueEnum)]
pub enum BackendType {
    /// Auto-select the best available backend
    Auto,
    /// Use CPU backend
    Cpu,
    /// Use GPU backend
    Gpu,
}

impl From<BackendType> for BackendSelection {
    fn from(backend_type: BackendType) -> Self {
        match backend_type {
            BackendType::Auto => BackendSelection::Auto,
            BackendType::Cpu => BackendSelection::Cpu,
            BackendType::Gpu => BackendSelection::Gpu,
        }
    }
}
impl std::fmt::Display for BackendType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BackendType::Auto => write!(f, "auto"),
            BackendType::Cpu => write!(f, "cpu"),
            BackendType::Gpu => write!(f, "gpu"),
        }
    }
}

/// Export formats
#[derive(Debug, Clone, ValueEnum)]
pub enum ExportFormat {
    Json,
    Csv,
    Binary,
}
impl std::fmt::Display for ExportFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExportFormat::Json => write!(f, "json"),
            ExportFormat::Csv => write!(f, "csv"),
            ExportFormat::Binary => write!(f, "binary"),
        }
    }
}


fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    
    // Set up logging based on verbosity
    let log_level = match cli.verbose {
        0 => "warn",
        1 => "info",
        2 => "debug",
        _ => "trace",
    };
    env::set_var("RUST_LOG", log_level);
    env_logger::init();

    log::info!("noxy Physics Simulator v{}", env!("CARGO_PKG_VERSION"));
    log::info!("Backend: {:?}, Rigidbodies: {}, Steps: {}", cli.backend, cli.rigidbodies, cli.steps);
    log::debug!("Timestep: {}s, Gravity: {}m/s²", cli.timestep, cli.gravity);
    
    if let Some(ref config_path) = cli.config {
        log::info!("Loading configuration from: {}", config_path);
        // TODO: Load configuration file
    }
    
    if let Some(seed) = cli.seed {
        log::info!("Using seed for reproducible simulation: {}", seed);
    }
    
    if let Some(ref output) = cli.output {
        log::info!("Output file: {}", output);
    }

    if cli.profile {
        log::info!("Performance profiling enabled");
    }

    let mode = cli.get_mode();
    match mode {
        Mode::Interactive => {
            log::info!("Running in interactive mode");
            run_interactive_mode_with_config(cli.rigidbodies, cli.backend, cli.output, cli.timestep, cli.gravity)
        }
        Mode::Headless => {
            log::info!("Running in headless mode");
            run_headless_mode_with_config(cli.rigidbodies, cli.steps, cli.backend, cli.output, cli.timestep, cli.gravity)
        }
        Mode::Auto => {
            // Auto mode: use interactive if possible, fallback to headless
            if std::env::var("DISPLAY").is_ok() || cfg!(target_os = "windows") || cfg!(target_os = "macos") {
                log::info!("Auto mode: Running in interactive mode");
                run_interactive_mode_with_config(cli.rigidbodies, cli.backend, cli.output, cli.timestep, cli.gravity)
            } else {
                log::info!("Auto mode: No display detected, running in headless mode");
                run_headless_mode_with_config(cli.rigidbodies, cli.steps, cli.backend, cli.output, cli.timestep, cli.gravity)
            }
        }
    }
}

#[allow(dead_code)]
fn run_interactive_mode() -> anyhow::Result<()> {
    let event_loop = EventLoop::new()?;
    let mut app = Application::new();

    log::info!("Starting interactive mode with 3D visualization");
    Ok(event_loop.run_app(&mut app)?)
}

fn run_interactive_mode_with_config(
    rigidbody_count: u32, 
    backend: BackendType, 
    output: Option<String>,
    timestep: f32,
    gravity: f32
) -> anyhow::Result<()> {
    let event_loop = EventLoop::new()?;
    let mut app = Application::new();

    log::info!("Starting interactive mode with 3D visualization");
    log::info!("Configuration: {} rigidbodies, backend: {:?}", rigidbody_count, backend);
    log::info!("Physics settings: timestep: {}s, gravity: {}m/s²", timestep, gravity);
    
    if let Some(output_file) = output {
        log::info!("Output file configured: {}", output_file);
    }
    
    // TODO: Pass configuration to application
    Ok(event_loop.run_app(&mut app)?)
}

fn run_headless_mode_with_config(
    rigidbody_count: u32, 
    steps: u32, 
    backend: BackendType, 
    output_file: Option<String>,
    timestep: f32,
    gravity: f32
) -> anyhow::Result<()> {
    use crate::physics::backends::traits::BackendSelection;
    use crate::physics::simulation::RigidBodyProperties;
    use crate::physics::simulation::SimulationConfig;
    use app::modes::{ExecutionMode, ModeConfig, ModeManager, OutputFormat};

    log::info!("Starting headless mode for physics computation");
    log::info!("Configuration: {} rigidbodies, {} steps, backend: {:?}", 
               rigidbody_count, steps, backend);
    log::info!("Physics settings: timestep: {}s, gravity: {}m/s²", timestep, gravity);

    // Convert BackendType to BackendSelection
    let backend_selection: BackendSelection = backend.into();

    // Create headless mode configuration
    let output_path = output_file
        .map(|f| std::path::PathBuf::from(f))
        .unwrap_or_else(|| std::path::PathBuf::from("./simulation_output"));
    
    let mode = ExecutionMode::Headless {
        steps: steps,
        output_path: Some(output_path),
        output_format: OutputFormat::Json,
    };

    let config = ModeConfig {
        execution_mode: mode.clone(),
        simulation_config: SimulationConfig {
            backend_selection,
            max_rigidbodies: rigidbody_count as usize,
            gravity: glam::Vec3::new(0.0, -gravity, 0.0),
        },
    };

    // Initialize mode manager
    let mut mode_manager = ModeManager::new(mode);
    mode_manager.initialize(config)?;

    // Add configured number of test rigid bodies
    if let Some(simulation) = mode_manager.simulation_mut() {
        log::info!("Creating {} rigidbodies...", rigidbody_count);
        
        // Create a grid of falling objects
        let grid_size = (rigidbody_count as f32).sqrt().ceil() as u32;
        
        for i in 0..rigidbody_count {
            let x = (i % grid_size) as f32 * 2.0 - grid_size as f32;
            let z = (i / grid_size) as f32 * 2.0 - grid_size as f32;
            
            let properties = RigidBodyProperties {
                position: glam::Vec3::new(x, 10.0 + (i / 10) as f32, z),
                velocity: glam::Vec3::ZERO,
                mass: 1.0 + (i as f32 % 10.0) * 0.1,
            };
            simulation.add_rigidbody(properties)?;
        }

        log::info!("Added {} rigid bodies to simulation", simulation.rigidbody_count());
    }

    // Run the simulation
    log::info!("Starting simulation...");
    let start_time = std::time::Instant::now();
    let result = mode_manager.run()?;
    let elapsed = start_time.elapsed();

    if let app::modes::ModeResult::Headless {
        steps_completed,
        rigidbodies_simulated,
        backend_used,
        data_points,
    } = result
    {
        log::info!("Headless simulation completed successfully:");
        log::info!("  Steps: {}", steps_completed);
        log::info!("  Rigid bodies: {}", rigidbodies_simulated);
        log::info!("  Backend: {}", backend_used);
        log::info!("  Data points: {}", data_points);
        log::info!("  Duration: {:.2}s", elapsed.as_secs_f64());
        log::info!("  Performance: {:.1} steps/sec", steps_completed as f64 / elapsed.as_secs_f64());
    }

    Ok(())
}

#[allow(dead_code)]
fn run_headless_mode() -> anyhow::Result<()> {
    use crate::physics::backends::traits::BackendSelection;
    use crate::physics::simulation::RigidBodyProperties;
    use crate::physics::simulation::SimulationConfig;
    use app::modes::{ExecutionMode, ModeConfig, ModeManager, OutputFormat};

    log::info!("Starting headless mode for physics computation");

    // Create headless mode configuration
    let mode = ExecutionMode::Headless {
        steps: 1000,
        output_path: Some(std::path::PathBuf::from("./simulation_output")),
        output_format: OutputFormat::Json,
    };

    let config = ModeConfig {
        execution_mode: mode.clone(),
        simulation_config: SimulationConfig {
            backend_selection: BackendSelection::Auto,
            max_rigidbodies: 100,
            gravity: glam::Vec3::new(0.0, -9.81, 0.0),
        },
    };

    // Initialize mode manager
    let mut mode_manager = ModeManager::new(mode);
    mode_manager.initialize(config)?;

    // Add some test rigid bodies
    if let Some(simulation) = mode_manager.simulation_mut() {
        // Create a simple falling objects scenario
        for i in 0..10 {
            let properties = RigidBodyProperties {
                position: glam::Vec3::new(i as f32 * 0.5, 10.0 + i as f32, 0.0),
                velocity: glam::Vec3::ZERO,
                mass: 1.0 + i as f32 * 0.1,
            };
            simulation.add_rigidbody(properties)?;
        }

        log::info!(
            "Added {} rigid bodies to simulation",
            simulation.rigidbody_count()
        );
    }

    // Run the simulation
    let result = mode_manager.run()?;

    if let app::modes::ModeResult::Headless {
        steps_completed,
        rigidbodies_simulated,
        backend_used,
        data_points,
    } = result
    {
        log::info!("Headless simulation completed successfully:");
        log::info!("  Steps: {}", steps_completed);
        log::info!("  Rigid bodies: {}", rigidbodies_simulated);
        log::info!("  Backend: {}", backend_used);
        log::info!("  Data points: {}", data_points);
    }

    Ok(())
}
