mod app;
mod physics;
mod utils;

use std::env;

use app::Application;
use winit::event_loop::EventLoop;

fn main() -> anyhow::Result<()> {
    env::set_var("RUST_LOG", "info");
    env_logger::init();

    // Parse command line arguments for mode selection
    let args: Vec<String> = std::env::args().collect();

    if args.len() > 1 && args[1] == "--headless" {
        // Run in headless mode for testing
        run_headless_mode()
    } else {
        // Run in interactive mode (existing behavior)
        run_interactive_mode()
    }
}

fn run_interactive_mode() -> anyhow::Result<()> {
    let event_loop = EventLoop::new()?;
    let mut app = Application::new();

    log::info!("Starting interactive mode with 3D visualization");
    Ok(event_loop.run_app(&mut app)?)
}

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
