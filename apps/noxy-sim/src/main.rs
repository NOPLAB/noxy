//! # Noxy Physics Simulator
//! 
//! Main application for running physics simulations.

use noxy_api_rust::prelude::*;
use clap::Parser;
use anyhow::Result;

#[derive(Parser)]
#[command(name = "noxy-sim")]
#[command(about = "High-performance GPU physics simulator")]
struct Args {
    /// Number of rigid bodies to simulate
    #[arg(long, default_value = "10")]
    rigidbodies: usize,
    
    /// Number of simulation steps
    #[arg(long, default_value = "1000")]
    steps: u32,
    
    /// Physics solver to use
    #[arg(long, default_value = "pbd")]
    solver: String,
    
    /// Backend to use (cpu, gpu, auto)
    #[arg(long, default_value = "auto")]
    backend: String,
    
    /// Run in headless mode
    #[arg(long)]
    headless: bool,
}

fn main() -> Result<()> {
    env_logger::init();
    let args = Args::parse();
    
    println!("=== Noxy Physics Simulator ===");
    println!("Bodies: {}, Steps: {}, Solver: {}, Backend: {}", 
             args.rigidbodies, args.steps, args.solver, args.backend);
    
    // Parse solver type
    let solver_type = match args.solver.as_str() {
        "implicit" => SolverType::Implicit,
        "pbd" => SolverType::PBD,
        "xpbd" => SolverType::XPBD,
        _ => {
            eprintln!("Unknown solver: {}. Using PBD.", args.solver);
            SolverType::PBD
        }
    };
    
    // Parse backend type
    let backend_type = match args.backend.as_str() {
        "cpu" => BackendType::CPU,
        "gpu" => BackendType::GPU,
        "auto" => BackendType::Auto,
        _ => {
            eprintln!("Unknown backend: {}. Using auto.", args.backend);
            BackendType::Auto
        }
    };
    
    // Create simulation
    let mut sim = SimulationBuilder::new()
        .solver(solver_type)
        .backend(backend_type)
        .max_rigid_bodies(args.rigidbodies)
        .build()?;
    
    println!("Solver: {}", sim.solver_info().description);
    
    // Add rigid bodies
    for i in 0..args.rigidbodies {
        let x = (i % 10) as f32 * 2.0 - 10.0;
        let y = 5.0 + (i / 10) as f32 * 2.0;
        let z = 0.0;
        
        sim.add_rigidbody(
            Shape::Sphere { radius: 0.5 },
            Transform::from_translation(Vec3::new(x, y, z)),
            Material::default(),
        )?;
    }
    
    println!("Added {} rigid bodies", args.rigidbodies);
    
    // Run simulation
    let dt = 1.0 / 60.0;
    let start_time = std::time::Instant::now();
    
    for step in 0..args.steps {
        if step % (args.steps / 10).max(1) == 0 {
            println!("Step {}/{}", step, args.steps);
        }
        
        sim.step(dt)?;
    }
    
    let elapsed = start_time.elapsed();
    let stats = sim.statistics();
    
    println!("\n=== Results ===");
    println!("Simulation completed in {:.2}s", elapsed.as_secs_f32());
    println!("Average step time: {:.3}ms", stats.avg_step_time * 1000.0);
    println!("Total rigid bodies: {}", stats.rigid_body_count);
    println!("Convergence failures: {}", stats.convergence_failures);
    
    Ok(())
}