// Basic physics compute shader for rigid body simulation
// Combines force computation and Verlet integration in single pass

struct PhysicsUniforms {
    dt: f32,
    gravity: vec4<f32>,  // vec3 with padding for alignment
    rigidbody_count: u32,
    _padding0: u32,
    _padding1: u32,
    _padding2: u32,
}

@group(0) @binding(0) var<storage, read_write> positions: array<vec3<f32>>;
@group(0) @binding(1) var<storage, read_write> velocities: array<vec3<f32>>;
@group(0) @binding(2) var<storage, read> masses: array<f32>;
@group(0) @binding(3) var<storage, read_write> forces: array<vec3<f32>>;
@group(0) @binding(4) var<uniform> uniforms: PhysicsUniforms;

@compute @workgroup_size(64)
fn physics_step(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let index = global_id.x;
    
    // Bounds check
    if (index >= uniforms.rigidbody_count) {
        return;
    }
    
    let dt = uniforms.dt;
    let gravity = uniforms.gravity.xyz;  // Extract vec3 from vec4
    
    // Read current state
    let position = positions[index];
    let velocity = velocities[index];
    let mass = masses[index];
    
    // Reset forces and apply gravity
    var total_force = gravity * mass;
    
    // TODO: Add inter-particle forces (springs, collisions, etc.)
    // For now, just gravity
    
    // Verlet integration
    // v(t+dt) = v(t) + a*dt
    // x(t+dt) = x(t) + v(t+dt)*dt
    let acceleration = total_force / mass;
    let new_velocity = velocity + acceleration * dt;
    let new_position = position + new_velocity * dt;
    
    // Simple ground collision (y = 0)
    var final_position = new_position;
    var final_velocity = new_velocity;
    
    if (final_position.y < 0.0) {
        final_position.y = 0.0;
        final_velocity.y = -final_velocity.y * 0.8; // Bounce with damping
    }
    
    // Write back results
    positions[index] = final_position;
    velocities[index] = final_velocity;
    forces[index] = total_force;
}