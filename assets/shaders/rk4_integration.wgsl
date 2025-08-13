// Runge-Kutta 4th order integration shader - high accuracy

struct PhysicsUniforms {
    dt: f32,
    gravity: vec3<f32>,
    rigidbody_count: u32,
}

@group(0) @binding(0) var<storage, read_write> positions: array<vec3<f32>>;
@group(0) @binding(1) var<storage, read_write> velocities: array<vec3<f32>>;
@group(0) @binding(2) var<storage, read> forces: array<vec3<f32>>;
@group(0) @binding(3) var<storage, read> masses: array<f32>;
@group(0) @binding(4) var<uniform> uniforms: PhysicsUniforms;

// Helper function to compute acceleration from force
fn compute_acceleration(force: vec3<f32>, mass: f32) -> vec3<f32> {
    return force / mass;
}

@compute @workgroup_size(64)
fn rk4_integrate(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let index = global_id.x;
    
    // Bounds check
    if (index >= uniforms.rigidbody_count) {
        return;
    }
    
    let dt = uniforms.dt;
    
    // Read current state
    let x0 = positions[index];
    let v0 = velocities[index];
    let force = forces[index];
    let mass = masses[index];
    
    let a0 = compute_acceleration(force, mass);
    
    // RK4 integration
    // k1 = f(t, y)
    let k1_v = a0 * dt;
    let k1_x = v0 * dt;
    
    // k2 = f(t + dt/2, y + k1/2)
    let v1 = v0 + k1_v * 0.5;
    let k2_v = a0 * dt; // Simplified: assuming constant acceleration
    let k2_x = v1 * dt;
    
    // k3 = f(t + dt/2, y + k2/2)
    let v2 = v0 + k2_v * 0.5;
    let k3_v = a0 * dt;
    let k3_x = v2 * dt;
    
    // k4 = f(t + dt, y + k3)
    let v3 = v0 + k3_v;
    let k4_v = a0 * dt;
    let k4_x = v3 * dt;
    
    // Combine results
    let new_velocity = v0 + (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v) / 6.0;
    let new_position = x0 + (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x) / 6.0;
    
    // Apply boundary conditions
    var final_position = new_position;
    var final_velocity = new_velocity;
    
    // Ground collision at y = 0
    if (final_position.y < 0.0) {
        final_position.y = 0.0;
        final_velocity.y = -final_velocity.y * 0.8; // Bounce with damping
    }
    
    // Write results
    positions[index] = final_position;
    velocities[index] = final_velocity;
}