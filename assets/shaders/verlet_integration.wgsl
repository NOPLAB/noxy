// Verlet integration shader for better energy conservation

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

@compute @workgroup_size(64)
fn verlet_integrate(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let index = global_id.x;
    
    // Bounds check
    if (index >= uniforms.rigidbody_count) {
        return;
    }
    
    let dt = uniforms.dt;
    
    // Read current state
    let position = positions[index];
    let velocity = velocities[index];
    let force = forces[index];
    let mass = masses[index];
    
    // Compute acceleration
    let acceleration = force / mass;
    
    // Velocity Verlet integration
    // x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt^2
    // v(t+dt) = v(t) + a(t)*dt
    
    let new_position = position + velocity * dt + 0.5 * acceleration * dt * dt;
    let new_velocity = velocity + acceleration * dt;
    
    // Apply boundary conditions
    var final_position = new_position;
    var final_velocity = new_velocity;
    
    // Ground collision at y = 0
    if (final_position.y < 0.0) {
        final_position.y = 0.0;
        // Bounce with energy loss
        final_velocity.y = -final_velocity.y * 0.7;
    }
    
    // Simple box boundaries (optional)
    let boundary_size = 50.0;
    let restitution = 0.8;
    
    // X boundaries
    if (final_position.x < -boundary_size) {
        final_position.x = -boundary_size;
        final_velocity.x = -final_velocity.x * restitution;
    } else if (final_position.x > boundary_size) {
        final_position.x = boundary_size;
        final_velocity.x = -final_velocity.x * restitution;
    }
    
    // Z boundaries  
    if (final_position.z < -boundary_size) {
        final_position.z = -boundary_size;
        final_velocity.z = -final_velocity.z * restitution;
    } else if (final_position.z > boundary_size) {
        final_position.z = boundary_size;
        final_velocity.z = -final_velocity.z * restitution;
    }
    
    // Write results
    positions[index] = final_position;
    velocities[index] = final_velocity;
}