// Basic collision detection and response shader

struct PhysicsUniforms {
    dt: f32,
    gravity: vec3<f32>,
    rigidbody_count: u32,
}

@group(0) @binding(0) var<storage, read> positions: array<vec3<f32>>;
@group(0) @binding(1) var<storage, read_write> velocities: array<vec3<f32>>;
@group(0) @binding(2) var<storage, read_write> forces: array<vec3<f32>>;
@group(0) @binding(3) var<storage, read> masses: array<f32>;
@group(0) @binding(4) var<uniform> uniforms: PhysicsUniforms;

@compute @workgroup_size(64)
fn detect_collisions(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let index = global_id.x;
    
    // Bounds check
    if (index >= uniforms.rigidbody_count) {
        return;
    }
    
    let position = positions[index];
    let velocity = velocities[index];
    let mass = masses[index];
    
    // Initialize collision force
    var collision_force = vec3<f32>(0.0);
    
    // Check collisions with other particles
    for (var other_index: u32 = index + 1u; other_index < uniforms.rigidbody_count; other_index = other_index + 1u) {
        let other_position = positions[other_index];
        let other_velocity = velocities[other_index];
        let other_mass = masses[other_index];
        
        let distance_vec = position - other_position;
        let distance = length(distance_vec);
        
        // Collision radius (assuming all particles have radius 0.5)
        let collision_radius = 1.0;
        
        if (distance < collision_radius && distance > 0.001) {
            // Collision detected
            let normal = normalize(distance_vec);
            let overlap = collision_radius - distance;
            
            // Relative velocity
            let relative_velocity = velocity - other_velocity;
            let velocity_along_normal = dot(relative_velocity, normal);
            
            // Do not resolve if velocities are separating
            if (velocity_along_normal > 0.0) {
                continue;
            }
            
            // Restitution coefficient (bounciness)
            let restitution = 0.7;
            
            // Calculate impulse magnitude
            let impulse_magnitude = -(1.0 + restitution) * velocity_along_normal;
            let total_mass = mass + other_mass;
            impulse_magnitude = impulse_magnitude / (1.0/mass + 1.0/other_mass);
            
            // Apply impulse
            let impulse = impulse_magnitude * normal;
            
            // Convert impulse to force (F = impulse / dt)
            let contact_force = impulse / uniforms.dt;
            
            // Add repulsion force to prevent overlap
            let repulsion_strength = 100.0;
            let repulsion_force = normal * repulsion_strength * overlap;
            
            collision_force = collision_force + contact_force + repulsion_force;
        }
    }
    
    // Update forces with collision forces
    forces[index] = forces[index] + collision_force;
}