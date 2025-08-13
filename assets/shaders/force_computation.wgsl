// Force computation shader for rigid body physics
// Computes gravitational and inter-particle forces

struct PhysicsUniforms {
    dt: f32,
    gravity: vec3<f32>,
    rigidbody_count: u32,
}

@group(0) @binding(0) var<storage, read> positions: array<vec3<f32>>;
@group(0) @binding(1) var<storage, read> velocities: array<vec3<f32>>;
@group(0) @binding(2) var<storage, read_write> forces: array<vec3<f32>>;
@group(0) @binding(3) var<storage, read> masses: array<f32>;
@group(0) @binding(4) var<uniform> uniforms: PhysicsUniforms;

@compute @workgroup_size(64)
fn compute_forces(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let index = global_id.x;
    
    // Bounds check
    if (index >= uniforms.rigidbody_count) {
        return;
    }
    
    let position = positions[index];
    let velocity = velocities[index];
    let mass = masses[index];
    
    // Initialize with gravitational force
    var total_force = uniforms.gravity * mass;
    
    // Add drag force (simple linear drag)
    let drag_coefficient = 0.01;
    total_force = total_force - velocity * drag_coefficient;
    
    // TODO: Add collision forces
    // TODO: Add spring forces for joints
    // TODO: Add constraint forces
    
    // Simple inter-particle repulsion (to prevent overlap)
    for (var other_index: u32 = 0u; other_index < uniforms.rigidbody_count; other_index = other_index + 1u) {
        if (other_index == index) {
            continue;
        }
        
        let other_position = positions[other_index];
        let distance_vec = position - other_position;
        let distance = length(distance_vec);
        
        // Avoid division by zero and apply repulsion at close distances
        let min_distance = 0.5; // Minimum separation distance
        if (distance < min_distance && distance > 0.001) {
            let repulsion_strength = 10.0;
            let repulsion_force = normalize(distance_vec) * repulsion_strength * (min_distance - distance) / distance;
            total_force = total_force + repulsion_force;
        }
    }
    
    // Write computed force
    forces[index] = total_force;
}