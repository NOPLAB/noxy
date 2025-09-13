//! Simple GPU backend test

use noxy::physics::backends::{
    traits::PhysicsBackend, 
    gpu::GpuBackend,
    factory::BackendFactory,
    traits::BackendSelection
};

#[test]
fn test_gpu_backend_creation() {
    let backend = GpuBackend::new();
    assert_eq!(backend.name(), "gpu");
}

#[test]
fn test_gpu_backend_availability() {
    // Test that GPU backend reports availability correctly
    let is_available = GpuBackend::is_available();
    
    // For TDD version, this should be false
    assert!(!is_available, "GPU should not be available in TDD version");
    println!("GPU backend availability: {}", is_available);
}

#[test]
fn test_gpu_backend_factory_creation() {
    let backend = BackendFactory::create(BackendSelection::Gpu);
    assert_eq!(backend.name(), "gpu");
}

#[test]
fn test_gpu_backend_initialization() {
    let mut backend = GpuBackend::new();
    
    // Should fail since GPU is not available in TDD version
    let result = backend.initialize(100);
    assert!(result.is_err(), "GPU initialization should fail in TDD version");
    
    println!("GPU initialization result: {:?}", result);
}

#[test]
fn test_gpu_backend_auto_fallback() {
    // Test that auto selection falls back to CPU when GPU unavailable
    let backend = BackendFactory::create(BackendSelection::Auto);
    
    // Should fallback to CPU since GPU is unavailable
    assert_eq!(backend.name(), "cpu");
    println!("Auto selection chose: {}", backend.name());
}