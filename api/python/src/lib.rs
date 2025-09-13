//! # Noxy Python Bindings
//! 
//! Python bindings for the noxy physics engine.

use pyo3::prelude::*;

/// A Python module implemented in Rust.
#[pymodule]
fn noxy_python(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add("__version__", "0.0.0")?;
    Ok(())
}