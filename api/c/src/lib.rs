//! # Noxy C API
//! 
//! C API for the noxy physics engine.

#[no_mangle]
pub extern "C" fn noxy_version() -> *const std::os::raw::c_char {
    b"0.0.0\0".as_ptr() as *const std::os::raw::c_char
}