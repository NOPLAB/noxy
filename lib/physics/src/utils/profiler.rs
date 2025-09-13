//! # Performance Profiling
//! 
//! Tools for measuring and analyzing physics simulation performance.

use std::time::{Duration, Instant};
use std::collections::HashMap;

/// Performance profiler for physics simulation
#[derive(Debug, Default)]
pub struct Profiler {
    timers: HashMap<String, Timer>,
    frame_times: Vec<Duration>,
    max_frame_history: usize,
}

/// Individual timer for measuring performance
#[derive(Debug)]
struct Timer {
    start_time: Option<Instant>,
    total_time: Duration,
    call_count: u64,
}

impl Default for Timer {
    fn default() -> Self {
        Self {
            start_time: None,
            total_time: Duration::ZERO,
            call_count: 0,
        }
    }
}

impl Profiler {
    /// Create a new profiler
    pub fn new() -> Self {
        Self {
            timers: HashMap::new(),
            frame_times: Vec::new(),
            max_frame_history: 60, // Keep last 60 frames
        }
    }
    
    /// Start timing a section
    pub fn start_timer(&mut self, name: &str) {
        let timer = self.timers.entry(name.to_string()).or_default();
        timer.start_time = Some(Instant::now());
    }
    
    /// End timing a section
    pub fn end_timer(&mut self, name: &str) {
        if let Some(timer) = self.timers.get_mut(name) {
            if let Some(start_time) = timer.start_time.take() {
                timer.total_time += start_time.elapsed();
                timer.call_count += 1;
            }
        }
    }
    
    /// Record a frame time
    pub fn record_frame_time(&mut self, duration: Duration) {
        self.frame_times.push(duration);
        if self.frame_times.len() > self.max_frame_history {
            self.frame_times.remove(0);
        }
    }
    
    /// Get average time for a timer
    pub fn average_time(&self, name: &str) -> Option<Duration> {
        self.timers.get(name).map(|timer| {
            if timer.call_count > 0 {
                timer.total_time / timer.call_count as u32
            } else {
                Duration::ZERO
            }
        })
    }
    
    /// Get total time for a timer
    pub fn total_time(&self, name: &str) -> Option<Duration> {
        self.timers.get(name).map(|timer| timer.total_time)
    }
    
    /// Get call count for a timer
    pub fn call_count(&self, name: &str) -> Option<u64> {
        self.timers.get(name).map(|timer| timer.call_count)
    }
    
    /// Get average frame time
    pub fn average_frame_time(&self) -> Option<Duration> {
        if self.frame_times.is_empty() {
            None
        } else {
            let total: Duration = self.frame_times.iter().sum();
            Some(total / self.frame_times.len() as u32)
        }
    }
    
    /// Get frame rate (FPS)
    pub fn frame_rate(&self) -> Option<f32> {
        self.average_frame_time().map(|avg| {
            1.0 / avg.as_secs_f32()
        })
    }
    
    /// Reset all timers
    pub fn reset(&mut self) {
        self.timers.clear();
        self.frame_times.clear();
    }
    
    /// Generate a performance report
    pub fn report(&self) -> String {
        let mut report = String::new();
        report.push_str("=== Performance Report ===\n");
        
        if let Some(avg_frame) = self.average_frame_time() {
            if let Some(fps) = self.frame_rate() {
                report.push_str(&format!("Frame Rate: {:.1} FPS\n", fps));
                report.push_str(&format!("Frame Time: {:.3} ms\n", avg_frame.as_secs_f64() * 1000.0));
            }
        }
        
        report.push_str("\nTimers:\n");
        for (name, timer) in &self.timers {
            let avg = if timer.call_count > 0 {
                timer.total_time / timer.call_count as u32
            } else {
                Duration::ZERO
            };
            
            report.push_str(&format!(
                "  {}: {:.3} ms avg, {} calls, {:.3} ms total\n",
                name,
                avg.as_secs_f64() * 1000.0,
                timer.call_count,
                timer.total_time.as_secs_f64() * 1000.0
            ));
        }
        
        report
    }
}

/// RAII timer that automatically ends when dropped
pub struct ScopedTimer<'a> {
    profiler: &'a mut Profiler,
    name: String,
}

impl<'a> ScopedTimer<'a> {
    /// Create a new scoped timer
    pub fn new(profiler: &'a mut Profiler, name: &str) -> Self {
        profiler.start_timer(name);
        Self {
            profiler,
            name: name.to_string(),
        }
    }
}

impl<'a> Drop for ScopedTimer<'a> {
    fn drop(&mut self) {
        self.profiler.end_timer(&self.name);
    }
}

/// Macro for creating scoped timers
#[macro_export]
macro_rules! profile {
    ($profiler:expr, $name:expr, $code:block) => {
        {
            let _timer = ScopedTimer::new($profiler, $name);
            $code
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_profiler_basic() {
        let mut profiler = Profiler::new();
        
        profiler.start_timer("test");
        thread::sleep(Duration::from_millis(10));
        profiler.end_timer("test");
        
        let avg_time = profiler.average_time("test").unwrap();
        assert!(avg_time >= Duration::from_millis(10));
        assert_eq!(profiler.call_count("test").unwrap(), 1);
    }

    #[test]
    fn test_scoped_timer() {
        let mut profiler = Profiler::new();
        
        {
            let _timer = ScopedTimer::new(&mut profiler, "scoped");
            thread::sleep(Duration::from_millis(5));
        }
        
        assert_eq!(profiler.call_count("scoped").unwrap(), 1);
    }

    #[test]
    fn test_frame_rate() {
        let mut profiler = Profiler::new();
        
        // Simulate 60 FPS
        for _ in 0..10 {
            profiler.record_frame_time(Duration::from_millis(16));
        }
        
        let fps = profiler.frame_rate().unwrap();
        assert!((fps - 62.5).abs() < 1.0); // 1000/16 ≈ 62.5
    }
}