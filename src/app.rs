pub mod modes;
mod renderer;

use std::{collections::HashMap, sync::Arc};

use winit::{
    application::ApplicationHandler,
    event::WindowEvent,
    event_loop::{self, ActiveEventLoop},
    window::{Window, WindowId},
};

pub struct Application {
    windows: HashMap<WindowId, Arc<Window>>,
    window_impl: HashMap<WindowId, Box<dyn ApplicationWindow>>,
}

impl Application {
    pub fn new() -> Self {
        Self {
            windows: HashMap::new(),
            window_impl: HashMap::new(),
        }
    }

    pub fn create_window(&mut self, event_loop: &ActiveEventLoop) -> anyhow::Result<WindowId> {
        let window_attrs = Window::default_attributes().with_title("window");
        let window = event_loop.create_window(window_attrs)?;
        let window_id = window.id();
        self.windows.insert(window_id, Arc::new(window));
        let window_impl = pollster::block_on(renderer::RendererWindow::new(
            self.windows[&window_id].clone(),
        ));

        self.window_impl.insert(window_id, Box::new(window_impl));

        Ok(window_id)
    }
}

impl ApplicationHandler for Application {
    fn resumed(&mut self, event_loop: &event_loop::ActiveEventLoop) {
        log::info!("Application resumed");

        self.create_window(event_loop).unwrap();
    }

    fn window_event(
        &mut self,
        event_loop: &event_loop::ActiveEventLoop,
        window_id: winit::window::WindowId,
        event: winit::event::WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }
            WindowEvent::RedrawRequested => {}
            _ => {}
        }

        if let Some(window_impl) = self.window_impl.get_mut(&window_id) {
            window_impl.window_event(event_loop, event);
        }
    }
}

pub trait ApplicationWindow {
    fn window_event(
        &mut self,
        event_loop: &event_loop::ActiveEventLoop,
        event: winit::event::WindowEvent,
    );
}
