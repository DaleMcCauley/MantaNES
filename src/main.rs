extern crate sdl2;

mod cpu;
mod bus;
mod cartridge;
mod ppu;
mod flags;
mod system;

use std::env::args;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use std::time::Duration;
use crate::bus::Bus;

pub fn main() {
    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();

    let window = video_subsystem.window("MantaNES", 800, 600)
        .position_centered()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().build().unwrap();

    canvas.set_draw_color(Color::RGB(0, 0, 0));
    canvas.clear();
    canvas.present();

    let mut event_pump = sdl_context.event_pump().unwrap();




    let mut cpu = cpu::Cpu6502::init_cpu();
    let args: Vec<String> = args().collect();
    let rom_path = args[1].to_string();
    let mut bus = Bus::new();
    let cartridge = cartridge::Cartridge::load_cartridge(rom_path);
    cpu.load_rom(&mut bus, cartridge);
    println!("Rom loaded into memory");
    cpu.reset(&mut bus);

    'running: loop {

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} | Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    break 'running
                },
                _ => {}
            }
        }
        
        canvas.present();
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
    }

}
