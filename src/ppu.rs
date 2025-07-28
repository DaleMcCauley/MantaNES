use sdl2::keyboard::Scancode::P;
use crate::bus::Bus;
use crate::bus::*;
use crate::flags::FlagRegister;


pub(crate) struct Ppu {
    // Registers
    crtl: u8,
    mask: u8,
    status: u8,
    oamaddr: u8,
    oamdata: u8,
    scroll_low: u8,
    scroll_high: u8,
    addr_low: u8,
    addr_high: u8,
    data: u8,
    oamdma: u8,

    // Memory
    name_table: [[u8; 1024]; 2],
    pallete_table: [u8; 32],

}

impl Ppu {
    pub(crate) fn new() -> Self {
        Ppu {
            crtl: 0,
            mask: 0,
            status: 0b11100000,
            oamaddr: 0,
            oamdata: 0,
            scroll_low: 0,
            scroll_high: 0,
            addr_low: 0,
            addr_high: 0,
            data: 0,
            oamdma: 0,

            name_table: [[0; 1024]; 2],
            pallete_table: [0; 32]
        }
    }
    pub(crate) fn cpu_read(&mut self, address: u16) -> u8 {
        match address {
            0x0000 => 0,
            0x0001 => 0,
            0x0002 => self.status,
            0x0003 => 0,
            0x0004 => 0,
            0x0005 => 0,
            0x0006 => 0,
            0x0007 => 0,
            _ => 0,
        }
    }

    fn cpu_write(mut bus: Bus, address: u16, value: u8) {
        Bus::write(&mut bus, address, value);
    }

    fn ppu_read(address: u16) -> u8 {
        let masked_address = address & 0x3FFF;

        0
    }

    fn ppu_write(address: u16, value: u8) {
        let masked_address = address & 0x3FFF;
    }

}