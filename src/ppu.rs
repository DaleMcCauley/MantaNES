use sdl2::keyboard::Scancode::P;
use crate::bus::Bus;
use crate::bus::*;
use crate::flags::FlagRegister;
use crate::flags::ppu::{VBLANK_FLAG, V_BLANK_NMI_ENABLE};

pub(crate) struct Ppu {
    // Registers
    pub(crate) ctrl: u8,
    mask: u8,
    pub(crate) status: u8,
    oamaddr: u8,
    oamdata: u8,
    scroll_low: u8,
    scroll_high: u8,
    data: u8,
    oamdma: u8,

    // Memory
    name_table: [[u8; 1024]; 2],
    pallete_table: [u8; 32],

    address_latch: u8,
    ppu_data_buffer: u8,
    ppu_address: u16,

    pub(crate) nmi: bool,
    pub(crate) scan_line: u16,
    pub(crate) cycle: usize,
}

impl Ppu {
    pub(crate) fn new() -> Self {
        Ppu {
            ctrl: 0,
            mask: 0,
            status: 0b11100000,
            oamaddr: 0,
            oamdata: 0,
            scroll_low: 0,
            scroll_high: 0,
            ppu_address: 0x0000,
            data: 0,
            oamdma: 0,

            name_table: [[0; 1024]; 2],
            pallete_table: [0; 32],

            address_latch: 0x00,
            ppu_data_buffer: 0x00,

            nmi: false,
            scan_line: 0,
            cycle: 0,

        }
    }
    pub(crate) fn cpu_read(&mut self, address: u16) -> u8 {
        match address {
            0x0000 => 0,
            0x0001 => 0,
            0x0002 => {
                self.status.set_flag(VBLANK_FLAG, true);
                let value = (self.status & 0xE0) | (self.ppu_data_buffer & 0x1F);
                self.status.set_flag(VBLANK_FLAG, false);
                self.address_latch = 0;
                value
                    },
            0x0003 => 0,
            0x0004 => 0,
            0x0005 => 0,
            0x0006 => 0,
            0x0007 => {
                let mut value = self.ppu_data_buffer;
                self.ppu_data_buffer = self.ppu_read(address);

                if (self.ppu_address > 0x3f00) {
                    value = self.ppu_data_buffer;
                }
                self.ppu_address += 1;
                value
            },
            _ => 0,
        }
    }

    fn cpu_write(&mut self, mut bus: Bus, address: u16, value: u8) {
        match address {
            0x0000 => {
                self.ctrl = value;
            },
            0x0001 => {
                self.mask = value;
            },
            0x0002 => {
            },
            0x0003 => {},
            0x0004 => {},
            0x0005 => {},
            0x0006 => { // PPU Address
                if self.address_latch == 0 {
                    self.ppu_address = (self.ppu_address & 0x00FF) | ((value as u16)  << 8);
                    self.address_latch = 1;
                } else {
                    self.ppu_address = (self.ppu_address & 0xFF00) | (value as u16);
                    self.address_latch = 0;
                }

            },
            0x0007 => {
                self.ppu_write(self.ppu_address, value);
                self.ppu_address += 1;
            },
            _ => {},
        }
    }


    fn ppu_read(&mut self, address: u16) -> u8 {
        let masked_address = address & 0x3FFF;

        0
    }

    fn ppu_write(&mut self, address: u16, value: u8) {
        let masked_address = address & 0x3FFF;
    }

    pub(crate) fn step(&mut self) {

        if (self.scan_line == 1) && (self.cycle == 1) {
            self.status.set_flag(VBLANK_FLAG, true);
            if self.ctrl.check_flag(V_BLANK_NMI_ENABLE) {
                self.nmi = true;
            }
        }
    }
}