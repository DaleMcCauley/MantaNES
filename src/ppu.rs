use sdl2::keyboard::Scancode::P;
use crate::bus::Bus;
use crate::bus::*;
use crate::flags::FlagRegister;
use crate::flags::ppu::{VBLANK_FLAG, V_BLANK_NMI_ENABLE};

pub(crate) struct Ppu {
    // Registers
    pub(crate) ctrl: u8,
    pub(crate) mask: u8,
    pub(crate) status: u8,
    oamaddr: u8,
    oamdata: u8,
    scroll_low: u8,
    scroll_high: u8,
    data: u8,
    oamdma: u8,

    // Memory
    pub(crate) chr_rom: Vec<u8>,
    name_table: [[u8; 1024]; 2],
    pallete_table: [u8; 32],

    pub(crate) address_latch: u8,
    pub(crate) ppu_data_buffer: u8,
    pub(crate) ppu_address: u16,

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

            chr_rom: Vec::new(),
            name_table: [[0; 1024]; 2],
            pallete_table: [0; 32],

            address_latch: 0x00,
            ppu_data_buffer: 0x00,

            nmi: false,
            scan_line: 0,
            cycle: 0,

        }
    }



        pub(crate) fn ppu_read(&mut self, address: u16) -> u8 {
            let masked_address = address & 0x3FFF;

            match masked_address {
                0x0000..=0x1FFF => { // cartridge chr-rom
                    self.chr_rom[masked_address as usize]
                },
                0x2000..=0x2FFF => { // name-table array

                },
                0x3000..=0x3EFF => { // mirror of name table

                },
                0x3F00..=0x3FFF => {   // palette table with special mirroring

                },
                _ => 0
            }


    }
    pub(crate) fn ppu_write(&mut self, address: u16, value: u8) {
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