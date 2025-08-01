use crate::cartridge::Cartridge;
use crate::cpu::Cpu6502;
use crate::flags::FlagRegister;
use crate::flags::ppu::{VBLANK_FLAG, V_BLANK_NMI_ENABLE};
use crate::ppu::Ppu;

pub(crate) trait Memory {
    fn read(&mut self, address: u16) -> u8;
    fn write(&mut self, address: u16, value: u8);
    fn cpu_read_ppu(&mut self, ppu: Ppu, address: u16) -> u8;
    fn cpu_write_ppu(&mut self, mut ppu: Ppu, address: u16, value: u8);
    fn load_cartridge(&mut self, cartridge: Cartridge);
}
pub struct Bus {
    cpu_ram: [u8; 0x0800],
    cartridge: Option<Cartridge>,

}

impl Bus {
    pub fn new() -> Bus {
        Bus {
            cpu_ram: [0; 0x0800],
            cartridge: None,
        }
    }


        fn read_cartridge(&mut self, address: u16) -> u8 {
            if let Some(ref cartridge) = self.cartridge {
                match address {
                    0x8000..=0xFFFF => {
                        let rom_addr = address - 0x8000;

                        if cartridge.prg_rom.len() == 16384 {
                            cartridge.prg_rom[(rom_addr & 0x3FFF) as usize]
                        } else {
                            cartridge.prg_rom[rom_addr as usize % cartridge.prg_rom.len()]
                        }
                    },
                    _ => 0
                }
            } else {
                0
            }
        }


    }

impl Memory for Bus {
    fn read(&mut self, address: u16) -> u8 {
        match address {
            // Internal RAM
            0x0000..=0x1FFF => {
                self.cpu_ram[address as usize & 0x07FF]
            },
            // PPU Registers
            0x2000..=0x3FFF => {
                println!("Reading ppu register");
                self.cpu_read_ppu(address)
            },

            // APU and I/O registers
            0x4000..=0x4017 => {
                0
            },
            0x4018..=0x401F => {
                0
            },
            0x4020..=0xFFFF => {
                if let Some(ref cartridge) = self.cartridge {
                    self.read_cartridge(address)
                } else {
                    0
                }
            },

            _ => { 0 }
        }
    }
    fn write(&mut self, mut ppu: Ppu address: u16, value: u8) {
        match address {
            // Internal RAM
            0x0000..=0x1FFF => {
                self.cpu_ram[address as usize & 0x07FF] = value
            },
            // PPU Registers
            0x2000..=0x3FFF => {
                self.cpu_write_ppu(ppu, 0, 0)
            },

            // APU and I/O registers
            0x4000..=0x4017 => {},
            0x4018..=0x401F => {},
            // Cartridge space
            0x4020..=0xFFFF => {},
            _ => ()
        }
    }
    fn cpu_read_ppu(&mut self, mut ppu: Ppu, address: u16) -> u8 {
        match address {
            0x0000 => 0,
            0x0001 => 0,
            0x0002 => {
                ppu.status.set_flag(VBLANK_FLAG, true);
                let value = (ppu.status & 0xE0) | (ppu.ppu_data_buffer & 0x1F);
                ppu.status.set_flag(VBLANK_FLAG, false);
                ppu.address_latch = 0;
                value
            },
            0x0003 => 0,
            0x0004 => 0,
            0x0005 => 0,
            0x0006 => 0,
            0x0007 => {
                let mut value = ppu.ppu_data_buffer;
                ppu.ppu_data_buffer = self.ppu_read(address);

                if (ppu.ppu_address > 0x3f00) {
                    value = ppu.ppu_data_buffer;
                }
                ppu.ppu_address += 1;
                value
            },
            _ => 0,
        }
    }

    fn  cpu_write_ppu(&mut self, mut ppu: Ppu, address: u16, value: u8) {
        match address {
            0x0000 => {
                ppu.ctrl = value;
            },
            0x0001 => {
                ppu.mask = value;
            },
            0x0002 => {
            },
            0x0003 => {},
            0x0004 => {},
            0x0005 => {},
            0x0006 => { // PPU Address
                if ppu.address_latch == 0 {
                    ppu.ppu_address = (ppu.ppu_address & 0x00FF) | ((value as u16)  << 8);
                    ppu.address_latch = 1;
                } else {
                    ppu.ppu_address = (ppu.ppu_address & 0xFF00) | (value as u16);
                    ppu.address_latch = 0;
                }

            },
            0x0007 => {
                ppu.ppu_write(ppu.ppu_address, value);
                ppu.ppu_address += 1;
            },
            _ => {},
        }
    }
    fn load_cartridge(&mut self, cartridge: Cartridge) {
        self.cartridge = Some(cartridge);
    }

}
