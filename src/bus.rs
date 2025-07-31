use crate::cartridge::Cartridge;
use crate::cpu::Cpu6502;
use crate::ppu::Ppu;
pub struct Bus {
    cpu_ram: [u8; 0x0800],
    ppu: Ppu,
//    apu: Apu,
    cartridge: Option<Cartridge>,
}

impl Bus {
    pub fn new() -> Bus {
        Bus {
            cpu_ram: [0; 0x0800],
            cartridge: None,
            ppu: Ppu::new()
        }
    }
    pub fn load_cartridge(&mut self, cartridge: Cartridge) {
        self.cartridge = Some(cartridge);
    }
    pub fn read(&mut self, address: u16) -> u8 {
        match address {
            // Internal RAM
            0x0000..=0x1FFF => {
                self.cpu_ram[address as usize & 0x07FF]
            },
            // PPU Registers
            0x2000..=0x3FFF => {
                println!("Reading ppu register");
                self.ppu.cpu_read(address)
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
    pub fn write(&mut self, address: u16, value: u8) {
        match address {
            // Internal RAM
            0x0000..=0x1FFF => {
                self.cpu_ram[address as usize & 0x07FF] = value
            },
            // PPU Registers
            0x2000..=0x3FFF => {},

            // APU and I/O registers
            0x4000..=0x4017 => {},
            0x4018..=0x401F => {},
            // Cartridge space
            0x4020..=0xFFFF => {},
                _ => ()
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

        pub fn step(&mut self, cpu: &mut Cpu6502) {
            self.ppu.step();

            if

        }

    }

