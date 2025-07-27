use crate::cartridge::Cartridge;

pub struct Bus {
    cpu_ram: [u8; 0x0800],
//    ppu: Ppu,
//    apu: Apu,
//    cartridge: Cartridge,
}

impl Bus {
   pub fn read(&mut self, address: u16) -> u8 {

       match address {
           // Internal RAM
           0x0000..=0x1FFF => {
               self.cpu_ram[address as usize & 0x07FF]
           },
           // PPU Registers
           0x2000..=0x3FFF => {
               0
           },
           // APU and I/O registers
           0x4000..=0x4017 => {
               0
           },
           0x4018..=0x401F => {
               0
           },
           0x4020..=0xFFFF => {
               0
           },

           _ => {0}
       }
   }
    pub fn write(&mut self, address: u16, value: u8) {

        match address {
            // Internal RAM
            0x0000..=0x1FFF => {
                self.cpu_ram[address as usize & 0x07FF] = value
            },
            // PPU Registers
            0x2000..=0x3FFF => {

            },
            // APU and I/O registers
            0x4000..=0x4017 => {

            },
            0x4018..=0x401F => {

            },
            0x4020..=0xFFFF => {

            },

        }
    }

}
impl Bus {
    pub fn new() -> Bus {
        Bus {
            cpu_ram: [0; 0x0800],
        }
    }
}