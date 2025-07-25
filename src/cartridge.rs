use crate::cpu::Cpu6502;

pub struct Cartridge {
    pub prg_rom: Vec<u8>,
    pub chr_rom: Vec<u8>,
    pub metadata: RomMetadata
}

pub struct RomMetadata {
    pub prg_rom_size_bytes : usize,
    pub chr_rom_size_bytes: usize,
    pub mirroring_vert: u8,
    pub mapper_number: u8,


}
impl Cartridge {

    pub fn load_cartridge(path: String) -> Cartridge {
        let rom_data = std::fs::read(path).unwrap();

        assert_eq!(&rom_data[0..4], b"NES\x1A");

        let metadata = RomMetadata {
            prg_rom_size_bytes: rom_data[4] as usize * 16384,
            chr_rom_size_bytes: rom_data[5] as usize * 8192,
            mirroring_vert: rom_data[6] & 0x01,
            mapper_number: ((rom_data[7] & 0xF0) <<4) | ((rom_data[6] & 0xF0) >> 4) ,
        };
        Cartridge {
            prg_rom: rom_data[16..16 + metadata.prg_rom_size_bytes as usize].to_vec(),
            chr_rom: rom_data[16 + metadata.prg_rom_size_bytes as usize.. 16 + metadata.prg_rom_size_bytes + metadata.chr_rom_size_bytes].to_vec(),
            metadata
        }



        }
}