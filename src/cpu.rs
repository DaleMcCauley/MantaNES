pub struct Cpu6502 {
    pub a: u8,
    pub x: u8,
    pub y: u8,
    pub pc: u16,
    pub s: u8,
    pub p: u8,

    pub cycles: usize,

    memory: [u8; 0x10000],

}

impl Cpu6502 {

    pub fn init_cpu() -> Cpu6502 {
        let cpu: Cpu6502 = Cpu6502 {
            a: 0,
            x: 0,
            y: 0,
            pc: 0,
            s: 0,
            p: 0,

            cycles: 0,
            memory: [0; 0x10000],
        };
        cpu
    }

    pub fn step(&mut self) {
        let opcode = self.read_opcode(self.pc);

        match opcode {
            _ => println!("Opcode not interpreted: {}", opcode),

          
        }
    }
    pub fn read_opcode(&mut self, pc: u16) -> u8 {
        self.memory[pc as usize]

    }
}

