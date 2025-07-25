use crate::cartridge::Cartridge;

pub struct Cpu6502 {
    pub a: u8,
    pub x: u8,
    pub y: u8,
    pub pc: u16,
    pub sp: u8,
    pub p: u8,
    pub cycles: usize,
    pub stack: [u8; 256],
    pub memory: [u8; 0x10000],

}

// Flag definitions
const CARRY_FLAG:u8 = 0b00000001;
const ZERO_FLAG:u8 = 0b00000010;
const INTERRUPT_DISABLE_FLAG:u8 = 0b00000100;
const DECIMAL_FLAG:u8 = 0b00001000;
const OVERFLOW_FLAG:u8 = 0b01000000;
const NEGATIVE_FLAG:u8 = 0b10000000;
impl Cpu6502 {

    pub fn init_cpu() -> Cpu6502 {
        let cpu: Cpu6502 = Cpu6502 {
            a: 0,
            x: 0,
            y: 0,
            pc: 0,
            sp: 0,
            p: 0,
            stack: [0; 256],
            cycles: 0,
            memory: [0; 0x10000],
        };
        cpu
    }
    pub fn load_rom(&mut self, cartridge: Cartridge) {
        let game_code = cartridge.prg_rom;
        game_code.iter().enumerate().for_each(|(i, byte)| {self.memory[0x8000 + i] = *byte});
    }
    pub fn step(&mut self) {
        let opcode = self.memory[self.pc as usize];
// Main decoder hub
        let additional_cycles = match opcode {
            0xA9 => self.lda_immediate(),
            0xA5 => self.lda_zero_page(),
            0xB5 => self.lda_zero_page_x(),
            0xAD => self.lda_absolute(),
            0xBD => self.lda_absolute_x(),
            0xB9 => self.lda_absolute_y(),
            0xA1 => self.lda_indirect_x(),
            0xB1 => self.lda_indirect_y(),

            0x85 => self.sta_zero_page(),
            0x8D => self.sta_absolute(),
            0x9D => self.sta_absolute_x(),
            0x99 => self.sta_absolute_y(),
            0x81 => self.sta_indirect_x(),
            0x91 => self.sta_indirect_y(),

            0xA2 => self.ldx_immediate(),
            0xA6 => self.ldx_zero_page(),
            0xB6 => self.ldx_zero_page_y(),
            0xAE => self.ldx_absolute(),
            0xBE => self.ldx_absolute_y(),

            0xAA => self.tax_implied(),
            0xA8 => self.tay_implied(),
            0xBA => self.tsx_implied(),
            0x8A => self.txa_implied(),
            0x9A => self.txs_implied(),
            0x98 => self.tya_implied(),

            0x4C => self.jmp_absolute(),
            0x6C => self.jmp_indirect(),
            0x20 => self.jsr_absolute(),
            0x60 => self.rts_implied(),

            0xF0 => self.beq_relative(),
            0x90 => self.bcc_relative(),
            0xB0 => self.bcs_relative(),
            0x30 => self.bmi_relative(),
            0xD0 => self.bne_relative(),
            0x10 => self.bpl_relative(),
            0x50 => self.bvc_relative(),
            0x70 => self.bvs_relative(),





            _ => {println!("Opcode not interpreted: ;{}", opcode);
                0},
        };
        self.cycles += additional_cycles;
    }
    fn set_flag(&mut self, flag:u8, condition: bool) {
        if condition {
            self.p |= flag;
        } else {
            self.p &= !(flag);
        }
    }
    fn update_nz_flags(&mut self, value: u8) {
        self.set_flag(ZERO_FLAG, value == 0);
        self.set_flag(NEGATIVE_FLAG, (value & 0x80) != 0);
    }

    fn check_zero_flag(&self) -> bool {
        self.p & ZERO_FLAG != 0b00000000
    }

    fn check_negative_flag(&self) -> bool {
        self.p & NEGATIVE_FLAG != 0b00000000
    }

    fn check_carry_flag(&self) -> bool {
        self.p & CARRY_FLAG != 0b00000000
    }

    fn check_overflow_flag(&self) -> bool {
        self.p & OVERFLOW_FLAG != 0b00000000
    }
    fn stack_push(&mut self, input: u8) {
        self.stack[self.sp as usize] = input;
        self.sp = self.sp.wrapping_sub(1);
    }

    fn stack_pop(&mut self) -> u8 {
        self.sp = self.sp.wrapping_add(1);
        self.stack[self.sp as usize]
    }

    // LDA opcodes _________________________________________________________________________________
    fn lda_immediate(&mut self) -> usize {
        self.pc += 1;
        self.a = self.memory[self.pc as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);
        2
    }

    fn lda_zero_page(&mut self) -> usize {
        self.pc += 1;
        let address = self.memory[self.pc as usize] as u16;
        self.a = self.memory[address as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);
        3
    }

    fn lda_zero_page_x(&mut self) -> usize {
        self.pc += 1;
        let address1 = self.memory[self.pc as usize] as u16;
        let address2 = (address1 + self.x as u16) % 0x100;
        self.a = self.memory[address2 as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);
        4
    }

    fn lda_absolute(&mut self) -> usize {
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        let absolute_address = (high_byte << 8) | low_byte;
        self.a = self.memory[absolute_address as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);
        4
    }

    fn lda_absolute_x(&mut self) -> usize {
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        let base_address = (high_byte << 8) | low_byte;
        let absolute_address = base_address + self.x as u16;
        self.a = self.memory[absolute_address as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);

        // CHecks if a page boundary was crossed to determine number of cycles
        if  (base_address & 0xFF00) != (absolute_address & 0xFF00) {
            5
        } else {
            4
        }
    }

    fn lda_absolute_y(&mut self) -> usize {
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        let base_address = (high_byte << 8) | low_byte;
        let absolute_address = base_address + self.y as u16;
        self.a = self.memory[absolute_address as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);

        // Checks if a page boundary was crossed to determine number of cycles
        if  (base_address & 0xFF00) != (absolute_address & 0xFF00) {
            5
        } else {
            4
        }
    }

    fn lda_indirect_x(&mut self) -> usize {
        self.pc += 1;
        let address1 = self.memory[self.pc as usize] as u16;
        let address2 = (address1 + self.x as u16) % 0x100;
        let low_byte = self.memory[address2 as usize];
        let high_byte = self.memory[((address2 + 1) % 0x100) as usize] ;
        let pointer = (high_byte as u16) << 8 | low_byte as u16;
        self.a = self.memory[pointer as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);
        6
    }

    fn lda_indirect_y(&mut self) -> usize {
        self.pc += 1;
        let pointer = self.memory[self.pc as usize] as u16;
        let low_byte = self.memory[(pointer % 0x100) as usize];
        let high_byte = self.memory[((pointer + 1) % 0x100)as usize] as u16;
        let base_address = (high_byte << 8) | low_byte as u16;
        let final_address = base_address + self.y as u16;
        self.a = self.memory[final_address as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);

        if  (base_address & 0xFF00) != (final_address & 0xFF00) {
            6
        } else {
            5
        }
    }
    //STA opcodes _________________________________________________________________________________
    fn sta_zero_page(&mut self) -> usize {
        self.pc += 1;
        let address = self.memory[self.pc as usize] as u16;
        self.memory[address as usize] = self.a;
        self.pc += 1;
        3
    }

    fn sta_zero_page_x(&mut self) -> usize {
        self.pc += 1;
        let address = self.memory[self.pc as usize] as u16;
        let xaddress = (address + self.x as u16) % 0x100;
        self.memory[xaddress as usize] = self.a;
        self.pc += 1;
        4
    }

    fn sta_absolute(&mut self) -> usize {
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        let absolute_address = (high_byte << 8) | low_byte;
        self.memory[absolute_address as usize] = self.a;
        4
    }

    fn sta_absolute_x(&mut self) -> usize {
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        let absolute_address = (high_byte << 8) | low_byte;
        let final_address = absolute_address + self.x as u16;
        self.memory[final_address as usize] = self.a;
        self.pc += 1;
        5
    }

    fn sta_absolute_y(&mut self) -> usize {
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        let absolute_address = (high_byte << 8) | low_byte;
        let final_address = absolute_address + self.y as u16;
        self.memory[final_address as usize] = self.a;
        self.pc += 1;
        5
    }
    fn sta_indirect_x(&mut self) -> usize {
        self.pc += 1;
        let address1 = self.memory[self.pc as usize];
        let address2 = address1.wrapping_add(self.x);
        let low_byte = self.memory[address2 as usize];
        let high_byte = self.memory[address2.wrapping_add(1) as usize] ;
        let pointer = (high_byte as u16) << 8 | low_byte as u16;
        self.memory[pointer as usize] = self.a;
        self.pc += 1;
        6
    }

    fn sta_indirect_y(&mut self) -> usize {
        self.pc += 1;
        let pointer = self.memory[self.pc as usize];
        let low_byte = self.memory[pointer as usize];
        let high_byte = self.memory[pointer.wrapping_add(1) as usize] as u16;
        let base_address = (high_byte << 8) | low_byte as u16;
        let final_address = base_address + self.y as u16;
        self.memory[final_address as usize] = self.a;
        self.pc += 1;
        6
    }

    // LDX opcodes ________________________________________________________________________________
    fn ldx_immediate(&mut self) -> usize {
        self.pc += 1;
        self.x = self.memory[self.pc as usize];
        self.pc += 1;
        self.update_nz_flags(self.x);
        2
    }

    fn ldx_zero_page(&mut self) -> usize {
        self.pc += 1;
        let address = self.memory[self.pc as usize] as u16;
        self.x = self.memory[address as usize];
        self.pc += 1;
        self.update_nz_flags(self.x);
        3
    }

    fn ldx_zero_page_y(&mut self) -> usize {
        self.pc += 1;
        let address1 = self.memory[self.pc as usize];
        let address2 = address1.wrapping_add(self.y);
        self.x = self.memory[address2 as usize];
        self.pc += 1;
        self.update_nz_flags(self.x);
        4
    }

    fn ldx_absolute(&mut self) -> usize {
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        let absolute_address = (high_byte << 8) | low_byte;
        self.x = self.memory[absolute_address as usize];
        self.pc += 1;
        self.update_nz_flags(self.x);
        4
    }

    fn ldx_absolute_y(&mut self) -> usize {
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        let base_address = (high_byte << 8) | low_byte;
        let absolute_address = base_address + self.y as u16;
        self.x = self.memory[absolute_address as usize];
        self.pc += 1;
        self.update_nz_flags(self.x);

        // Checks if a page boundary was crossed to determine number of cycles
        if  (base_address & 0xFF00) != (absolute_address & 0xFF00) {
            5
        } else {
            4
        }
    }

    // Transfer opcodes ____________________________________________________________________________

    fn tax_implied(&mut self) -> usize {
        self.x = self.a;
        self.pc += 1;
        self.update_nz_flags(self.x);
        2
    }

    fn tay_implied(&mut self) -> usize {
        self.y = self.a;
        self.pc += 1;
        self.update_nz_flags(self.y);
        2
    }

    fn tsx_implied(&mut self) -> usize {
        self.x = self.sp;
        self.pc += 1;
        self.update_nz_flags(self.x);
        2
    }

    fn txa_implied(&mut self) -> usize {
        self.a = self.x;
        self.pc += 1;
        self.update_nz_flags(self.a);
        2
    }

    fn txs_implied(&mut self) -> usize {
        self.sp = self.x;
        self.pc += 1;
        2
    }

    fn tya_implied(&mut self) -> usize {
        self.a = self.y;
        self.pc += 1;
        self.update_nz_flags(self.a);
        2
    }
    // Jumps opcodes _______________________________________________________________________________

    fn jmp_absolute(&mut self) -> usize {
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        self.pc = (high_byte << 8) | low_byte;
        3
    }

    fn jmp_indirect(&mut self) -> usize {
        self.pc += 1;
        let pointer_low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let pointer_high_byte = self.memory[self.pc as usize] as u16;
        let pointer = (pointer_high_byte << 8) | pointer_low_byte;
        let low_byte = self.memory[pointer as usize] as u16;
        let high_byte = self.memory[pointer.wrapping_add(1) as usize] as u16;
        self.pc = (high_byte << 8) | low_byte;
        5
    }

    fn jsr_absolute(&mut self) -> usize {
        let return_address = self.pc + 2;
        // Pushes high byte
        self.stack_push(((return_address - 1) >> 8) as u8);
        // Pushes low byte
        self.stack_push(return_address as u8 - 1);
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        self.pc = (high_byte << 8) | low_byte;
        6
    }

    fn rts_implied(&mut self) -> usize {
        let low_byte = self.stack_pop() as u16;
        let high_byte = self.stack_pop() as u16;
        let return_address = (high_byte << 8) | low_byte;
        self.pc = return_address + 1;
        6
    }

    //Branching opcodes ____________________________________________________________________________
    fn beq_relative(&mut self) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = self.memory[self.pc as usize] as i8;
        self.pc += 1;
        if self.check_zero_flag() {
            self.pc = (self.pc as i16 + offset as i16) as u16;
            if  (initial_address & 0xFF00) != (self.pc & 0xFF00) {
                4
            } else {
                3
            }
        } else {
            2
        }
    }
    fn bne_relative(&mut self) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = self.memory[self.pc as usize] as i8;
        self.pc += 1;
        if !self.check_zero_flag() {
            self.pc = (self.pc as i16 + offset as i16) as u16;
            if  (initial_address & 0xFF00) != (self.pc & 0xFF00) {
                4
            } else {
                3
            }
        } else {
            2
        }
    }

    fn bcs_relative(&mut self) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = self.memory[self.pc as usize] as i8;
        self.pc += 1;
        if self.check_carry_flag() {
            self.pc = (self.pc as i16 + offset as i16) as u16;
            if  (initial_address & 0xFF00) != (self.pc & 0xFF00) {
                4
            } else {
                3
            }
        } else {
            2
        }
    }

    fn bcc_relative(&mut self) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = self.memory[self.pc as usize] as i8;
        self.pc += 1;
        if !self.check_carry_flag() {
            self.pc = (self.pc as i16 + offset as i16) as u16;
            if  (initial_address & 0xFF00) != (self.pc & 0xFF00) {
                4
            } else {
                3
            }
        } else {
            2
        }
    }

    fn bmi_relative(&mut self) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = self.memory[self.pc as usize] as i8;
        self.pc += 1;
        if self.check_negative_flag() {
            self.pc = (self.pc as i16 + offset as i16) as u16;
            if  (initial_address & 0xFF00) != (self.pc & 0xFF00) {
                4
            } else {
                3
            }
        } else {
            2
        }
    }
    fn bpl_relative(&mut self) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = self.memory[self.pc as usize] as i8;
        self.pc += 1;
        if !self.check_negative_flag() {
            self.pc = (self.pc as i16 + offset as i16) as u16;
            if  (initial_address & 0xFF00) != (self.pc & 0xFF00) {
                4
            } else {
                3
            }
        } else {
            2
        }
    }

    fn bvs_relative(&mut self) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = self.memory[self.pc as usize] as i8;
        self.pc += 1;
        if self.check_overflow_flag() {
            self.pc = (self.pc as i16 + offset as i16) as u16;
            if  (initial_address & 0xFF00) != (self.pc & 0xFF00) {
                4
            } else {
                3
            }
        } else {
            2
        }
    }

    fn bvc_relative(&mut self) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = self.memory[self.pc as usize] as i8;
        self.pc += 1;
        if !self.check_overflow_flag() {
            self.pc = (self.pc as i16 + offset as i16) as u16;
            if  (initial_address & 0xFF00) != (self.pc & 0xFF00) {
                4
            } else {
                3
            }
        } else {
            2
        }
    }
}





