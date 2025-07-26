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

            0xC9 => self.cmp_immediate(),
            0xC5 => self.cmp_zero_page(),
            0xCD => self.cmp_absolute(),

            0xE0 => self.cpx_immediate(),
            0xE4 => self.cpx_zero_page(),
            0xEC => self.cpx_absolute(),

            0xC0 => self.cpy_immediate(),
            0xC4 => self.cpy_zero_page(),
            0xCC => self.cpy_absolute(),

            0x69 => self.adc_immediate(),
            0x65 => self.adc_zero_page(),
            0x75 => self.adc_zero_page_x(),
            0x6D => self.adc_absolute(),
            0x7D => self.adc_absolute_x(),
            0x79 => self.adc_absolute_y(),
            0x61 => self.adc_indirect_x(),
            0x71 => self.adc_indirect_y(),

            0xE9 => self.sbc_immediate(),
            0xE5 => self.sbc_zero_page(),
            0xF5 => self.sbc_zero_page_x(),
            0xED => self.sbc_absolute(),
            0xFD => self.sbc_absolute_x(),
            0xF9 => self.sbc_absolute_y(),
            0xE1 => self.sbc_indirect_x(),
            0xF1 => self.sbc_indirect_y(),









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
    // Addressing functions ________________________________________________________________________
    fn get_zero_page_addr(&mut self) -> u16 {
        self.pc += 1;
        let address = self.memory[self.pc as usize] as u16;
        address
    }

    fn get_zero_page_x_addr(&mut self) -> u16 {
        self.pc += 1;
        let address1 = self.memory[self.pc as usize] as u16;
        let address2 = (address1 + self.x as u16) % 0x100;
        address2
    }

    fn get_zero_page_y_addr(&mut self) -> u16 {
        self.pc += 1;
        let address1 = self.memory[self.pc as usize] as u16;
        let address2 = (address1 + self.y as u16) % 0x100;
        address2
    }

    fn get_absolute_addr(&mut self) -> u16 {
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        let absolute_address = (high_byte << 8) | low_byte;
        absolute_address
    }

    fn get_absolute_x_addr(&mut self) -> u16 {
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        let base_address = (high_byte << 8) | low_byte;
        let absolute_address = base_address + self.x as u16;
        absolute_address
    }

    fn get_absolute_y_addr(&mut self) -> (u16) {
        self.pc += 1;
        let low_byte = self.memory[self.pc as usize] as u16;
        self.pc += 1;
        let high_byte = self.memory[self.pc as usize] as u16;
        let base_address = (high_byte << 8) | low_byte;
        let absolute_address = base_address + self.y as u16;
        absolute_address
    }

    fn get_indirect_x_addr(&mut self) -> u16 {
        self.pc += 1;
        let address1 = self.memory[self.pc as usize] as u16;
        let address2 = (address1 + self.x as u16) % 0x100;
        let low_byte = self.memory[address2 as usize];
        let high_byte = self.memory[((address2 + 1) % 0x100) as usize];
        let pointer = (high_byte as u16) << 8 | low_byte as u16;
        pointer
    }

    fn get_indirect_y_addr(&mut self) -> u16 {
        self.pc += 1;
        let pointer = self.memory[self.pc as usize] as u16;
        let low_byte = self.memory[(pointer % 0x100) as usize];
        let high_byte = self.memory[((pointer + 1) % 0x100) as usize] as u16;
        let base_address = (high_byte << 8) | low_byte as u16;
        let final_address = base_address + self.y as u16;
        final_address
    }

    fn check_if_page_crossed45(&mut self, start: u16, end: u16) -> usize {
        if  (start as u16 & 0xFF00) != (end & 0xFF00) {
            5
        } else {
            4
        }
    }

    fn check_if_page_crossed56(&mut self, start: u16, end: u16) -> usize {
        if  (start as u16 & 0xFF00) != (end & 0xFF00) {
            6
        } else {
            5
        }
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
        let address = self.get_zero_page_addr();
        self.a = self.memory[address as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);
        3
    }



    fn lda_zero_page_x(&mut self) -> usize {
        let address2 = self.get_zero_page_x_addr();
        self.a = self.memory[address2 as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);
        4
    }

    fn lda_absolute(&mut self) -> usize {
        let absolute_address = self.get_absolute_addr();
        self.a = self.memory[absolute_address as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);
        4
    }


    fn lda_absolute_x(&mut self) -> usize {
        let base_address = self.memory[self.pc as usize];
        let absolute_address = self.get_absolute_x_addr();
        self.a = self.memory[absolute_address as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);

        // CHecks if a page boundary was crossed to determine number of cycles
        if  (base_address as u16 & 0xFF00) != (absolute_address & 0xFF00) {
            5
        } else {
            4
        }
    }

    fn lda_absolute_y(&mut self) -> usize {
        let base_address = self.memory[self.pc as usize];
        let absolute_address =  self.get_absolute_y_addr();
        self.a = self.memory[absolute_address as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);

        // Checks if a page boundary was crossed to determine number of cycles
        if  (base_address as u16 & 0xFF00) != (absolute_address & 0xFF00) {
            5
        } else {
            4
        }
    }


    fn lda_indirect_x(&mut self) -> usize {
        let pointer = self.get_indirect_x_addr();
        self.a = self.memory[pointer as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);
        6
    }

    fn lda_indirect_y(&mut self) -> usize {
        let base_address = self.memory[self.pc as usize];
        let final_address = self.get_indirect_y_addr();
        self.a = self.memory[final_address as usize];
        self.pc += 1;
        self.update_nz_flags(self.a);

        if  (base_address as u16 & 0xFF00) != (final_address & 0xFF00) {
            6
        } else {
            5
        }
    }



    //STA opcodes _________________________________________________________________________________
    fn sta_zero_page(&mut self) -> usize {
        let address = self.get_zero_page_addr();
        self.memory[address as usize] = self.a;
        self.pc += 1;
        3
    }

    fn sta_zero_page_x(&mut self) -> usize {
        let xaddress = self.get_zero_page_x_addr();
        self.memory[xaddress as usize] = self.a;
        self.pc += 1;
        4
    }

    fn sta_absolute(&mut self) -> usize {
        let absolute_address = self.get_absolute_addr();
        self.memory[absolute_address as usize] = self.a;
        4
    }

    fn sta_absolute_x(&mut self) -> usize {
        let final_address = self.get_absolute_x_addr();
        self.memory[final_address as usize] = self.a;
        self.pc += 1;
        5
    }

    fn sta_absolute_y(&mut self) -> usize {
        let final_address = self.get_absolute_y_addr();
        self.memory[final_address as usize] = self.a;
        self.pc += 1;
        5
    }
    fn sta_indirect_x(&mut self) -> usize {
        let pointer = self.get_indirect_x_addr();
        self.memory[pointer as usize] = self.a;
        self.pc += 1;
        6
    }

    fn sta_indirect_y(&mut self) -> usize {
        let final_address = self.get_indirect_y_addr();
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
        let address = self.get_zero_page_addr();
        self.x = self.memory[address as usize];
        self.pc += 1;
        self.update_nz_flags(self.x);
        3
    }

    fn ldx_zero_page_y(&mut self) -> usize {
        let address2 = self.get_zero_page_y_addr();
        self.x = self.memory[address2 as usize];
        self.pc += 1;
        self.update_nz_flags(self.x);
        4
    }

    fn ldx_absolute(&mut self) -> usize {
        let absolute_address = self.get_absolute_addr();
        self.x = self.memory[absolute_address as usize];
        self.pc += 1;
        self.update_nz_flags(self.x);
        4
    }

    fn ldx_absolute_y(&mut self) -> usize {
        let base_address = self.memory[self.pc as usize] as u16;
        let absolute_address = self.get_absolute_y_addr();
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
        let address = self.get_absolute_addr();
        self.pc = address;
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
        self.stack_push((return_address - 1) as u8);
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

    // Comparison opcodes __________________________________________________________________________

    fn cmp_immediate(&mut self) -> usize {
        self.pc += 1;
        let (result, borrow) = (self.a).overflowing_sub(self.memory[self.pc as usize]);
        self.p &= !(ZERO_FLAG | NEGATIVE_FLAG | CARRY_FLAG);

        if result == 0 {self.p |= ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= NEGATIVE_FLAG};
        if !borrow {self.p |= CARRY_FLAG};
        self.pc += 1;
        2
    }

    fn cmp_zero_page(&mut self) -> usize {
        let value = self.memory[self.get_zero_page_addr() as usize];
        let (result, borrow) = self.a.overflowing_sub(value);

        self.p &= !(ZERO_FLAG | NEGATIVE_FLAG | CARRY_FLAG);
        if result == 0 {self.p |= ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= NEGATIVE_FLAG};
        if !borrow {self.p |= CARRY_FLAG};
        self.pc += 1;
        3
    }

    fn cmp_absolute(&mut self) -> usize {
        let address = self.get_absolute_addr();
        let value = self.memory[address as usize];
        let (result, borrow) = self.a.overflowing_sub(value);

        self.p &= !(ZERO_FLAG | NEGATIVE_FLAG | CARRY_FLAG);
        if result == 0 {self.p |= ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= NEGATIVE_FLAG};
        if !borrow {self.p |= CARRY_FLAG};
        self.pc += 1;
        4
    }

    fn cpx_immediate(&mut self) -> usize {
        self.pc += 1;
        let (result, borrow) = self.x.overflowing_sub(self.memory[self.pc as usize]);
        self.p &= !(ZERO_FLAG | NEGATIVE_FLAG | CARRY_FLAG);

        if result == 0 {self.p |= ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= NEGATIVE_FLAG};
        if !borrow {self.p |= CARRY_FLAG};
        self.pc += 1;
        2
    }

    fn cpx_zero_page(&mut self) -> usize {
        let value = self.memory[self.get_zero_page_addr() as usize];
        let (result, borrow) = self.x.overflowing_sub(value);

        self.p &= !(ZERO_FLAG | NEGATIVE_FLAG | CARRY_FLAG);
        if result == 0 {self.p |= ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= NEGATIVE_FLAG};
        if !borrow {self.p |= CARRY_FLAG};
        self.pc += 1;
        3
    }

    fn cpx_absolute(&mut self) -> usize {
        let address = self.get_zero_page_addr();
        let value = self.memory[address as usize];
        let (result, borrow) = self.x.overflowing_sub(value);

        self.p &= !(ZERO_FLAG | NEGATIVE_FLAG | CARRY_FLAG);
        if result == 0 {self.p |= ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= NEGATIVE_FLAG};
        if !borrow {self.p |= CARRY_FLAG};
        self.pc += 1;
        4
    }

    fn cpy_immediate(&mut self) -> usize {
        self.pc += 1;
        let (result, borrow) = self.y.overflowing_sub(self.memory[self.pc as usize]);
        self.p &= !(ZERO_FLAG | NEGATIVE_FLAG | CARRY_FLAG);

        if result == 0 {self.p |= ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= NEGATIVE_FLAG};
        if !borrow {self.p |= CARRY_FLAG};
        self.pc += 1;
        2
    }

    fn cpy_zero_page(&mut self) -> usize {
        let value = self.memory[self.get_zero_page_addr() as usize];
        let (result, borrow) = self.y.overflowing_sub(value);

        self.p &= !(ZERO_FLAG | NEGATIVE_FLAG | CARRY_FLAG);
        if result == 0 {self.p |= ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= NEGATIVE_FLAG};
        if !borrow {self.p |= CARRY_FLAG};
        self.pc += 1;
        3
    }

    fn cpy_absolute(&mut self) -> usize {
        let value = self.memory[self.get_absolute_addr() as usize];
        let (result, borrow) = self.y.overflowing_sub(value);

        self.p &= !(ZERO_FLAG | NEGATIVE_FLAG | CARRY_FLAG);
        if result == 0 {self.p |= ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= NEGATIVE_FLAG};
        if !borrow {self.p |= CARRY_FLAG};
        self.pc += 1;
        4
    }

    // Arithmatic opcodes __________________________________________________________________________

    fn adc_immediate(&mut self) -> usize {
        self.pc += 1;
        let value = self.memory[self.pc as usize];
        let result = self.a as u16 + value as u16 + (self.p & CARRY_FLAG) as u16;

        self.set_flag(CARRY_FLAG, result > 255);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        2
    }

    fn adc_zero_page(&mut self) -> usize {
        let value = self.memory[self.get_zero_page_addr() as usize];
        let result = self.a as u16 + value as u16 + (self.p & CARRY_FLAG) as u16;

        self.set_flag(CARRY_FLAG, result > 255);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        3
    }

    fn adc_zero_page_x(&mut self) -> usize {
        let value = self.memory[self.get_zero_page_x_addr() as usize];
        let result = self.a as u16 + value as u16 + (self.p & CARRY_FLAG) as u16;

        self.set_flag(CARRY_FLAG, result > 255);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        4
    }


    fn adc_absolute(&mut self) -> usize {
        let value = self.memory[self.get_absolute_addr() as usize];
        let result = self.a as u16 + value as u16 + (self.p & CARRY_FLAG) as u16;

        self.set_flag(CARRY_FLAG, result > 255);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        4
    }
    fn adc_absolute_x(&mut self) -> usize {
        let start_addr = self.pc;
        let end_addr = self.get_absolute_x_addr();
        let value = self.memory[end_addr as usize];
        let result = self.a as u16 + value as u16 + (self.p & CARRY_FLAG) as u16;

        self.set_flag(CARRY_FLAG, result > 255);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        self.check_if_page_crossed45(start_addr, end_addr)
    }

    fn adc_absolute_y(&mut self) -> usize {
        let start_addr = self.pc;
        let end_addr = self.get_absolute_y_addr();
        let value = self.memory[end_addr as usize];
        let result = self.a as u16 + value as u16 + (self.p & CARRY_FLAG) as u16;

        self.set_flag(CARRY_FLAG, result > 255);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        self.check_if_page_crossed45(start_addr, end_addr)

    }

    fn adc_indirect_x(&mut self) -> usize {
        let end_addr = self.get_indirect_x_addr();
        let value = self.memory[end_addr as usize];
        let result = self.a as u16 + value as u16 + (self.p & CARRY_FLAG) as u16;

        self.set_flag(CARRY_FLAG, result > 255);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        6
    }

    fn adc_indirect_y(&mut self) -> usize {
        let start_addr = self.pc;
        let end_addr = self.get_indirect_y_addr();
        let value = self.memory[end_addr as usize];
        let result = self.a as u16 + value as u16 + (self.p & CARRY_FLAG) as u16;

        self.set_flag(CARRY_FLAG, result > 255);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        self.check_if_page_crossed56(start_addr, end_addr)

    }

    

    // Subtract with Carry _________________________________________________________________________
    fn sbc_immediate(&mut self) -> usize {
        self.pc += 1;
        let value = self.memory[self.pc as usize] as u16;
        let result = self.a as u16 - value - (1 - (self.p  & CARRY_FLAG) as u16);

        self.set_flag(CARRY_FLAG, result < 256);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        2
    }

    fn sbc_zero_page(&mut self) -> usize {
        let value = self.memory[self.get_zero_page_addr() as usize] as u16;
        let result = self.a as u16 - value - (1 - (self.p  & CARRY_FLAG) as u16);

        self.set_flag(CARRY_FLAG, result < 256);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        3
    }

    fn sbc_zero_page_x(&mut self) -> usize {
        let value = self.memory[self.get_zero_page_x_addr() as usize] as u16;
        let result = self.a as u16 - value - (1 - (self.p  & CARRY_FLAG) as u16);

        self.set_flag(CARRY_FLAG, result < 256);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        3
    }

    fn sbc_absolute(&mut self) -> usize {
        let value = self.memory[self.get_absolute_addr() as usize] as u16;
        let result = self.a as u16 - value - (1 - (self.p  & CARRY_FLAG) as u16);

        self.set_flag(CARRY_FLAG, result < 256);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        4
    }

    fn sbc_absolute_x(&mut self) -> usize {
        let start_addr = self.pc;
        let end_addr = self.get_absolute_x_addr();
        let value = self.memory[end_addr as usize] as u16;
        let result = self.a as u16 - value - (1 - (self.p  & CARRY_FLAG) as u16);

        self.set_flag(CARRY_FLAG, result < 256);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        self.check_if_page_crossed45(start_addr, end_addr)
    }

    fn sbc_absolute_y(&mut self) -> usize {
        let start_addr = self.pc as u16;
        let end_addr = self.get_absolute_y_addr();
        let value = self.memory[end_addr as usize] as u16;
        let result = self.a as u16 - value - (1 - (self.p  & CARRY_FLAG) as u16);

        self.set_flag(CARRY_FLAG, result < 256);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;

        self.check_if_page_crossed45(start_addr, end_addr)
    }

    fn sbc_indirect_x(&mut self) -> usize {
        let start_addr = self.pc as u16;
        let end_addr = self.get_indirect_x_addr();
        let value = self.memory[end_addr as usize] as u16;
        let result = self.a as u16 - value - (1 - (self.p & CARRY_FLAG) as u16);

        self.set_flag(CARRY_FLAG, result < 256);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        6
    }

    fn sbc_indirect_y(&mut self) -> usize {
        let start_addr = self.pc as u16;
        let end_addr = self.get_indirect_y_addr();
        let value = self.memory[end_addr as usize] as u16;
        let result = self.a as u16 - value - (1 - (self.p & CARRY_FLAG) as u16);

        self.set_flag(CARRY_FLAG, result < 256);
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
        self.set_flag(ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.set_flag(OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        self.check_if_page_crossed56(start_addr, end_addr)
    }
}








