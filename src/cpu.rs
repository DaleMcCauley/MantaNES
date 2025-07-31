use crate::cartridge::Cartridge;
use crate::flags::{FlagRegister, cpu};
use crate::bus::Bus;
pub struct Cpu6502 {
    pub a: u8,
    pub x: u8,
    pub y: u8,
    pub pc: u16,
    pub sp: u8,
    pub p: u8,
    pub cycles: usize,


}

impl Cpu6502 {

    pub fn init_cpu() -> Cpu6502 {
        let cpu: Cpu6502 = Cpu6502 {
            a: 0,
            x: 0,
            y: 0,
            // Program counter
            pc: 0,
            // Stack Pointer
            sp: 0xFD,
            // Status flags
            p: 0x24,
            cycles: 0,
            

        };
        cpu
    }

    pub fn reset(&mut self, bus: &mut Bus) {
        let low_byte = bus.read(0xFFFC) as u16;
        let high_byte = bus.read(0xFFFD) as u16;
        let reset_vector = (high_byte << 8) | low_byte;
        if reset_vector == 0 {
            println!("Reset vector is zero, defaulting to 0x8000");
            self.pc = 0x8000
        } else {
            self.pc = reset_vector;
            println!("Loaded reset vector: {}", reset_vector);
        }
        self.sp = 0xFD;
        self.p = 0x24;
    }
    pub fn load_rom(&mut self, bus: &mut Bus, cartridge: Cartridge) {
        bus.load_cartridge(cartridge);
    }
    pub fn step(&mut self, bus: &mut Bus) {
        let opcode = bus.read(self.pc);
        println!("PC: {:#06x}, Opcode: {:#04X}", self.pc, opcode);
// Main decoder hub
        let additional_cycles: usize = match opcode {
            // ROR
            0x6A => {println!("ror_accumulator"); self.ror_accumulator(bus)},
            0x66 => self.ror_zero_page(bus),
            0x76 => self.ror_zero_page_x(bus),
            0x6E => self.ror_absolute(bus),
            0x7E => self.ror_absolute_x(bus),
            // ROL
            0x2A => self.rol_accumulator(bus),
            0x26 => self.rol_zero_page(bus),
            0x36 => self.rol_zero_page_x(bus),
            0x2E => self.rol_absolute(bus),
            0x3E => self.rol_absolute_x(bus),
            //NOP
            0xEA => self.nop_implied(bus),
            // Set Flag
            0x38 => self.sec_implied(bus),
            0xF8 => self.sed_implied(bus),
            0x78 => self.sei_implied(bus),
            // Shift
            0x4A => self.lsr_accumulator(bus),
            0x46 => self.lsr_zero_page(bus),
            0x56 => self.lsr_zero_page_x(bus),
            0x4E => self.lsr_absolute(bus),
            0x5E => self.lsr_absolute_x(bus),

            0x0A => self.asl_accumulator(bus),
            0x06 => self.asl_zero_page(bus),
            0x16 => self.asl_zero_page_x(bus),
            0x0E => self.asl_absolute(bus),
            0x1E => self.asl_absolute_x(bus),

            // Clear Flag
            0xD8 => self.cld_implied(bus),
            0x18 => self.clc_implied(bus),
            0x58 => self.cli_implied(bus),
            0xB8 => self.clv_implied(bus),
            // Stack
            0x48 => self.pha_implied(bus),
            0x08 => self.php_implied(bus),
            0x68 => self.pla_implied(bus),
            0x28 => self.plp_implied(bus),
            // Load A
            0xA9 => self.lda_immediate(bus),
            0xA5 => self.lda_zero_page(bus),
            0xB5 => self.lda_zero_page_x(bus),
            0xAD => self.lda_absolute(bus),
            0xBD => self.lda_absolute_x(bus),
            0xB9 => self.lda_absolute_y(bus),
            0xA1 => self.lda_indirect_x(bus),
            0xB1 => self.lda_indirect_y(bus),
            // Store A
            0x85 => self.sta_zero_page(bus),
            0x8D => self.sta_absolute(bus),
            0x9D => self.sta_absolute_x(bus),
            0x99 => self.sta_absolute_y(bus),
            0x81 => self.sta_indirect_x(bus),
            0x91 => self.sta_indirect_y(bus),
            // Store X
            0x86 => self.stx_zero_page(bus),
            0x96 => self.stx_zero_page_y(bus),
            0x8E => self.stx_absolute(bus),
            // Store Y
            0x84 => self.sty_zero_page(bus),
            0x94 => self.sty_zero_page_x(bus),
            0x8C => self.sty_absolute(bus),
            // Load X
            0xA2 => self.ldx_immediate(bus),
            0xA6 => self.ldx_zero_page(bus),
            0xB6 => self.ldx_zero_page_y(bus),
            0xAE => self.ldx_absolute(bus),
            0xBE => self.ldx_absolute_y(bus),
            // Load Y
            0xA0 => self.ldy_immediate(bus),
            0xA4 => self.ldy_zero_page(bus),
            0xB4 => self.ldy_zero_page_x(bus),
            0xAC => self.ldy_absolute(bus),
            0xBC => self.ldy_absolute_x(bus),
            // Transfer
            0xAA => self.tax_implied(bus),
            0xA8 => self.tay_implied(bus),
            0xBA => self.tsx_implied(bus),
            0x8A => self.txa_implied(bus),
            0x9A => self.txs_implied(bus),
            0x98 => self.tya_implied(bus),
            // Jump
            0x4C => self.jmp_absolute(bus),
            0x6C => self.jmp_indirect(bus),
            0x20 => self.jsr_absolute(bus),
            0x60 => self.rts_implied(bus),
            0x00 => self.brk_implied(bus),
            0x40 => self.rti_implied(bus),
            // Branching
            0xF0 => self.beq_relative(bus),
            0x90 => self.bcc_relative(bus),
            0xB0 => self.bcs_relative(bus),
            0x30 => self.bmi_relative(bus),
            0xD0 => self.bne_relative(bus),
            0x10 => self.bpl_relative(bus),
            0x50 => self.bvc_relative(bus),
            0x70 => self.bvs_relative(bus),
            // Compare
            0xC9 => self.cmp_immediate(bus),
            0xC5 => self.cmp_zero_page(bus),
            0xCD => self.cmp_absolute(bus),

            0xE0 => self.cpx_immediate(bus),
            0xE4 => self.cpx_zero_page(bus),
            0xEC => self.cpx_absolute(bus),

            0xC0 => self.cpy_immediate(bus),
            0xC4 => self.cpy_zero_page(bus),
            0xCC => self.cpy_absolute(bus),
            // Add with carry
            0x69 => self.adc_immediate(bus),
            0x65 => self.adc_zero_page(bus),
            0x75 => self.adc_zero_page_x(bus),
            0x6D => self.adc_absolute(bus),
            0x7D => self.adc_absolute_x(bus),
            0x79 => self.adc_absolute_y(bus),
            0x61 => self.adc_indirect_x(bus),
            0x71 => self.adc_indirect_y(bus),
            // Subtract with carry
            0xE9 => self.sbc_immediate(bus),
            0xE5 => self.sbc_zero_page(bus),
            0xF5 => self.sbc_zero_page_x(bus),
            0xED => self.sbc_absolute(bus),
            0xFD => self.sbc_absolute_x(bus),
            0xF9 => self.sbc_absolute_y(bus),
            0xE1 => self.sbc_indirect_x(bus),
            0xF1 => self.sbc_indirect_y(bus),
            // Logic
            0x29 => self.and_immediate(bus),
            0x25 => self.and_zero_page(bus),
            0x35 => self.and_zero_page_x(bus),
            0x2D => self.and_absolute(bus),
            0x3D => self.and_absolute_x(bus),
            0x39 => self.and_absolute_y(bus),
            0x21 => self.and_indirect_x(bus),
            0x31 => self.and_indirect_y(bus),

            0x09 => self.ora_immediate(bus),
            0x05 => self.ora_zero_page(bus),
            0x15 => self.ora_zero_page_x(bus),
            0x0D => self.ora_absolute(bus),
            0x1D => self.ora_absolute_x(bus),
            0x19 => self.ora_absolute_y(bus),
            0x01 => self.ora_indirect_x(bus),
            0x11 => self.ora_indirect_y(bus),

            0x49 => self.eor_immediate(bus),
            0x45 => self.eor_zero_page(bus),
            0x55 => self.eor_zero_page_x(bus),
            0x4D => self.eor_absolute(bus),
            0x5D => self.eor_absolute_x(bus),
            0x59 => self.eor_absolute_y(bus),
            0x41 => self.eor_indirect_x(bus),
            0x51 => self.eor_indirect_y(bus),

            0x24 => self.bit_zero_page(bus),
            0x2C => self.bit_absolute(bus),
            // Increments
            0xE6 => self.inc_zero_page(bus),
            0xF6 => self.inc_zero_page_x(bus),
            0xEE => self.inc_absolute(bus),
            0xFE => self.inc_absolute_x(bus),
            0xE8 => self.inx_implied(bus),
            0xC8 => self.iny_implied(bus),
            // Decrements
            0xC6 => self.dec_zero_page(bus),
            0xD6 => self.dec_zero_page_x(bus),
            0xCE => self.dec_absolute(bus),
            0xDE => self.dec_absolute_x(bus),
            0xCA => self.dex_implied(bus),
            0x88 => self.dey_implied(bus),

            _ => {println!("Opcode not interpreted: {}", opcode);
                0},
        };
        self.cycles += additional_cycles;
    }
    /*
    fn set_flag(&mut self, bus: &mut Bus, flag:u8, condition: bool) {
        if condition {
            self.p |= flag;
        } else {
            self.p &= !(flag);
        }
    }
         */
    fn update_nz_flags(&mut self, bus: &mut Bus, value: u8) {
        self.p.set_flag(cpu::ZERO_FLAG, value == 0);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (value & 0x80) != 0);
    }

    fn check_zero_flag(&self) -> bool {
        self.p & cpu::ZERO_FLAG != 0b00000000
    }

    fn check_negative_flag(&self) -> bool {
        self.p & cpu::NEGATIVE_FLAG != 0b00000000
    }

    fn check_carry_flag(&self) -> bool {
        self.p & cpu::CARRY_FLAG != 0b00000000
    }

    fn check_overflow_flag(&self) -> bool {
        self.p & cpu::OVERFLOW_FLAG != 0b00000000
    }
    fn stack_push(&mut self, bus: &mut Bus, input: u8) {
        bus.write((0x0100 + self.sp as u16), input);
        self.sp = self.sp.wrapping_sub(1);
    }

    fn stack_pop(&mut self, bus: &mut Bus) -> u8 {
        self.sp = self.sp.wrapping_add(1);
        bus.read((0x0100 + self.sp as u16))
    }
    // Addressing functions ________________________________________________________________________
    fn get_zero_page_addr(&mut self, bus: &mut Bus) -> u16 {
        self.pc += 1;
        let address = bus.read(self.pc) as u16;
        address
    }

    fn get_zero_page_x_addr(&mut self, bus: &mut Bus) -> u16 {
        self.pc += 1;
        let address1 = bus.read(self.pc) as u16;
        let address2 = (address1 + self.x as u16) % 0x100;
        address2
    }

    fn get_zero_page_y_addr(&mut self, bus: &mut Bus) -> u16 {
        self.pc += 1;
        let address1 = bus.read(self.pc) as u16;
        let address2 = (address1 + self.y as u16) % 0x100;
        address2
    }

    fn get_absolute_addr(&mut self, bus: &mut Bus) -> u16 {
        self.pc += 1;
        let low_byte = bus.read(self.pc) as u16;
        self.pc += 1;
        let high_byte = bus.read(self.pc) as u16;
        let absolute_address = (high_byte << 8) | low_byte;
        absolute_address
    }

    fn get_absolute_x_addr(&mut self, bus: &mut Bus) -> (u16, u16) {
        self.pc += 1;
        let low_byte = bus.read(self.pc) as u16;
        self.pc += 1;
        let high_byte = bus.read(self.pc) as u16;
        let base_address = (high_byte << 8) | low_byte;
        let absolute_address = base_address + self.x as u16;
        (base_address, absolute_address)
    }

    fn get_absolute_y_addr(&mut self, bus: &mut Bus) -> (u16, u16) {
        self.pc += 1;
        let low_byte = bus.read(self.pc) as u16;
        self.pc += 1;
        let high_byte = bus.read(self.pc) as u16;
        let base_address = (high_byte << 8) | low_byte;
        let absolute_address = base_address + self.y as u16;
        (base_address,absolute_address)
    }

    fn get_indirect_x_addr(&mut self, bus: &mut Bus) -> u16 {
        self.pc += 1;
        let address1 = bus.read(self.pc) as u16;
        let address2 = (address1 + self.x as u16) % 0x100;
        let low_byte = bus.read(address2);
        let high_byte = bus.read(((address2 + 1) % 0x100));
        let pointer = (high_byte as u16) << 8 | low_byte as u16;
        pointer
    }

    fn get_indirect_y_addr(&mut self, bus: &mut Bus) -> (u16, u16) {
        self.pc += 1;
        let pointer = bus.read(self.pc) as u16;
        let low_byte = bus.read((pointer % 0x100));
        let high_byte = bus.read(((pointer + 1) % 0x100)) as u16;
        let base_address = (high_byte << 8) | low_byte as u16;
        let final_address = base_address + self.y as u16;
        (base_address, final_address)
    }

    fn check_if_page_crossed45(&mut self, bus: &mut Bus, start: u16, end: u16) -> usize {
        if  (start as u16 & 0xFF00) != (end & 0xFF00) {
            5
        } else {
            4
        }
    }

    fn check_if_page_crossed56(&mut self, bus: &mut Bus, start: u16, end: u16) -> usize {
        if  (start as u16 & 0xFF00) != (end & 0xFF00) {
            6
        } else {
            5
        }
    }
    // ROL opcodes _________________________________________________________________________________
    fn rol_accumulator(&mut self, bus: &mut Bus) -> usize {
        let old_carry = self.check_carry_flag();
        let new_carry = (self.a & 0x80) != 0;

        self.a = (self.a << 1) | (old_carry as u8);

        self.p.set_flag(cpu::CARRY_FLAG, new_carry);
        self.update_nz_flags(bus, self.a);
        self.pc += 1;

        2
    }

    fn rol_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        let old_carry = self.check_carry_flag();
        let mut value = bus.read(address);
        bus.write(address, value);
        let new_carry = (value & 0x80) != 0;

        value = (value << 1) | (old_carry as u8);

        self.p.set_flag(cpu::CARRY_FLAG, new_carry);
        bus.write(address, value);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        5
    }

    fn rol_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_x_addr(bus);
        let old_carry = self.check_carry_flag();
        let mut value = bus.read(address);
        bus.write(address, value);
        let new_carry = (value & 0x80) != 0;

        value = (value << 1) | (old_carry as u8);

        self.p.set_flag(cpu::CARRY_FLAG, new_carry);
        bus.write(address, value);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        6
    }

    fn rol_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        let old_carry = self.check_carry_flag();
        let mut value = bus.read(address);
        bus.write(address, value);
        let new_carry = (value & 0x80) != 0;

        value = (value << 1) | (old_carry as u8);

        self.p.set_flag(cpu::CARRY_FLAG, new_carry);
        bus.write(address, value);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        6
    }

    fn rol_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_x_addr(bus);
        let old_carry = self.check_carry_flag();
        let mut value = bus.read(address);
        bus.write(address, value);
        let new_carry = (value & 0x80) != 0;

        value = (value << 1) | (old_carry as u8);

        self.p.set_flag(cpu::CARRY_FLAG, new_carry);
        bus.write(address, value);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        7
    }

    // ROR opcodes _________________________________________________________________________________

    fn ror_accumulator(&mut self, bus: &mut Bus) -> usize {
        let old_carry = self.check_carry_flag();
        let new_carry = (self.a & 0x01) != 0;

        self.a = (self.a >> 1) | ((old_carry as u8) << 7);

        self.p.set_flag(cpu::CARRY_FLAG, new_carry);
        self.update_nz_flags(bus, self.a);
        self.pc += 1;

        2
    }

    fn ror_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        let old_carry = self.check_carry_flag();
        let value = bus.read(address);
        bus.write(address, value);
        let new_carry = (value & 0x01) != 0;

        let result = (value >> 1) | ((old_carry as u8) << 7);
        bus.write(address, result);
        self.p.set_flag(cpu::CARRY_FLAG, new_carry);
        self.update_nz_flags(bus, result);
        self.pc += 1;

        5
    }

    fn ror_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_x_addr(bus);
        let old_carry = self.check_carry_flag();
        let value = bus.read(address);
        bus.write(address, value);
        let new_carry = (value & 0x01) != 0;

        let result = (value >> 1) | ((old_carry as u8) << 7);
        bus.write(address, result);
        self.p.set_flag(cpu::CARRY_FLAG, new_carry);
        self.update_nz_flags(bus, result);
        self.pc += 1;

        6
    }

    fn ror_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        let old_carry = self.check_carry_flag();
        let value = bus.read(address);
        bus.write(address, value);
        let new_carry = (value & 0x01) != 0;

        let result = (value >> 1) | ((old_carry as u8) << 7);
        bus.write(address, result);
        self.p.set_flag(cpu::CARRY_FLAG, new_carry);
        self.update_nz_flags(bus, result);
        self.pc += 1;

        6
    }

    fn ror_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_x_addr(bus);
        let old_carry = self.check_carry_flag();
        let value = bus.read(address);
        bus.write(address, value);
        let new_carry = (value & 0x01) != 0;

        let result = (value >> 1) | ((old_carry as u8) << 7);
        bus.write(address, result);
        self.p.set_flag(cpu::CARRY_FLAG, new_carry);
        self.update_nz_flags(bus, result);
        self.pc += 1;

        7
    }
    // NOP _________________________________________________________________________________________

    fn nop_implied(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;

        2
    }
    // Set flag opcodes ____________________________________________________________________________
    fn sec_implied(&mut self, bus: &mut Bus) -> usize {
        self.p |= cpu::CARRY_FLAG;
        self.pc += 1;

        2
    }

    fn sei_implied(&mut self, bus: &mut Bus) -> usize {
        self.p |= cpu::INTERRUPT_DISABLE_FLAG;
        self.pc += 1;

        2
    }

    fn sed_implied(&mut self, bus: &mut Bus,) -> usize {
        self.p |= cpu::DECIMAL_FLAG;
        self.pc += 1;

        2
    }

    // Shifting opcodes ____________________________________________________________________________
    // LSR opcodes _________________________________________________________________________________
    fn lsr_accumulator(&mut self, bus: &mut Bus) -> usize {
        let carry = self.a & 0x01 != 0;
        self.a >>= 1;
        self.p.set_flag(cpu::CARRY_FLAG, carry);
        self.update_nz_flags(bus, self.a);
        self.pc += 1;

        2
    }

    fn lsr_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        let mut value = bus.read(address);
        let carry = value & 0x01 != 0;
        value >>= 1;
        bus.write(address, value);
        self.p.set_flag(cpu::CARRY_FLAG, carry);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        5
    }

    fn lsr_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_x_addr(bus);
        let mut value = bus.read(address);
        let carry = value & 0x01 != 0;
        value >>= 1;
        bus.write(address, value);
        self.p.set_flag(cpu::CARRY_FLAG, carry);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        6
    }

    fn lsr_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        let mut value = bus.read(address);
        let carry = value & 0x01 != 0;
        value >>= 1;
        bus.write(address, value);
        self.p.set_flag(cpu::CARRY_FLAG, carry);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        6
    }

    fn lsr_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_x_addr(bus);
        let mut value = bus.read(address);
        let carry = value & 0x01 != 0;
        value >>= 1;
        bus.write(address, value);
        self.p.set_flag(cpu::CARRY_FLAG, carry);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        7
    }

    // ASL opcodes _________________________________________________________________________________
    fn asl_accumulator(&mut self, bus: &mut Bus) -> usize {
        let carry = self.a >> 7 != 0;
        self.a <<= 1;
        self.p.set_flag(cpu::CARRY_FLAG, carry);
        self.update_nz_flags(bus, self.a);
        self.pc += 1;

        2
    }

    fn asl_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        let mut value = bus.read(address);
        let carry = value >> 7 != 0;
        value <<= 1;
        bus.write(address, value);
        self.p.set_flag(cpu::CARRY_FLAG, carry);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        5
    }

    fn asl_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_x_addr(bus);
        let mut value = bus.read(address);
        let carry = value >> 7 != 0;
        value <<= 1;
        bus.write(address, value);
        self.p.set_flag(cpu::CARRY_FLAG, carry);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        6
    }

    fn asl_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        let mut value = bus.read(address);
        let carry = value >> 7 != 0;
        value <<= 1;
        bus.write(address, value);
        self.p.set_flag(cpu::CARRY_FLAG, carry);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        6
    }

    fn asl_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_x_addr(bus);
        let mut value = bus.read(address);
        let carry = value >> 7 != 0;
        value <<= 1;
        bus.write(address, value);
        self.p.set_flag(cpu::CARRY_FLAG, carry);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        7
    }
    // Flag Clear opcodes __________________________________________________________________________

    fn clc_implied(&mut self, bus: &mut Bus) -> usize {
        self.p.set_flag(cpu::CARRY_FLAG, false);
        self.pc += 1;

        2
    }

    fn cld_implied(&mut self, bus: &mut Bus) -> usize {
        self.p.set_flag(cpu::DECIMAL_FLAG, false);
        self.pc += 1;

        2
    }

    fn cli_implied(&mut self, bus: &mut Bus) -> usize {
        self.p.set_flag(cpu::INTERRUPT_DISABLE_FLAG, false);
        self.pc += 1;

        2
    }

    fn clv_implied(&mut self, bus: &mut Bus) -> usize {
        self.p.set_flag(cpu::OVERFLOW_FLAG, false);
        self.pc += 1;

        2
    }



    //Stack opcodes ________________________________________________________________________________
    fn pha_implied(&mut self, bus: &mut Bus) -> usize {
        bus.write((0x0100 + self.sp as u16), self.a);
        self.sp = self.sp.wrapping_sub(1);
        self.pc += 1;

        3
    }

    fn php_implied(&mut self, bus: &mut Bus) -> usize {
        bus.write((0x0100 + self.sp as u16), self.p | 0b00010000);
        self.sp = self.sp.wrapping_sub(1);
        self.pc += 1;

        3
    }

    fn pla_implied(&mut self, bus: &mut Bus) -> usize {
        self.sp = self.sp.wrapping_add(1);
        let value =  bus.read((0x0100 + self.sp as u16));
        self.a = value;
        self.pc += 1;
        self.update_nz_flags(bus, value);

        4
    }

    fn plp_implied(&mut self, bus: &mut Bus) -> usize {
        self.sp = self.sp.wrapping_add(1);
        self.p = bus.read((0x0100 + self.sp as u16));
        self.pc += 1;

        4
    }

    // INC opcodes _________________________________________________________________________________

    fn inc_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        let mut value = bus.read(address);
        value += 1;
        bus.write(address, value);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        5
    }

    fn inc_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_x_addr(bus);
        let mut value = bus.read(address);
        value += 1;
        bus.write(address, value);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        6
    }

    fn inc_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        let mut value = bus.read(address);
        value += 1;
        bus.write(address, value);
        self.update_nz_flags(bus, value);
        self.pc += 1;
        6
    }

    fn inc_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_x_addr(bus);
        let mut value = bus.read(address);
        value += 1;
        bus.write(address, value);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        7
    }

    fn inx_implied(&mut self, bus: &mut Bus) -> usize {
        self.x += 1;
        self.update_nz_flags(bus, self.x);
        self.pc += 1;

        2
    }

    fn iny_implied(&mut self, bus: &mut Bus) -> usize {
        self.y += 1;
        self.update_nz_flags(bus, self.y);
        self.pc += 1;

        2
    }
    // DEC opcodes _________________________________________________________________________________
    fn dec_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        let mut value = bus.read(address);
        value -= 1;
        bus.write(address, value);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        5
    }

    fn dec_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_x_addr(bus);
        let mut value = bus.read(address);
        value -= 1;
        bus.write(address, value);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        6
    }

    fn dex_implied(&mut self, bus: &mut Bus) -> usize {
        self.x -= 1;
        self.update_nz_flags(bus, self.x);
        self.pc += 1;

        2
    }

    fn dey_implied(&mut self, bus: &mut Bus) -> usize {
        self.y -= 1;
        self.update_nz_flags(bus, self.y);
        self.pc += 1;

        2
    }

    fn dec_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        let mut value = bus.read(address);
        value -= 1;
        bus.write(address, value);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        6
    }

    fn dec_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_x_addr(bus);
        let mut value = bus.read(address);
        value -= 1;
        bus.write(address, value);
        self.update_nz_flags(bus, value);
        self.pc += 1;

        7
    }


    // LDA opcodes _________________________________________________________________________________
    fn lda_immediate(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;
        self.a = bus.read(self.pc);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        2
    }

    fn lda_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        self.a = bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        3
    }



    fn lda_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address2 = self.get_zero_page_x_addr(bus);
        self.a = bus.read(address2);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        4
    }

    fn lda_absolute(&mut self, bus: &mut Bus) -> usize {
        let absolute_address = self.get_absolute_addr(bus);
        self.a = bus.read(absolute_address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        4
    }


    fn lda_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, absolute_address) = self.get_absolute_x_addr(bus);
        self.a = bus.read(absolute_address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        // CHecks if a page boundary was crossed to determine number of cycles
        self.check_if_page_crossed45(bus, base_address, absolute_address)
    }

    fn lda_absolute_y(&mut self, bus: &mut Bus) -> usize {
        let (base_address, absolute_address) =  self.get_absolute_y_addr(bus);
        self.a = bus.read(absolute_address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        // Checks if a page boundary was crossed to determine number of cycles
        self.check_if_page_crossed45(bus, base_address, absolute_address)
    }


    fn lda_indirect_x(&mut self, bus: &mut Bus) -> usize {
        let pointer = self.get_indirect_x_addr(bus);
        self.a = bus.read(pointer);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        6
    }

    fn lda_indirect_y(&mut self, bus: &mut Bus) -> usize {
        let (base_address, final_address) = self.get_indirect_y_addr(bus);
        self.a = bus.read(final_address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        self.check_if_page_crossed56(bus, base_address, final_address)
    }



    //STA opcodes _________________________________________________________________________________
    fn sta_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        bus.write(address, self.a);
        self.pc += 1;

        3
    }

    fn sta_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let xaddress = self.get_zero_page_x_addr(bus);
        bus.write(xaddress, self.a);
        self.pc += 1;

        4
    }

    fn sta_absolute(&mut self, bus: &mut Bus) -> usize {
        let absolute_address = self.get_absolute_addr(bus);
        bus.write(absolute_address, self.a);
        self.pc += 1;

        4
    }

    fn sta_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let final_address = self.get_absolute_x_addr(bus);
        bus.write(final_address.1, self.a);
        self.pc += 1;

        5
    }

    fn sta_absolute_y(&mut self, bus: &mut Bus) -> usize {
        let final_address = self.get_absolute_y_addr(bus);
        bus.write(final_address.1, self.a);
        self.pc += 1;

        5
    }
    fn sta_indirect_x(&mut self, bus: &mut Bus) -> usize {
        let pointer = self.get_indirect_x_addr(bus);
        bus.write(pointer, self.a);
        self.pc += 1;

        6
    }

    fn sta_indirect_y(&mut self, bus: &mut Bus) -> usize {
        let final_address = self.get_indirect_y_addr(bus);
        bus.write(final_address.1, self.a);
        self.pc += 1;

        6
    }

    // STX (store x) opcodes _______________________________________________________________________
    fn stx_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        bus.write(address, self.x);
        self.pc += 1;

        3
    }

    fn stx_zero_page_y(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_y_addr(bus);
        bus.write(address, self.x);
        self.pc += 1;

        4
    }

    fn stx_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        bus.write(address, self.x);
        self.pc += 1;

        5
    }

    // STY (store y) opcodes _______________________________________________________________________
    fn sty_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        bus.write(address, self.y);
        self.pc += 1;

        3
    }

    fn sty_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_x_addr(bus);
        bus.write(address, self.y);
        self.pc += 1;

        4
    }

    fn sty_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        bus.write(address, self.y);
        self.pc += 1;

        5
    }

    // LDX opcodes ________________________________________________________________________________
    fn ldx_immediate(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;
        self.x = bus.read(self.pc);
        self.pc += 1;
        self.update_nz_flags(bus, self.x);

        2
    }

    fn ldx_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        self.x = bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.x);

        3
    }

    fn ldx_zero_page_y(&mut self, bus: &mut Bus) -> usize {
        let address2 = self.get_zero_page_y_addr(bus);
        self.x = bus.read(address2);
        self.pc += 1;
        self.update_nz_flags(bus, self.x);

        4
    }

    fn ldx_absolute(&mut self, bus: &mut Bus) -> usize {
        let absolute_address = self.get_absolute_addr(bus);
        self.x = bus.read(absolute_address);
        self.pc += 1;
        self.update_nz_flags(bus, self.x);

        4
    }

    fn ldx_absolute_y(&mut self, bus: &mut Bus) -> usize {
        let (base_address, absolute_address) = self.get_absolute_y_addr(bus);
        self.x = bus.read(absolute_address);
        self.pc += 1;
        self.update_nz_flags(bus, self.x);

        self.check_if_page_crossed45(bus, base_address, absolute_address)
    }

    // LDY (load y) opcodes ________________________________________________________________________
    fn ldy_immediate(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;
        self.y = bus.read(self.pc);
        self.pc += 1;
        self.update_nz_flags(bus, self.y);

        2
    }

    fn ldy_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        self.y = bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.y);

        3
    }

    fn ldy_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address2 = self.get_zero_page_x_addr(bus);
        self.y = bus.read(address2);
        self.pc += 1;
        self.update_nz_flags(bus, self.y);

        4
    }

    fn ldy_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        self.y = bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.y);

        4
    }

    fn ldy_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_x_addr(bus);
        self.y = bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.y);

        // Checks if a page boundary was crossed to determine number of cycles
        if  (base_address & 0xFF00) != (address & 0xFF00) {
            5
        } else {
            4
        }
    }
    // Transfer opcodes ____________________________________________________________________________

    fn tax_implied(&mut self, bus: &mut Bus) -> usize {
        self.x = self.a;
        self.pc += 1;
        self.update_nz_flags(bus, self.x);

        2
    }

    fn tay_implied(&mut self, bus: &mut Bus) -> usize {
        self.y = self.a;
        self.pc += 1;
        self.update_nz_flags(bus, self.y);

        2
    }

    fn tsx_implied(&mut self, bus: &mut Bus) -> usize {
        self.x = self.sp;
        self.pc += 1;
        self.update_nz_flags(bus, self.x);

        2
    }

    fn txa_implied(&mut self, bus: &mut Bus) -> usize {
        self.a = self.x;
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        2
    }

    fn txs_implied(&mut self, bus: &mut Bus) -> usize {
        self.sp = self.x;
        self.pc += 1;

        2
    }

    fn tya_implied(&mut self, bus: &mut Bus) -> usize {
        self.a = self.y;
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        2
    }
    // Jumps opcodes _______________________________________________________________________________

    fn jmp_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        self.pc = address;

        3
    }

    fn jmp_indirect(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;
        let pointer_low_byte = bus.read(self.pc) as u16;
        self.pc += 1;
        let pointer_high_byte = bus.read(self.pc) as u16;
        let pointer = (pointer_high_byte << 8) | pointer_low_byte;
        let low_byte = bus.read(pointer) as u16;
        let high_byte = bus.read(pointer.wrapping_add(1)) as u16;
        self.pc = (high_byte << 8) | low_byte;

        5
    }

    fn jsr_absolute(&mut self, bus: &mut Bus) -> usize {
        let return_address = self.pc + 2;
        // Pushes high byte
        self.stack_push(bus, ((return_address - 1) >> 8) as u8);
        // Pushes low byte
        self.stack_push(bus, (return_address - 1) as u8);
        self.pc += 1;
        let low_byte = bus.read(self.pc) as u16;
        self.pc += 1;
        let high_byte = bus.read(self.pc) as u16;
        self.pc = (high_byte << 8) | low_byte;

        6
    }

    fn rts_implied(&mut self, bus: &mut Bus) -> usize {
        let low_byte = self.stack_pop(bus) as u16;
        let high_byte = self.stack_pop(bus) as u16;
        let return_address = (high_byte << 8) | low_byte;
        self.pc = return_address + 1;

        6
    }

    fn brk_implied(&mut self, bus: &mut Bus) -> usize {
        self.pc += 2;
        let low_byte_pc = self.pc as u8;
        let high_byte_pc = (self.pc >> 8) as u8;
        self.stack_push(bus, high_byte_pc);
        self.stack_push(bus, low_byte_pc);
        let flags_with_b = self.p | cpu::BREAK_FLAG;
        self.stack_push(bus,flags_with_b);
        self.p.set_flag(cpu::INTERRUPT_DISABLE_FLAG, true);
        let low_byte_jump = bus.read(0xFFFE) as u16;
        let high_byte_jump = bus.read(0xFFFF) as u16;
        let jump_address = (high_byte_jump << 8) | low_byte_jump;
        self.pc = jump_address;

        7
    }

    fn rti_implied(&mut self, bus: &mut Bus) -> usize {
        let flags = self.stack_pop(bus);
        self.p = flags;
        let pc_low_byte = self.stack_pop(bus) as u16;
        let pc_high_byte = self.stack_pop(bus) as u16;
        self.pc = (pc_high_byte << 8) | pc_low_byte;

        6
    }

    //Branching opcodes ____________________________________________________________________________
    fn beq_relative(&mut self, bus: &mut Bus) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = bus.read(self.pc) as i8;
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
    fn bne_relative(&mut self, bus: &mut Bus) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = bus.read(self.pc) as i8;
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

    fn bcs_relative(&mut self, bus: &mut Bus) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = bus.read(self.pc) as i8;
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

    fn bcc_relative(&mut self, bus: &mut Bus) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = bus.read(self.pc) as i8;
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

    fn bmi_relative(&mut self, bus: &mut Bus) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = bus.read(self.pc) as i8;
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
    fn bpl_relative(&mut self, bus: &mut Bus) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = bus.read(self.pc) as i8;
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

    fn bvs_relative(&mut self, bus: &mut Bus) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = bus.read(self.pc) as i8;
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

    fn bvc_relative(&mut self, bus: &mut Bus) -> usize {
        let initial_address = self.pc;
        self.pc += 1;
        let offset = bus.read(self.pc) as i8;
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

    fn cmp_immediate(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;
        let (result, borrow) = (self.a).overflowing_sub(bus.read(self.pc));
        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);

        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        2
    }

    fn cmp_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        let value = bus.read(address);
        let (result, borrow) = self.a.overflowing_sub(value);

        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);
        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        3
    }

    fn cmp_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        let value = bus.read(address);
        let (result, borrow) = self.a.overflowing_sub(value);

        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);
        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        4
    }

    fn cmp_absolute_y(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_y_addr(bus);
        let value = bus.read(address);
        let (result, borrow) = self.a.overflowing_sub(value);

        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);
        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        4
    }

    fn cmp_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_x_addr(bus);
        let value = bus.read(address);
        let (result, borrow) = self.a.overflowing_sub(value);

        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);
        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        4
    }


    fn cmp_indirect_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_indirect_x_addr(bus);
        let value = bus.read(address);
        let (result, borrow) = self.a.overflowing_sub(value);

        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);
        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        6
    }


    fn cmp_indirect_y(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_indirect_y_addr(bus);
        let value = bus.read(address);
        let (result, borrow) = self.a.overflowing_sub(value);

        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);
        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        self.check_if_page_crossed56(bus, base_address, address)
    }

    fn cpx_immediate(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;
        let (result, borrow) = self.x.overflowing_sub(bus.read(self.pc));
        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);

        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        2
    }

    fn cpx_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        let value = bus.read(address);
        let (result, borrow) = self.x.overflowing_sub(value);

        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);
        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        3
    }

    fn cpx_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        let value = bus.read(address);
        let (result, borrow) = self.x.overflowing_sub(value);

        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);
        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        4
    }

    fn cpy_immediate(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;
        let (result, borrow) = self.y.overflowing_sub(bus.read(self.pc));
        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);

        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        2
    }

    fn cpy_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        let value = bus.read(address);
        let (result, borrow) = self.y.overflowing_sub(value);

        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);
        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        3
    }

    fn cpy_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        let value = bus.read(address);
        let (result, borrow) = self.y.overflowing_sub(value);

        self.p &= !(cpu::ZERO_FLAG | cpu::NEGATIVE_FLAG | cpu::CARRY_FLAG);
        if result == 0 {self.p |= cpu::ZERO_FLAG};
        if result & 0x80 != 0 {self.p |= cpu::NEGATIVE_FLAG};
        if !borrow {self.p |= cpu::CARRY_FLAG};
        self.pc += 1;

        4
    }

    // Arithmatic opcodes __________________________________________________________________________

    fn adc_immediate(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;
        let value = bus.read(self.pc);
        let result = self.a as u16 + value as u16 + (self.p & cpu::CARRY_FLAG) as u16;

        self.p.set_flag(cpu::CARRY_FLAG, result > 255);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;

        2
    }

    fn adc_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        let value = bus.read(address);
        let result = self.a as u16 + value as u16 + (self.p & cpu::CARRY_FLAG) as u16;

        self.p.set_flag(cpu::CARRY_FLAG, result > 255);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;

        3
    }

    fn adc_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_x_addr(bus);
        let value = bus.read(address);
        let result = self.a as u16 + value as u16 + (self.p & cpu::CARRY_FLAG) as u16;

        self.p.set_flag(cpu::CARRY_FLAG, result > 255);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;

        4
    }


    fn adc_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        let value = bus.read(address);
        let result = self.a as u16 + value as u16 + (self.p & cpu::CARRY_FLAG) as u16;

        self.p.set_flag(cpu::CARRY_FLAG, result > 255);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;

        4
    }
    fn adc_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, end_addr) = self.get_absolute_x_addr(bus);
        let value = bus.read(end_addr);
        let result = self.a as u16 + value as u16 + (self.p & cpu::CARRY_FLAG) as u16;

        self.p.set_flag(cpu::CARRY_FLAG, result > 255);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        self.check_if_page_crossed45(bus, base_address, end_addr)
    }

    fn adc_absolute_y(&mut self, bus: &mut Bus) -> usize {
        let (base_addr, end_addr) = self.get_absolute_y_addr(bus);
        let value = bus.read(end_addr);
        let result = self.a as u16 + value as u16 + (self.p & cpu::CARRY_FLAG) as u16;

        self.p.set_flag(cpu::CARRY_FLAG, result > 255);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        self.check_if_page_crossed45(bus, base_addr, end_addr)

    }

    fn adc_indirect_x(&mut self, bus: &mut Bus) -> usize {
        let end_addr = self.get_indirect_x_addr(bus);
        let value = bus.read(end_addr);
        let result = self.a as u16 + value as u16 + (self.p & cpu::CARRY_FLAG) as u16;

        self.p.set_flag(cpu::CARRY_FLAG, result > 255);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;

        6
    }

    fn adc_indirect_y(&mut self, bus: &mut Bus) -> usize {
        let (base_adress, end_addr) = self.get_indirect_y_addr(bus);
        let value = bus.read(end_addr);
        let result = self.a as u16 + value as u16 + (self.p & cpu::CARRY_FLAG) as u16;

        self.p.set_flag(cpu::CARRY_FLAG, result > 255);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign == val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        self.check_if_page_crossed56(bus, base_adress, end_addr)

    }



    // Subtract with Carry _________________________________________________________________________
    fn sbc_immediate(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;
        let value = bus.read(self.pc) as u16;
        let result = self.a as u16 - value - (1 - (self.p  & cpu::CARRY_FLAG) as u16);

        self.p.set_flag(cpu::CARRY_FLAG, result < 256);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;

        2
    }

    fn sbc_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        let value = bus.read(address) as u16;
        let result = self.a as u16 - value - (1 - (self.p  & cpu::CARRY_FLAG) as u16);

        self.p.set_flag(cpu::CARRY_FLAG, result < 256);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;

        3
    }

    fn sbc_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_x_addr(bus);
        let value = bus.read(address) as u16;
        let result = self.a as u16 - value - (1 - (self.p  & cpu::CARRY_FLAG) as u16);

        self.p.set_flag(cpu::CARRY_FLAG, result < 256);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;

        3
    }

    fn sbc_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        let value = bus.read(address) as u16;
        let result = self.a as u16 - value - (1 - (self.p  & cpu::CARRY_FLAG) as u16);

        self.p.set_flag(cpu::CARRY_FLAG, result < 256);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;

        4
    }

    fn sbc_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_addr, end_addr) = self.get_absolute_x_addr(bus);
        let value = bus.read(end_addr) as u16;
        let result = self.a as u16 - value - (1 - (self.p  & cpu::CARRY_FLAG) as u16);

        self.p.set_flag(cpu::CARRY_FLAG, result < 256);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        self.check_if_page_crossed45(bus, base_addr, end_addr)
    }

    fn sbc_absolute_y(&mut self, bus: &mut Bus) -> usize {
        let (base_addr, end_addr) = self.get_absolute_y_addr(bus);
        let value = bus.read(end_addr) as u16;
        let result = self.a as u16 - value - (1 - (self.p  & cpu::CARRY_FLAG) as u16);

        self.p.set_flag(cpu::CARRY_FLAG, result < 256);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;

        self.check_if_page_crossed45(bus, base_addr, end_addr)
    }

    fn sbc_indirect_x(&mut self, bus: &mut Bus) -> usize {
        let end_addr = self.get_indirect_x_addr(bus);
        let value = bus.read(end_addr) as u16;
        let result = self.a as u16 - value - (1 - (self.p & cpu::CARRY_FLAG) as u16);

        self.p.set_flag(cpu::CARRY_FLAG, result < 256);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;

        6
    }

    fn sbc_indirect_y(&mut self, bus: &mut Bus) -> usize {
        let (base_address, end_addr) = self.get_indirect_y_addr(bus);
        let value = bus.read(end_addr) as u16;
        let result = self.a as u16 - value - (1 - (self.p & cpu::CARRY_FLAG) as u16);

        self.p.set_flag(cpu::CARRY_FLAG, result < 256);
        self.p.set_flag(cpu::NEGATIVE_FLAG, (result & 0x80) != 0);
        self.p.set_flag(cpu::ZERO_FLAG, (result & 0xFF) == 0);

        let a_sign = self.a & 0x80;
        let val_sign = value as u8 & 0x80;
        let result_sign = result as u8 & 0x80;
        self.p.set_flag(cpu::OVERFLOW_FLAG, a_sign != val_sign && a_sign != result_sign);

        self.a = result as u8;
        self.pc += 1;
        self.check_if_page_crossed56(bus, base_address, end_addr)
    }

    // Logical operations __________________AND_____________________________________________________

    fn and_immediate(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;
        self.a = self.a & bus.read(self.pc);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        2
    }

    fn and_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        self.a = self.a & bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        3
    }

    fn and_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_x_addr(bus);
        self.a = self.a & bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        4
    }

    fn and_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        self.a = self.a & bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        4
    }
    fn and_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_x_addr(bus);
        self.a = self.a & bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        self.check_if_page_crossed45(bus, base_address, address)
    }

    fn and_absolute_y(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_y_addr(bus);
        self.a = self.a & bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        self.check_if_page_crossed45(bus, base_address, address)
    }

    fn and_indirect_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_indirect_x_addr(bus);
        self.a = self.a & bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        6
    }
    fn and_indirect_y(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_indirect_y_addr(bus);
        self.a = self.a & bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        self.check_if_page_crossed56(bus, base_address, address)
    }
    // ORA opcode __________________________________________________________________________________

    fn ora_immediate(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;
        self.a = self.a | bus.read(self.pc);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        2
    }

    fn ora_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        self.a = self.a | bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        3
    }

    fn ora_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_x_addr(bus);
        self.a = self.a | bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        4
    }

    fn ora_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        self.a = self.a | bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        4
    }

    fn ora_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_x_addr(bus);
        self.a = self.a | bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        self.check_if_page_crossed45(bus, base_address, address)
    }

    fn ora_absolute_y(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_y_addr(bus);
        self.a = self.a | bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        self.check_if_page_crossed45(bus, base_address, address)
    }

    fn ora_indirect_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_indirect_x_addr(bus);
        self.a = self.a | bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        6
    }

    fn ora_indirect_y(&mut self, bus: &mut Bus) -> usize {
        let (base_adress, address) = self.get_indirect_y_addr(bus);
        self.a = self.a | bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        self.check_if_page_crossed56(bus, base_adress, address)
    }
    //EOR opcodes __________________________________________________________________________________

    fn eor_immediate(&mut self, bus: &mut Bus) -> usize {
        self.pc += 1;
        self.a = self.a ^ bus.read(self.pc);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        2
    }

    fn eor_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        self.a = self.a ^ bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        3
    }

    fn eor_zero_page_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_x_addr(bus);
        self.a = self.a ^ bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        4
    }

    fn eor_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        self.a = self.a ^ bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        4
    }

    fn eor_absolute_x(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_x_addr(bus);
        self.a = self.a ^ bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        self.check_if_page_crossed45(bus, base_address, address)
    }

    fn eor_absolute_y(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_absolute_y_addr(bus);
        self.a = self.a ^ bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        self.check_if_page_crossed45(bus, base_address, address)
    }

    fn eor_indirect_x(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_indirect_x_addr(bus);
        self.a = self.a ^ bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        6
    }

    fn eor_indirect_y(&mut self, bus: &mut Bus) -> usize {
        let (base_address, address) = self.get_indirect_y_addr(bus);
        self.a = self.a ^ bus.read(address);
        self.pc += 1;
        self.update_nz_flags(bus, self.a);

        self.check_if_page_crossed56(bus, base_address, address)
    }

    // BIT opcodes _________________________________________________________________________________

    fn bit_zero_page(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_zero_page_addr(bus);
        let value = bus.read(address);
        let result = value & self.a;
        self.p.set_flag(cpu::ZERO_FLAG, result == 0);
        self.p.set_flag(cpu::OVERFLOW_FLAG, value & cpu::OVERFLOW_FLAG != 0);
        self.p.set_flag(cpu::NEGATIVE_FLAG, value & cpu::NEGATIVE_FLAG != 0);
        self.pc += 1;

        3
    }

    fn bit_absolute(&mut self, bus: &mut Bus) -> usize {
        let address = self.get_absolute_addr(bus);
        let value = bus.read(address);
        let result = value & self.a;
        self.p.set_flag(cpu::ZERO_FLAG, result == 0);
        self.p.set_flag(cpu::OVERFLOW_FLAG, value & cpu::OVERFLOW_FLAG != 0);
        self.p.set_flag(cpu::NEGATIVE_FLAG, value & cpu::NEGATIVE_FLAG != 0);
        self.pc += 1;

        4
    }
}








