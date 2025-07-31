use crate::ppu::Ppu;

pub trait FlagRegister {
    fn set_flag(&mut self, flag:u8, condition: bool);
    fn check_flag(&self, flag:u8) -> bool;
}

impl FlagRegister for u8 {
    fn set_flag(&mut self, flag: u8, condition: bool) {
        if condition {
            *self |= flag;
        } else {
            *self &= !flag;
        }
    }
    
    fn check_flag(&self, flag:u8) -> bool {
        (*self & flag) == flag
    }
}


pub mod ppu {
    // Status Flags
    pub(crate) const VBLANK_FLAG: u8 = 0b10000000;
    pub(crate) const SPRITE_0_HIT: u8 = 0b01000000;
    pub(crate) const SPRITE_OVERFLOW: u8 = 0b00100000;

    // Control Flags
    pub(crate) const V_BLANK_NMI_ENABLE: u8 = 0b10000000;
    pub(crate) const PPU_MASTER_SLAVE: u8 = 0b01000000;
}

pub mod cpu {
    pub(crate) const CARRY_FLAG:u8 = 0b00000001;
    pub(crate) const ZERO_FLAG:u8 = 0b00000010;
    pub(crate) const INTERRUPT_DISABLE_FLAG:u8 = 0b00000100;
    pub(crate) const DECIMAL_FLAG:u8 = 0b00001000;
    pub(crate) const OVERFLOW_FLAG:u8 = 0b01000000;
    pub(crate) const NEGATIVE_FLAG:u8 = 0b10000000;
    pub(crate) const BREAK_FLAG:u8 = 0b00010000;
}