use crate::bus;
use crate::bus::Bus;
use crate::cpu::Cpu6502;
use crate::ppu::Ppu;


struct System {
    cpu: Cpu6502,
    bus: Bus,
    system_clock: usize,

}
impl System {
    fn new() -> System {
        System {
            cpu: Cpu6502::init_cpu(),
            bus: bus::Bus::new(),
            system_clock: 0,
        }
    }


    pub fn step(mut self) {
        self.bus.ppu.step();

        let clock = self.system_clock;
        if clock % 3 == 0 {
            self.cpu.step(&mut self.bus);
        }

        if (self.bus.ppu.nmi) {

        }

    }
}