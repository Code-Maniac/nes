mod addressing;
mod audio;
mod clocks;
mod memory;
mod opcodes;
mod processors;
mod video;

use std::cell::RefCell;
use std::rc::Rc;

pub fn start() {
    let cpu_clock = Rc::new(RefCell::new(clocks::CpuClock::new(1.0)));
    let mut cpu_mem = memory::CpuMemory::new(cpu_clock.clone());
    let mut cpu = processors::Ricoh2A03::new(cpu_clock.clone(), &mut cpu_mem);
}
