pub struct CpuClock {
    // the clock speed in Hz
    clock_speed: f64,

    // the cycle time of the processor
    // if PAL then the clock speed is 1.66MHz -> 602ns
    // if NTSC then the clock speed is 1.79MHz -> 558ns
    cycle_time_ns: u128,

    // cycles to wait before processing the next instruction
    wait_cycles: u8,
}

impl CpuClock {
    pub fn new(clock_speed: f64) -> Self {
        CpuClock {
            clock_speed,
            cycle_time_ns: (1000000000.0 / clock_speed) as u128,
            wait_cycles: 0,
        }
    }

    // push cpu cycles onto the clock
    pub fn push_cycles(&mut self, cycles: u8) {
        self.wait_cycles += cycles;
    }

    // clear the wait cycles and return the wait time in ns
    pub fn pop_cycles(&mut self) -> u128 {
        let wait_time = self.cycle_time_ns * (self.wait_cycles as u128);
        self.wait_cycles = 0;
        wait_time
    }
}
