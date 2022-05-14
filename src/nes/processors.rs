use super::memory::Memory;

// address of registers within the status register
const STATUS_REG_ADDR_CARRY: u8 = 0;
const STATUS_REG_ADDR_ZERO: u8 = 1;
const STATUS_REG_ADDR_INTERRUPT: u8 = 2;
const STATUS_REG_ADDR_BREAK: u8 = 4;
const STATUS_REG_ADDR_OVERFLOW: u8 = 6;
const STATUS_REG_ADDR_NEGATIVE: u8 = 7;

pub trait Processor {
    fn process(&mut self);
    fn next_op_time_ns(&self) -> u128;
}

// enum Ricoh2A03Op {}

// The central processign unit
pub struct Ricoh2A03<'a> {
    // the cycle time of the processor
    // if PAL then the clock speed is 1.66MHz -> 602ns
    // if NTSC then the clock speed is 1.79MHz -> 558ns
    cycle_time_ns: u128,

    // cycles to wait before processing the next instruction
    wait_cycles: u8,

    // the program counter
    pc: u16,

    // the stack pointer
    sp: u8,

    // the status register
    // contains a number of single bit status'
    // 0 - Carry flag
    // 1 - Zero flag
    // 2 - Interrupt disable
    // 3 - Decimal mode (unused)
    // 4 - Break command
    // 5 -
    // 6 - Overflow flag
    // 7 - Negative flag
    sr: u8,

    // the accumulator register
    accum: u8,

    // index register x
    irx: u8,

    // index register y
    iry: u8,

    // the cpu memory map
    cpu_mem: &'a dyn Memory,
}

impl<'a> Ricoh2A03<'a> {
    pub fn new(clockspeed: f64, cpu_mem: &'a dyn Memory) -> Self {
        Ricoh2A03 {
            cycle_time_ns: (1000000000.0 / clockspeed) as u128,
            wait_cycles: 0,
            pc: 0,
            sp: 0,
            sr: 0,
            accum: 0,
            irx: 0,
            iry: 0,
            cpu_mem,
        }
    }

    // get and set the carry flag
    fn carry_flag(&self) -> bool {
        ((self.sr >> STATUS_REG_ADDR_CARRY) & 0x1) != 0
    }
    fn set_carry_flag(&mut self, val: bool) {
        self.sr |= (val as u8) << STATUS_REG_ADDR_CARRY;
    }

    // get and set the zero flag
    fn zero_flag(&self) -> bool {
        ((self.sr >> STATUS_REG_ADDR_ZERO) & 0x1) != 0
    }
    fn set_zero_flag(&mut self, val: bool) {
        self.sr |= (val as u8) << STATUS_REG_ADDR_ZERO;
    }

    // get and set the interrupt disable flag
    fn interrupt_disable(&self) -> bool {
        ((self.sr >> STATUS_REG_ADDR_INTERRUPT) & 0x1) != 0
    }
    fn set_interrupt_disable(&mut self, val: bool) {
        self.sr |= (val as u8) << STATUS_REG_ADDR_INTERRUPT;
    }

    // get and set break command
    fn break_command(&self) -> bool {
        ((self.sr >> STATUS_REG_ADDR_BREAK) & 0x1) != 0
    }
    fn set_break_command(&mut self, val: bool) {
        self.sr |= (val as u8) << STATUS_REG_ADDR_BREAK;
    }

    // get and set the overflow flag
    fn overflow_flag(&self) -> bool {
        ((self.sr >> STATUS_REG_ADDR_OVERFLOW) & 0x1) != 0
    }
    fn set_overflow_flag(&mut self, val: bool) {
        self.sr |= (val as u8) << STATUS_REG_ADDR_OVERFLOW;
    }

    // get and set the negative flag
    fn negative_flag(&self) -> bool {
        ((self.sr >> STATUS_REG_ADDR_NEGATIVE) & 0x1) != 0
    }
    fn set_negative_flag(&mut self, val: bool) {
        self.sr |= (val as u8) << STATUS_REG_ADDR_NEGATIVE;
    }

    // INSTRUCTION PROCESSING
    // All instruction are implemented below
    // Each instruction will have to implement variants with different memory
    // addressing modes. The supported addressing modes along with the
    // shorthand and postfix for the instruction functions are as follows:
    // NAME | SHORTHAND | POSTFIX
    // Accumulator | A | _acc
    // Implied | i | _imp
    // Immediate | # | _imm
    // Absolute | a | _abs
    // Zero Page | zp | _zp
    // Relative | r | _r
    // Absolute Indirect | (a) | _abs_ind
    // Absolute Indexed with X | a,x | _abs_idx_x
    // Absolute Indexed with Y | a,y | _abs_idx_y
    // Zero Page Indexed with X | zp,x | _zp_idx_x
    // Zero Page Indexed with Y | zp,y | _zp_idx_y
    // Zero Page Indexed Indirect | (zp,x) | _zp_idx_ind
    // Zero Page Indirect Indexed with Y | (zp),y | _zp_idx_ind_y
    //
    // After instruction is processed we then have to wait an ammount of time
    // to allow the process to finish in a time that it would take on the real
    // hardware. Each instruction will take a different number of discrete cpu
    // cycles with the maximum number of cycles for any instruction being 7.
    // Some instructions may take 1 extra than normal if a page boundary is
    // crossed

    // LDA -> Load Accumulator with Memory
    // Affects flags: N,Z
    fn lda_acc(&mut self) {}
    fn lda_i(&mut self) {}
    fn lda_imm(&mut self) {}
    fn lda_a(&mut self) {}
    fn lda_zp(&mut self) {}
    fn lda_r(&mut self) {}
    fn lda_abs_ind(&mut self) {}
    fn lda_abs_idx_x(&mut self) {}
    fn lda_abs_idx_y(&mut self) {}
    fn lda_abs_zp_idx_x(&mut self) {}
    fn lda_abs_zp_idx_y(&mut self) {}
    fn lda_abs_zp_idx_ind(&mut self) {}
    fn lda_abs_zp_idx_ind_y(&mut self) {}

    // LDX -> Load Index X with Memory
    // Affects flags: N,Z
    fn ldx_imm(&mut self) {}
    fn ldx_abs(&mut self) {}
    fn ldx_zp(&mut self) {}
    fn ldx_abs_idx_y(&mut self) {}
    fn ldx_zp_idx_y(&mut self) {}

    // LDY -> Load Index Y with Memory
    // Affects flags: N,Z
    fn ldy_imm(&mut self) {}
    fn ldy_abs(&mut self) {}
    fn ldy_zp(&mut self) {}
    fn ldy_abs_idx_x(&mut self) {}
    fn ldy_zp_idx_x(&mut self) {}

    // STA -> Store Accumulator in Memory
    fn sta_abs(&mut self) {}
    fn sta_zp(&mut self) {}
    fn sta_abs_x(&mut self) {}
    fn sta_abs_y(&mut self) {}
    fn sta_zp_x(&mut self) {}
    fn sta_zp_idx_ind(&mut self) {}
    fn sta_zp_idx_ind_y(&mut self) {}

    // STX -> Store X Register in Memory
    fn stx_abs(&mut self) {}
    fn stx_zp(&mut self) {}
    fn stx_zp_y(&mut self) {}

    // STY -> Store Y Register in Memory
    fn sty_abs(&mut self) {}
    fn sty_zp(&mut self) {}
    fn sty_zp_x(&mut self) {}

    // NOTE: TASKS
    // 1. Write unit tests for memory instruction functions
    // 2. Write the memory instruction functions until unit tests pass
    // 3. Implement additional instructions following the above process
}

impl Processor for Ricoh2A03<'_> {
    fn process(&mut self) {
        // read the next instruction and process it
    }

    fn next_op_time_ns(&self) -> u128 {
        0u128
    }
}

pub struct Ricoh2C02 {}

impl Ricoh2C02 {}

impl Processor for Ricoh2C02 {
    fn process(&mut self) {}

    fn next_op_time_ns(&self) -> u128 {
        0u128
    }
}

// all tests go here
// #[cfg(test)]
// mod tests {
//     #[test]
//     fn
// }
