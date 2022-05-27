use super::memory::CpuMemory;
use super::memory::CPU_MEM_PAGE_SIZE;
use super::opcodes::*;
use std::num::Wrapping;

// address of registers within the status register
const STATUS_REG_ADDR_CARRY: u8 = 0;
const STATUS_REG_ADDR_ZERO: u8 = 1;
const STATUS_REG_ADDR_INTERRUPT: u8 = 2;
const STATUS_REG_ADDR_DECIMAL: u8 = 3;
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
    // 3 - Decimal mode
    // 4 - Break command
    // 5 -
    // 6 - Overflow flag
    // 7 - Negative flag
    sr: u8,

    // the accumulator register
    acc: u8,

    // index register x
    irx: u8,

    // index register y
    iry: u8,

    // the cpu memory map
    cpu_mem: &'a mut CpuMemory,

    // the number of cpu cycles that the current opcode has taken
    opcode_cycles: u8,
}

impl<'a> Ricoh2A03<'a> {
    pub fn new(clockspeed: f64, cpu_mem: &'a mut CpuMemory) -> Self {
        Ricoh2A03 {
            cycle_time_ns: (1000000000.0 / clockspeed) as u128,
            wait_cycles: 0,
            pc: 0,
            sp: 0xff,
            sr: 0,
            acc: 0,
            irx: 0,
            iry: 0,
            cpu_mem,
            opcode_cycles: 0,
        }
    }

    // get the page of the program counter (used to check crossing of page
    // boundaries during branches)
    fn pc_page(&self) -> u16 {
        self.pc % CPU_MEM_PAGE_SIZE as u16
    }

    // NOTE: cycles code should be moved to CpuClock
    fn push_cycles(&mut self, num: u8) {
        // happy to let a panic occur here as no single cpu opcode can take more
        // than around 10 cycles
        self.opcode_cycles += num;
    }

    pub fn pop_cycles(&mut self) -> u8 {
        let output = self.opcode_cycles;
        self.opcode_cycles = 0;
        output
    }

    // push a value onto the stack
    // stack works top down so we decrement stack pointer
    fn push_stack(&mut self, val: u8) {
        // write to the current stack pointer location then decrement
        self.cpu_mem.write_stack(self.sp, val);
        self.sp = self.sp.wrapping_sub(1);
    }

    // pull a value from the stack
    // stack works top down so we increment the stack pointer
    fn pop_stack(&mut self) -> u8 {
        // increment stack pointer then read from current location
        self.sp = self.sp.wrapping_add(1);
        self.cpu_mem.read_stack(self.sp)
    }

    fn push_pc(&mut self, offset: u16) {
        let pc2 = self.pc + offset;
        let pcl = (pc2 & 0x00ff) as u8;
        let pch = ((pc2 & 0xff00) >> 8) as u8;

        // push low then high
        self.push_stack(pcl);
        self.push_stack(pch);
    }

    fn pop_pc(&mut self) {
        let pch = self.pop_stack();
        let pcl = self.pop_stack();

        self.pc = (pcl as u16) | ((pch as u16) << 8);
    }

    // advance the program counter by the given ammount
    fn adv_pc(&mut self, val: u16) {
        self.pc += val;
    }

    // adv program counter for different addressing
    fn adv_pc_zpg(&mut self) {
        self.adv_pc(2);
    }

    fn adv_pc_zpg_x(&mut self) {
        self.adv_pc_zpg();
    }

    fn adv_pc_zpg_y(&mut self) {
        self.adv_pc_zpg();
    }

    fn adv_pc_abs(&mut self) {
        self.adv_pc(3);
    }

    fn adv_pc_abs_x(&mut self) {
        self.adv_pc_abs();
    }

    fn adv_pc_abs_y(&mut self) {
        self.adv_pc_abs();
    }

    fn adv_pc_imm(&mut self) {
        self.adv_pc_zpg();
    }

    fn adv_pc_ind_x(&mut self) {
        self.adv_pc_zpg();
    }

    fn adv_pc_ind_y(&mut self) {
        self.adv_pc_zpg();
    }

    fn adv_pc_imp(&mut self) {
        self.adv_pc(1);
    }

    // get and set the carry flag
    fn carry_flag(&self) -> bool {
        ((self.sr >> STATUS_REG_ADDR_CARRY) & 0x1) != 0
    }
    fn set_carry_flag(&mut self, val: bool) {
        self.sr |= (val as u8) << STATUS_REG_ADDR_CARRY;
    }

    // get the arithmetic value of the carry
    fn carry_val(&self) -> u8 {
        self.carry_flag() as u8
    }

    // get the arithmetic value for borrow
    fn borrow_val(&self) -> u8 {
        (!self.carry_flag()) as u8
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

    // get and set the decimal flag
    fn decimal_flag(&self) -> bool {
        ((self.sr >> STATUS_REG_ADDR_DECIMAL) & 0x1) != 0
    }
    fn set_decimal_flag(&mut self, val: bool) {
        self.sr |= (val as u8) << STATUS_REG_ADDR_DECIMAL;
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

    // set the negative and zero flags based on the value
    fn set_nz_flags(&mut self, val: u8) {
        self.set_zero_flag(val == 0);
        self.set_negative_flag(val & 0x80 > 0);
    }

    // PC and register dependant read operations
    fn read_opcode(&self) -> u8 {
        self.cpu_mem.read_opcode(self.pc)
    }

    fn read_zpg(&self) -> u8 {
        let zpg_addr = self.cpu_mem.read_opcode_aa(self.pc);
        self.cpu_mem.read_zpg(zpg_addr)
    }

    fn write_zpg(&mut self, val: u8) {
        let zpg_addr = self.cpu_mem.read_opcode_aa(self.pc);
        self.cpu_mem.write_zpg(zpg_addr, val);
    }

    fn read_zpg_x(&self) -> u8 {
        let zpg_addr = self.cpu_mem.read_opcode_aa(self.pc);
        self.cpu_mem.read_zpg_x(zpg_addr, self.irx)
    }

    fn write_zpg_x(&mut self, val: u8) {
        let zpg_addr = self.cpu_mem.read_opcode_aa(self.pc);
        self.cpu_mem.write_zpg_x(zpg_addr, self.irx, val)
    }

    fn read_zpg_y(&self) -> u8 {
        let zpg_addr = self.cpu_mem.read_opcode_aa(self.pc);
        self.cpu_mem.read_zpg_x(zpg_addr, self.iry)
    }

    fn write_zpg_y(&mut self, val: u8) {
        let zpg_addr = self.cpu_mem.read_opcode_aa(self.pc);
        self.cpu_mem.write_zpg_x(zpg_addr, self.irx, val)
    }

    fn read_abs(&self) -> u8 {
        let aa = self.cpu_mem.read_opcode_aa(self.pc);
        let bb = self.cpu_mem.read_opcode_bb(self.pc);
        self.cpu_mem.read_abs(aa, bb)
    }

    fn write_abs(&mut self, val: u8) {
        let aa = self.cpu_mem.read_opcode_aa(self.pc);
        let bb = self.cpu_mem.read_opcode_bb(self.pc);
        self.cpu_mem.write_abs(aa, bb, val);
    }

    fn read_abs_x(&self) -> u8 {
        let aa = self.cpu_mem.read_opcode_aa(self.pc);
        let bb = self.cpu_mem.read_opcode_bb(self.pc);
        self.cpu_mem.read_abs_x(aa, bb, self.irx)
    }

    fn write_abs_x(&mut self, val: u8, free: bool) {
        let aa = self.cpu_mem.read_opcode_aa(self.pc);
        let bb = self.cpu_mem.read_opcode_bb(self.pc);

        if free {
            self.cpu_mem.write_abs_x_free(aa, bb, self.irx, val);
        } else {
            self.cpu_mem.write_abs_x(aa, bb, self.irx, val);
        }
    }

    fn read_abs_y(&self) -> u8 {
        let aa = self.cpu_mem.read_opcode_aa(self.pc);
        let bb = self.cpu_mem.read_opcode_bb(self.pc);
        self.cpu_mem.read_abs_x(aa, bb, self.iry)
    }

    fn write_abs_y(&mut self, val: u8, free: bool) {
        let aa = self.cpu_mem.read_opcode_aa(self.pc);
        let bb = self.cpu_mem.read_opcode_bb(self.pc);

        if free {
            self.cpu_mem.write_abs_x_free(aa, bb, self.iry, val);
        } else {
            self.cpu_mem.write_abs_x(aa, bb, self.iry, val);
        }
    }

    fn read_ind_x(&self) -> u8 {
        let aa = self.cpu_mem.read_opcode_aa(self.pc);
        self.cpu_mem.read_ind_x(aa, self.irx)
    }

    fn write_ind_x(&mut self, val: u8) {
        let aa = self.cpu_mem.read_opcode_aa(self.pc);
        self.cpu_mem.write_ind_x(aa, self.irx, val);
    }

    fn read_ind_y(&self) -> u8 {
        let aa = self.cpu_mem.read_opcode_aa(self.pc);
        self.cpu_mem.read_ind_y(aa, self.iry)
    }

    fn write_ind_y(&mut self, val: u8, free: bool) {
        let aa = self.cpu_mem.read_opcode_aa(self.pc);
        if free {
            self.cpu_mem.write_ind_y_free(aa, self.iry, val);
        } else {
            self.cpu_mem.write_ind_y(aa, self.iry, val);
        }
    }

    fn read_imm(&self) -> u8 {
        self.cpu_mem.read_opcode_aa(self.pc)
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

    fn process_opcode(&mut self) {
        match self.read_opcode() {
            BRK => {
                self.brk();
            }
            ORA_IND_X => {
                self.ora_ind_x();
            }
            ORA_ZPG => {
                self.ora_zpg();
            }
            ASL_ZPG => {
                self.asl_zpg();
            }
            PHP => {
                self.php();
            }
            ORA_IMM => {
                self.ora_imm();
            }
            ASL_A => {
                self.asl_a();
            }
            ORA_ABS => {
                self.ora_abs();
            }
            ASL_ABS => {
                self.asl_abs();
            }
            BPL => {
                self.bpl();
            }
            ORA_IND_Y => {
                self.ora_ind_y();
            }
            ORA_ZPG_X => {
                self.ora_zpg_x();
            }
            ASL_ZPG_X => {
                self.asl_zpg_x();
            }
            CLC => {
                self.clc();
            }
            ORA_ABS_Y => {
                self.ora_abs_y();
            }
            ORA_ABS_X => {
                self.ora_abs_x();
            }
            ASL_ABS_X => {
                self.asl_abs_x();
            }
            JSR_ABS => {
                self.jsr_abs();
            }
            AND_IND_X => {
                self.and_ind_x();
            }
            BIT_ZPG => {
                self.bit_zpg();
            }
            AND_ZPG => {
                self.and_zpg();
            }
            ROL_ZPG => {
                self.rol_zpg();
            }
            PLP => {
                self.plp();
            }
            AND_IMM => {
                self.and_imm();
            }
            ROL_A => {
                self.rol_a();
            }
            BIT_ABS => {
                self.bit_abs();
            }
            AND_ABS => {
                self.and_abs();
            }
            ROL_ABS => {
                self.rol_abs();
            }
            BMI => {
                self.bmi();
            }
            AND_IND_Y => {
                self.and_ind_y();
            }
            AND_ZPG_X => {
                self.and_zpg_x();
            }
            ROL_ZPG_X => {
                self.rol_zpg_x();
            }
            SEC => {
                self.sec();
            }
            AND_ABS_Y => {
                self.and_abs_y();
            }
            AND_ABS_X => {
                self.and_abs_x();
            }
            ROL_ABS_X => {
                self.rol_abs_x();
            }
            RTI => {
                self.rti();
            }
            EOR_IND_X => {
                self.eor_ind_x();
            }
            EOR_ZPG => {
                self.eor_zpg();
            }
            LSR_ZPG => {
                self.lsr_zpg();
            }
            PHA => {
                self.pha();
            }
            EOR_IMM => {
                self.eor_imm();
            }
            LSR_A => {
                self.lsr_a();
            }
            JMP_ABS => {
                self.jmp_abs();
            }
            EOR_ABS => {
                self.eor_abs();
            }
            LSR_ABS => {
                self.lsr_abs();
            }
            BVC => {
                self.bvc();
            }
            EOR_IND_Y => {
                self.eor_ind_y();
            }
            EOR_ZPG_X => {
                self.eor_zpg_x();
            }
            LSR_ZPG_X => {
                self.lsr_zpg_x();
            }
            CLI => {
                self.cli();
            }
            EOR_ABS_Y => {
                self.eor_abs_y();
            }
            EOR_ABS_X => {
                self.eor_abs_x();
            }
            LSR_ABS_X => {
                self.lsr_abs_x();
            }
            RTS => {
                self.rts();
            }
            ADC_IND_X => {
                self.adc_ind_x();
            }
            ADC_ZPG => {
                self.adc_zpg();
            }
            ROR_ZPG => {
                self.ror_zpg();
            }
            PLA => {
                self.pla();
            }
            ADC_IMM => {
                self.adc_imm();
            }
            ROR_A => {
                self.ror_a();
            }
            JMP_IND => {
                self.jmp_ind();
            }
            ADC_ABS => {
                self.adc_abs();
            }
            ROR_ABS => {
                self.ror_abs();
            }
            BVS => {
                self.bvs();
            }
            ADC_IND_Y => {
                self.adc_ind_y();
            }
            ADC_ZPG_X => {
                self.adc_zpg_x();
            }
            ROR_ZPG_X => {
                self.ror_zpg_x();
            }
            SEI => {
                self.sei();
            }
            ADC_ABS_Y => {
                self.adc_abs_y();
            }
            ADC_ABS_X => {
                self.adc_abs_x();
            }
            ROR_ABS_X => {
                self.ror_abs_x();
            }
            STA_IND_X => {
                self.sta_ind_x();
            }
            STY_ZPG => {
                self.sty_zpg();
            }
            STA_ZPG => {
                self.sta_zpg();
            }
            STX_ZPG => {
                self.stx_zpg();
            }
            DEY => {
                self.dey();
            }
            TXA => {
                self.txa();
            }
            STY_ABS => {
                self.sty_abs();
            }
            STA_ABS => {
                self.sta_abs();
            }
            STX_ABS => {
                self.stx_abs();
            }
            BCC => {
                self.bcc();
            }
            STA_IND_Y => {
                self.sta_ind_y();
            }
            STY_ZPG_X => {
                self.sty_zpg_x();
            }
            STA_ZPG_X => {
                self.sta_zpg_x();
            }
            STX_ZPG_Y => {
                self.stx_zpg_y();
            }
            TYA => {
                self.tya();
            }
            STA_ABS_Y => {
                self.sta_abs_y();
            }
            TXS => {
                self.txs();
            }
            STA_ABS_X => {
                self.sta_abs_x();
            }
            LDY_IMM => {
                self.ldy_imm();
            }
            LDA_IND_X => {
                self.lda_ind_x();
            }
            LDX_IMM => {
                self.ldx_imm();
            }
            LDY_ZPG => {
                self.ldy_zpg();
            }
            LDA_ZPG => {
                self.lda_zpg();
            }
            LDX_ZPG => {
                self.ldx_zpg();
            }
            TAY => {
                self.tay();
            }
            LDA_IMM => {
                self.lda_imm();
            }
            TAX => {
                self.tax();
            }
            LDY_ABS => {
                self.ldy_abs();
            }
            LDA_ABS => {
                self.lda_abs();
            }
            LDX_ABS => {
                self.ldx_abs();
            }
            BCS => {
                self.bcs();
            }
            LDA_IND_Y => {
                self.lda_ind_y();
            }
            LDY_ZPG_X => {
                self.ldy_zpg_x();
            }
            LDA_ZPG_X => {
                self.lda_zpg_x();
            }
            LDX_ZPG_Y => {
                self.ldx_zpg_y();
            }
            CLV => {
                self.clv();
            }
            LDA_ABS_Y => {
                self.lda_abs_y();
            }
            TSX => {
                self.tsx();
            }
            LDY_ABS_X => {
                self.ldy_abs_x();
            }
            LDA_ABS_X => {
                self.lda_abs_x();
            }
            LDX_ABS_Y => {
                self.ldx_abs_y();
            }
            CPY_IMM => {
                self.cpy_imm();
            }
            CMP_IND_X => {
                self.cmp_ind_x();
            }
            CPY_ZPG => {
                self.cpy_zpg();
            }
            CMP_ZPG => {
                self.cmp_zpg();
            }
            DEC_ZPG => {
                self.dec_zpg();
            }
            INY => {
                self.iny();
            }
            CMP_IMM => {
                self.cmp_imm();
            }
            DEX => {
                self.dex();
            }
            CPY_ABS => {
                self.cpy_abs();
            }
            CMP_ABS => {
                self.cmp_abs();
            }
            DEC_ABS => {
                self.dec_abs();
            }
            BNE => {
                self.bne();
            }
            CMP_IND_Y => {
                self.cmp_ind_y();
            }
            CMP_ZPG_X => {
                self.cmp_zpg_x();
            }
            DEC_ZPG_X => {
                self.dec_zpg_x();
            }
            CLD => {
                self.cld();
            }
            CMP_ABS_Y => {
                self.cmp_abs_y();
            }
            CMP_ABS_X => {
                self.cmp_abs_x();
            }
            DEC_ABS_X => {
                self.dec_abs_x();
            }
            CPX_IMM => {
                self.cpx_imm();
            }
            SBC_IND_X => {
                self.sbc_ind_x();
            }
            CPX_ZPG => {
                self.cpx_zpg();
            }
            SBC_ZPG => {
                self.sbc_zpg();
            }
            INC_ZPG => {
                self.inc_zpg();
            }
            INX => {
                self.inx();
            }
            SBC_IMM => {
                self.sbc_imm();
            }
            NOP => {
                self.nop();
            }
            CPX_ABS => {
                self.cpx_abs();
            }
            SBC_ABS => {
                self.sbc_abs();
            }
            INC_ABS => {
                self.inc_abs();
            }
            BEQ => {
                self.beq();
            }
            SBC_IND_Y => {
                self.sbc_ind_y();
            }
            SBC_ZPG_X => {
                self.sbc_zpg_x();
            }
            INC_ZPG_X => {
                self.inc_zpg_x();
            }
            SED => {
                self.sed();
            }
            SBC_ABS_Y => {
                self.sbc_abs_y();
            }
            SBC_ABS_X => {
                self.sbc_abs_x();
            }
            INC_ABS_X => {
                self.inc_abs_x();
            }
            _ => {
                panic!("Illegal opcode");
            }
        }
    }

    // addressing independant functions

    // ora affects the following registers:
    // N, Z
    fn ora(&mut self, mem_val: u8) {
        self.acc |= mem_val;
        self.set_nz_flags(self.acc);
    }

    // and affects the following registers:
    // N, Z
    fn and(&mut self, mem_val: u8) {
        self.acc |= mem_val;
        self.set_nz_flags(self.acc);
    }

    // xor affects the following registers:
    // N, Z
    fn eor(&mut self, mem_val: u8) {
        self.acc ^= mem_val;
        self.set_nz_flags(self.acc);
    }

    // asl affects the following registers:
    // N, Z, C
    // Shifted out bit is stored in carry flag
    // return the shifted value
    fn asl(&mut self, val: u8) -> u8 {
        let bit = val >> 7;
        let shift_val = val << 1;

        self.set_nz_flags(val);
        self.set_carry_flag(bit != 0);
        shift_val
    }

    // lsr affects the following registers
    // N, Z, C
    // Shifted out bit is stored in carry flag
    // return the shifted value
    fn lsr(&mut self, val: u8) -> u8 {
        let bit = val & 0x1;
        let shift_val = val >> 1;

        self.set_nz_flags(shift_val);
        self.set_carry_flag(bit != 0);
        shift_val
    }

    // rol affects the following registers
    // N, Z, C
    // Carry flag is shifted in on the right
    // Shifted out bit is stored in carry flag
    // return the shifted value
    fn rol(&mut self, val: u8) -> u8 {
        let bit = val >> 7;
        let rot_val = (val << 1) | self.carry_flag() as u8;

        self.set_nz_flags(rot_val);
        self.set_carry_flag(bit != 0);
        rot_val
    }

    // ror affects the following registers
    // N, Z, C
    // Carry flag is shifting in on the left
    // Shifted out bit is stored in carry flag
    // return the shifted value
    fn ror(&mut self, val: u8) -> u8 {
        let bit = val & 0x1;
        let rot_val = (val >> 1) | ((self.carry_flag() as u8) << 7);

        self.set_nz_flags(rot_val);
        self.set_carry_flag(bit != 0);
        rot_val
    }

    // lda affects the following registers:
    // N, Z
    fn lda(&mut self, mem_val: u8) {
        self.acc = mem_val;
        self.set_nz_flags(self.acc);
    }

    // ldx affects the following registers:
    // N, Z
    fn ldx(&mut self, mem_val: u8) {
        self.irx = mem_val;
        self.set_nz_flags(self.irx);
    }

    // ldy affects the following registers:
    // N, Z
    fn ldy(&mut self, mem_val: u8) {
        self.iry = mem_val;
        self.set_nz_flags(self.iry);
    }

    // adc affects the following registers:
    // N, Z, C, V
    fn adc(&mut self, mem_val: u8) {
        let mut output = self.acc;
        let mut carry = false;
        let mut carry_tmp: bool;

        // add the mem_val and check for carry
        (output, carry_tmp) = output.overflowing_add(mem_val);

        if carry_tmp {
            carry = true;
        }

        // add the carry_val and check for carry
        (output, carry_tmp) = output.overflowing_add(self.carry_val());

        if carry_tmp {
            carry = true;
        }

        self.acc = output;

        self.set_nz_flags(self.acc);
        self.set_carry_flag(!carry);
    }

    // sbc affects the following registers:
    // N, Z, C, V
    fn sbc(&mut self, mem_val: u8) {
        let mut output = self.acc;
        let mut borrow = true;
        let mut borrow_tmp: bool;

        // subtract the mem_val and check for borrow
        (output, borrow_tmp) = output.overflowing_sub(mem_val);

        if borrow_tmp {
            borrow = false;
        }

        // subtract the borrow val and check for borrow
        (output, borrow_tmp) = output.overflowing_sub(self.borrow_val());

        if borrow_tmp {
            borrow = false;
        }

        self.acc = output;

        self.set_nz_flags(self.acc);
        self.set_carry_flag(borrow);
    }

    // perform increment operation and return modified value
    fn inc_val(&mut self, val: u8) -> u8 {
        let mut wrap_val = Wrapping(val);
        wrap_val += Wrapping(1);
        self.set_nz_flags(wrap_val.0);
        wrap_val.0
    }

    // perform decrement operation and return modified value
    fn dec_val(&mut self, val: u8) -> u8 {
        let mut wrap_val = Wrapping(val);
        wrap_val -= Wrapping(1);
        self.set_nz_flags(wrap_val.0);
        wrap_val.0
    }

    // perform bit operation
    // set negative flag to bit 7 of val
    // set overflow flag to bit 6 of val
    // set zero flag to result of val AND accumulator
    fn bit(&mut self, val: u8) {
        self.set_negative_flag((val >> 7) == 1);
        self.set_overflow_flag(((val >> 6) & 0x1) == 1);
        self.set_zero_flag(val & self.acc != 0);
    }

    // compare memory with accumulator
    // set N, Z, C flags
    fn cmp(&mut self, mem_val: u8) {
        self.compare(mem_val, self.acc);
    }

    // compare memory with x index reg
    // set N, Z, C flags
    fn cpx(&mut self, mem_val: u8) {
        self.compare(mem_val, self.irx);
    }

    // compare memory with y index reg
    // set N, Z, C flags
    fn cpy(&mut self, mem_val: u8) {
        self.compare(mem_val, self.iry);
    }

    // utility function for the above 3
    fn compare(&mut self, mem_val: u8, reg_val: u8) {
        let sign_bit = (reg_val.wrapping_sub(mem_val)) >> 7;

        let zcn: (bool, bool, bool);
        if reg_val < mem_val {
            zcn = (false, false, sign_bit == 1)
        } else if reg_val > mem_val {
            zcn = (false, true, sign_bit == 1)
        } else {
            zcn = (true, true, true);
        }

        // set the flags
        self.set_zero_flag(zcn.0);
        self.set_carry_flag(zcn.1);
        self.set_negative_flag(zcn.2);
    }

    // modify the program counter by the offset
    fn branch(&mut self, val: u8) {
        let pc_page = self.pc_page();
        if val >> 7 == 1 {
            // value is negative so convert
            let sub_val = ((val & 0x7F) ^ 0x7F) + 1;
            self.pc = self.pc.wrapping_sub(sub_val as u16);
        } else {
            // value is positive
            self.pc = self.pc.wrapping_add(val as u16);
        }

        // check that page boundary was crossed in prg memory
        // if it has then we add an additional 2 cycles
        // otherwise just 1 cycle
        if self.pc_page() != pc_page {
            // a page boundary was crossed
            self.push_cycles(2);
        } else {
            self.push_cycles(1);
        }
    }

    // check condition. if true then branch
    fn cond_branch(&mut self, cond: bool) {
        if cond {
            self.branch(self.read_imm())
        } else {
            self.adv_pc(2);
        }
        self.push_cycles(2); // always takes an additional 2 cycles
    }

    // jump to the location defined by the opcode args
    fn jump(&mut self) {
        // now read the new pcl and pch from the prg mem
        let pcl = self.cpu_mem.read_opcode_aa(self.pc);
        let pch = self.cpu_mem.read_opcode_bb(self.pc);

        self.pc = (pcl as u16) | ((pch as u16) << 8);
    }

    // HI 0
    fn brk(&mut self) {
        self.push_pc(2);
        self.set_break_command(true);
        self.push_stack(self.sr);
    }

    fn ora_ind_x(&mut self) {
        self.ora(self.read_ind_x());
        self.adv_pc_ind_x();
    }

    fn ora_zpg(&mut self) {
        self.ora(self.read_zpg());
        self.adv_pc_zpg();
    }

    fn asl_zpg(&mut self) {
        let val = self.asl(self.read_zpg());
        self.write_zpg(val);
        self.adv_pc_zpg();
    }

    fn php(&mut self) {
        self.set_break_command(true);
        self.set_interrupt_disable(true);
        self.push_stack(self.sr);
    }

    fn ora_imm(&mut self) {
        self.ora(self.read_imm());
        self.adv_pc_imm();
    }

    fn asl_a(&mut self) {
        self.acc = self.asl(self.acc);
        self.adv_pc_imp();
    }

    fn ora_abs(&mut self) {
        self.ora(self.read_abs());
        self.adv_pc_abs();
    }

    fn asl_abs(&mut self) {
        let val = self.asl(self.read_abs());
        self.write_abs(val);
        self.adv_pc_abs();
    }

    // HI 1
    fn bpl(&mut self) {
        self.cond_branch(!self.negative_flag());
    }

    fn ora_ind_y(&mut self) {
        self.ora(self.read_ind_y());
        self.adv_pc_ind_y();
    }

    fn ora_zpg_x(&mut self) {
        self.ora(self.read_zpg_x());
        self.adv_pc_zpg_x();
    }

    fn asl_zpg_x(&mut self) {
        let val = self.asl(self.read_zpg_x());
        self.write_abs(val);
        self.adv_pc_zpg_x();
    }

    fn clc(&mut self) {
        // clear the carry flag
        self.set_carry_flag(false);
        self.adv_pc_imp();
    }

    fn ora_abs_y(&mut self) {
        self.ora(self.read_abs_y());
        self.adv_pc_abs_y();
    }

    fn ora_abs_x(&mut self) {
        self.ora(self.read_abs_x());
        self.adv_pc_abs_x();
    }

    fn asl_abs_x(&mut self) {
        let val = self.asl(self.read_abs_x());
        self.write_abs_x(val, true);
        self.adv_pc_abs_x();
    }

    // HI 2
    fn jsr_abs(&mut self) {
        // push the current program counter to the stack
        self.push_pc(2);
        self.jump();
    }

    fn and_ind_x(&mut self) {
        self.and(self.read_ind_x());
        self.adv_pc_ind_x();
    }

    fn bit_zpg(&mut self) {
        self.bit(self.read_zpg());
        self.adv_pc_zpg();
    }

    fn and_zpg(&mut self) {
        self.and(self.read_zpg());
        self.adv_pc_zpg();
    }

    fn rol_zpg(&mut self) {
        let val = self.rol(self.read_zpg());
        self.write_zpg(val);
        self.adv_pc_zpg();
    }

    fn plp(&mut self) {
        self.sr = self.pop_stack();
        self.adv_pc_imm();
    }

    fn and_imm(&mut self) {
        self.and(self.read_imm());
        self.adv_pc_imm();
    }

    fn rol_a(&mut self) {
        self.acc = self.rol(self.acc);
        self.adv_pc_imp();
    }

    fn bit_abs(&mut self) {
        self.bit(self.read_abs());
        self.adv_pc_abs();
    }

    fn and_abs(&mut self) {
        self.and(self.read_abs());
        self.adv_pc_abs();
    }

    fn rol_abs(&mut self) {
        let val = self.rol(self.read_abs());
        self.write_abs(val);
        self.adv_pc_abs();
    }

    // HI 3
    fn bmi(&mut self) {
        self.cond_branch(self.negative_flag());
    }

    fn and_ind_y(&mut self) {
        self.and(self.read_ind_y());
        self.adv_pc_ind_y();
    }

    fn and_zpg_x(&mut self) {
        self.and(self.read_zpg_x());
        self.adv_pc_zpg_x();
    }

    fn rol_zpg_x(&mut self) {
        self.rol(self.read_zpg_x());
        self.adv_pc_zpg_x();
    }

    fn sec(&mut self) {
        self.set_carry_flag(true);
        self.adv_pc_imp();
    }

    fn and_abs_y(&mut self) {
        self.and(self.read_abs_y());
        self.adv_pc_abs_y();
    }

    fn and_abs_x(&mut self) {
        self.and(self.read_abs_x());
        self.adv_pc_abs_x();
    }

    fn rol_abs_x(&mut self) {
        let val = self.rol(self.read_abs_x());
        self.write_abs_x(val, true);
        self.adv_pc_abs_x();
    }

    // HI 4
    fn rti(&mut self) {
        self.sr = self.pop_stack();
        self.pop_pc();
    }

    fn eor_ind_x(&mut self) {
        self.eor(self.read_ind_x());
        self.adv_pc_ind_x();
    }

    fn eor_zpg(&mut self) {
        self.eor(self.read_zpg());
        self.adv_pc_zpg();
    }

    fn lsr_zpg(&mut self) {
        let val = self.lsr(self.read_zpg());
        self.write_zpg(val);
        self.adv_pc_zpg();
    }

    fn pha(&mut self) {
        self.push_stack(self.acc);
        self.adv_pc_imm();
    }

    fn eor_imm(&mut self) {
        self.eor(self.read_imm());
        self.adv_pc_imm();
    }

    fn lsr_a(&mut self) {
        self.acc = self.lsr(self.acc);
        self.adv_pc_imp();
    }

    fn jmp_abs(&mut self) {
        self.jump();
    }

    fn eor_abs(&mut self) {
        self.eor(self.read_abs());
        self.adv_pc_abs();
    }

    fn lsr_abs(&mut self) {
        let val = self.lsr(self.read_abs());
        self.write_abs(val);
        self.adv_pc_abs();
    }

    // HI 5
    fn bvc(&mut self) {
        self.cond_branch(!self.overflow_flag());
    }

    fn eor_ind_y(&mut self) {
        self.eor(self.read_ind_y());
        self.adv_pc_ind_y();
    }

    fn eor_zpg_x(&mut self) {
        self.eor(self.read_zpg_x());
        self.adv_pc_zpg_x();
    }

    fn lsr_zpg_x(&mut self) {
        let val = self.lsr(self.read_zpg());
        self.write_zpg(val);
        self.adv_pc_zpg_x();
    }

    fn cli(&mut self) {
        self.set_interrupt_disable(false);
        self.adv_pc_imp();
    }

    fn eor_abs_y(&mut self) {
        self.eor(self.read_abs_y());
        self.adv_pc_abs_y();
    }

    fn eor_abs_x(&mut self) {
        self.eor(self.read_abs_x());
        self.adv_pc_abs_x();
    }

    fn lsr_abs_x(&mut self) {
        let val = self.lsr(self.read_abs_x());
        self.write_abs_x(val, true);
        self.adv_pc_abs_x();
    }

    // HI 6
    fn rts(&mut self) {
        self.pop_pc();
        self.pc += 1;
    }

    fn adc_ind_x(&mut self) {
        self.adc(self.read_ind_x());
        self.adv_pc_ind_x();
    }

    fn adc_zpg(&mut self) {
        self.adc(self.read_zpg());
        self.adv_pc_zpg();
    }

    fn ror_zpg(&mut self) {
        let val = self.ror(self.read_zpg());
        self.write_zpg(val);
        self.adv_pc_zpg();
    }

    fn pla(&mut self) {
        let stack_val = self.pop_stack();
        self.lda(stack_val);
        self.adv_pc_imp();
    }

    fn adc_imm(&mut self) {
        self.adc(self.read_imm());
        self.adv_pc_imm();
    }

    fn ror_a(&mut self) {
        self.acc = self.ror(self.acc);
        self.adv_pc_imp();
    }

    fn jmp_ind(&mut self) {
        self.pc = self.cpu_mem.read_ind(self.pc);
    }

    fn adc_abs(&mut self) {
        self.adc(self.read_abs());
        self.adv_pc_abs();
    }

    fn ror_abs(&mut self) {
        let val = self.ror(self.read_abs());
        self.write_abs(val);
        self.adv_pc_abs();
    }

    // HI 7
    fn bvs(&mut self) {
        self.cond_branch(self.overflow_flag());
    }

    fn adc_ind_y(&mut self) {
        self.adc(self.read_ind_y());
        self.adv_pc_ind_y();
    }

    fn adc_zpg_x(&mut self) {
        self.adc(self.read_zpg_x());
        self.adv_pc_zpg_x();
    }

    fn ror_zpg_x(&mut self) {
        let val = self.ror(self.read_zpg_x());
        self.write_zpg_x(val);
        self.adv_pc_zpg_x();
    }

    fn sei(&mut self) {
        self.set_interrupt_disable(true);
        self.adv_pc_imp();
    }

    fn adc_abs_y(&mut self) {
        self.adc(self.read_abs_y());
        self.adv_pc_abs_y();
    }

    fn adc_abs_x(&mut self) {
        self.adc(self.read_abs_x());
        self.adv_pc_abs_x();
    }

    fn ror_abs_x(&mut self) {
        let val = self.ror(self.read_abs_x());
        self.write_abs_x(val, true);
        self.adv_pc_abs_x();
    }

    // HI 8
    fn sta_ind_x(&mut self) {
        self.write_ind_x(self.acc);
        self.adv_pc_ind_x();
    }

    fn sty_zpg(&mut self) {
        self.write_zpg(self.iry);
        self.adv_pc_zpg();
    }

    fn sta_zpg(&mut self) {
        self.write_zpg(self.acc);
        self.adv_pc_zpg();
    }

    fn stx_zpg(&mut self) {
        self.write_zpg(self.irx);
        self.adv_pc_zpg();
    }

    fn dey(&mut self) {
        self.iry = self.dec_val(self.iry);
        self.adv_pc_imp();
    }

    fn txa(&mut self) {
        self.acc = self.irx;
        self.set_nz_flags(self.acc);
        self.adv_pc_imp();
    }

    fn sty_abs(&mut self) {
        self.write_abs(self.iry);
        self.adv_pc_abs();
    }

    fn sta_abs(&mut self) {
        self.write_abs(self.acc);
        self.adv_pc_abs();
    }

    fn stx_abs(&mut self) {
        self.write_abs(self.irx);
        self.adv_pc_abs();
    }

    // HI 9
    fn bcc(&mut self) {
        self.cond_branch(!self.carry_flag());
    }

    fn sta_ind_y(&mut self) {
        self.write_ind_y(self.acc, false);
        self.adv_pc_ind_y();
    }

    fn sty_zpg_x(&mut self) {
        self.write_zpg_x(self.iry);
        self.adv_pc_zpg_x();
    }

    fn sta_zpg_x(&mut self) {
        self.write_zpg_x(self.acc);
        self.adv_pc_zpg_x();
    }

    fn stx_zpg_y(&mut self) {
        self.write_zpg_y(self.irx);
        self.adv_pc_zpg_y();
    }

    fn tya(&mut self) {
        self.acc = self.iry;
        self.set_nz_flags(self.acc);
        self.adv_pc_imp();
    }

    fn sta_abs_y(&mut self) {
        self.write_abs_y(self.acc, false);
        self.adv_pc_abs_y();
    }

    fn txs(&mut self) {
        self.sr = self.irx;
        self.adv_pc_imp();
    }

    fn sta_abs_x(&mut self) {
        self.write_abs_x(self.acc, false);
        self.adv_pc_abs_x();
    }

    // HI A
    fn ldy_imm(&mut self) {
        self.ldy(self.read_imm());
        self.adv_pc_imm();
    }

    fn lda_ind_x(&mut self) {
        self.lda(self.read_ind_x());
        self.adv_pc_ind_x();
    }

    fn ldx_imm(&mut self) {
        self.ldx(self.read_imm());
        self.adv_pc_imm();
    }

    fn ldy_zpg(&mut self) {
        self.ldy(self.read_zpg());
        self.adv_pc_zpg();
    }

    fn lda_zpg(&mut self) {
        self.lda(self.read_zpg());
        self.adv_pc_zpg();
    }

    fn ldx_zpg(&mut self) {
        self.ldx(self.read_zpg());
        self.adv_pc_zpg();
    }

    fn tay(&mut self) {
        self.iry = self.acc;
        self.set_nz_flags(self.iry);
        self.adv_pc_imp();
    }

    fn lda_imm(&mut self) {
        self.lda(self.read_imm());
        self.adv_pc_imm();
    }

    fn tax(&mut self) {
        self.irx = self.acc;
        self.set_nz_flags(self.irx);
        self.adv_pc_imp();
    }

    fn ldy_abs(&mut self) {
        self.ldy(self.read_abs());
        self.adv_pc_abs();
    }

    fn lda_abs(&mut self) {
        self.lda(self.read_abs());
        self.adv_pc_abs();
    }

    fn ldx_abs(&mut self) {
        self.ldx(self.read_abs());
        self.adv_pc_abs();
    }

    // HI B
    fn bcs(&mut self) {
        self.cond_branch(self.carry_flag());
    }

    fn lda_ind_y(&mut self) {
        self.lda(self.read_ind_y());
        self.adv_pc_ind_y();
    }

    fn ldy_zpg_x(&mut self) {
        self.ldy(self.read_zpg_x());
        self.adv_pc_zpg_x();
    }

    fn lda_zpg_x(&mut self) {
        self.lda(self.read_zpg_x());
        self.adv_pc_zpg_x();
    }

    fn ldx_zpg_y(&mut self) {
        self.ldx(self.read_zpg_y());
        self.adv_pc_zpg_y();
    }

    fn clv(&mut self) {
        self.set_overflow_flag(false);
        self.adv_pc_imp();
    }

    fn lda_abs_y(&mut self) {
        self.lda(self.read_abs_y());
        self.adv_pc_abs_y();
    }

    fn tsx(&mut self) {
        self.irx = self.sp;
        self.set_nz_flags(self.irx);
        self.adv_pc_imp();
    }

    fn ldy_abs_x(&mut self) {
        self.ldy(self.read_abs_x());
        self.adv_pc_abs_x();
    }

    fn lda_abs_x(&mut self) {
        self.lda(self.read_abs_x());
        self.adv_pc_abs_x();
    }

    fn ldx_abs_y(&mut self) {
        self.ldx(self.read_abs_y());
        self.adv_pc_abs_y();
    }

    // HI C
    fn cpy_imm(&mut self) {
        self.cpy(self.read_imm());
        self.adv_pc_imm();
    }

    fn cmp_ind_x(&mut self) {
        self.cmp(self.read_ind_x());
        self.adv_pc_ind_x();
    }

    fn cpy_zpg(&mut self) {
        self.cpy(self.read_zpg());
        self.adv_pc_zpg();
    }

    fn cmp_zpg(&mut self) {
        self.cmp(self.read_zpg());
        self.adv_pc_zpg();
    }

    fn dec_zpg(&mut self) {
        let val = self.dec_val(self.read_zpg());
        self.write_zpg(val);
        self.adv_pc_zpg();
    }

    fn iny(&mut self) {
        self.iry = self.inc_val(self.iry);
        self.adv_pc_imp();
    }

    fn cmp_imm(&mut self) {
        self.cmp(self.read_imm());
        self.adv_pc_imm();
    }

    fn dex(&mut self) {
        self.irx = self.dec_val(self.irx);
        self.adv_pc_imp();
    }

    fn cpy_abs(&mut self) {
        self.cpy(self.read_abs());
        self.adv_pc_abs();
    }

    fn cmp_abs(&mut self) {
        self.cmp(self.read_abs());
        self.adv_pc_abs();
    }

    fn dec_abs(&mut self) {
        let val = self.dec_val(self.read_abs());
        self.write_abs(val);
        self.adv_pc_abs();
    }

    // HI D
    fn bne(&mut self) {
        self.cond_branch(!self.zero_flag());
    }

    fn cmp_ind_y(&mut self) {
        self.cmp(self.read_ind_y());
        self.adv_pc_ind_y();
    }

    fn cmp_zpg_x(&mut self) {
        self.cmp(self.read_zpg_x());
        self.adv_pc_zpg_x();
    }

    fn dec_zpg_x(&mut self) {
        let val = self.dec_val(self.read_zpg_x());
        self.write_zpg_x(val);
        self.adv_pc_zpg_x();
    }

    fn cld(&mut self) {
        self.set_decimal_flag(false);
        self.adv_pc_imp();
    }

    fn cmp_abs_y(&mut self) {
        self.cmp(self.read_abs_y());
        self.adv_pc_abs_y();
    }

    fn cmp_abs_x(&mut self) {
        self.cmp(self.read_abs_x());
        self.adv_pc_abs_x();
    }

    fn dec_abs_x(&mut self) {
        let val = self.dec_val(self.read_abs_x());
        self.write_abs_x(val, true);
        self.adv_pc_abs_x();
    }

    // HI E
    fn cpx_imm(&mut self) {
        self.cpx(self.read_imm());
        self.adv_pc_imm();
    }

    fn sbc_ind_x(&mut self) {
        self.sbc(self.read_ind_x());
        self.adv_pc_ind_x();
    }

    fn cpx_zpg(&mut self) {
        self.cpx(self.read_zpg());
        self.adv_pc_zpg();
    }

    fn sbc_zpg(&mut self) {
        self.sbc(self.read_zpg());
        self.adv_pc_zpg();
    }

    fn inc_zpg(&mut self) {
        let val = self.inc_val(self.read_zpg());
        self.write_zpg(val);
        self.adv_pc_zpg();
    }

    fn inx(&mut self) {
        self.irx = self.inc_val(self.irx);
        self.adv_pc_imp();
    }

    fn sbc_imm(&mut self) {
        self.sbc(self.read_imm());
        self.adv_pc_imm();
    }

    fn nop(&mut self) {
        // do nothing
        self.adv_pc_imp();
    }

    fn cpx_abs(&mut self) {
        self.cpx(self.read_abs());
        self.adv_pc_abs();
    }

    fn sbc_abs(&mut self) {
        self.sbc(self.read_abs());
        self.adv_pc_abs();
    }

    fn inc_abs(&mut self) {
        let val = self.inc_val(self.read_abs());
        self.write_abs(val);
        self.adv_pc_abs();
    }

    // HI F
    fn beq(&mut self) {
        self.cond_branch(self.zero_flag());
    }

    fn sbc_ind_y(&mut self) {
        self.sbc(self.read_ind_y());
        self.adv_pc_ind_y();
    }

    fn sbc_zpg_x(&mut self) {
        self.sbc(self.read_zpg_x());
        self.adv_pc_zpg_x();
    }

    fn inc_zpg_x(&mut self) {
        let val = self.inc_val(self.read_zpg_x());
        self.write_zpg_x(val);
        self.adv_pc_zpg_x();
    }

    fn sed(&mut self) {
        self.set_decimal_flag(true);
        self.adv_pc_imp();
    }

    fn sbc_abs_y(&mut self) {
        self.sbc(self.read_abs_y());
        self.adv_pc_abs_y();
    }

    fn sbc_abs_x(&mut self) {
        self.sbc(self.read_abs_x());
        self.adv_pc_abs_x();
    }

    fn inc_abs_x(&mut self) {
        let val = self.inc_val(self.read_abs_x());
        self.write_abs_x(val, true);
        self.adv_pc_abs_x();
    }
}

impl Processor for Ricoh2A03<'_> {
    fn process(&mut self) {
        // read the next instruction and process it
        self.process_opcode();
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

#[cfg(test)]
mod tests {
    use super::super::memory::*;
    use super::super::opcodes::*;
    use super::*;
    use std::vec::Vec;

    fn get_test_mem() -> CpuMemory {
        CpuMemory::new()
    }

    fn get_test_2a03<'a>(cpu_mem: &'a mut CpuMemory) -> Ricoh2A03<'a> {
        let cpu = Ricoh2A03::new(0.0, cpu_mem);
        cpu
    }

    #[test]
    fn test_brk() {
        // let mem = get_test_mem();
        // let cpu = get_test_2a03(&mut mem);
    }

    #[test]
    fn test_ora_ind_x() {}

    // HI 0
    #[test]
    fn test_ora_zpg() {}

    #[test]
    fn test_asl_zpg() {}

    #[test]
    fn test_php() {}

    #[test]
    fn test_ora_imm() {}

    #[test]
    fn test_asl_a() {}

    #[test]
    fn test_ora_abs() {}

    #[test]
    fn test_asl_abs() {}

    // HI 1
    #[test]
    fn test_bpl() {}

    #[test]
    fn test_ora_ind_y() {}

    #[test]
    fn test_ora_zpg_x() {}

    #[test]
    fn test_asl_zpg_x() {}

    #[test]
    fn test_clc_imp() {}

    #[test]
    fn test_ora_abs_y() {}

    #[test]
    fn test_ora_abs_x() {}

    #[test]
    fn test_asl_abs_x() {}

    // HI 2
    #[test]
    fn test_jsr_abs() {}

    #[test]
    fn test_and_ind_x() {}

    #[test]
    fn test_bit_zpg() {}

    #[test]
    fn test_and_zpg() {}

    #[test]
    fn test_rol_zpg() {}

    #[test]
    fn test_plp() {}

    #[test]
    fn test_and_imm() {}

    #[test]
    fn test_rol_a() {}

    #[test]
    fn test_bit_abs() {}

    #[test]
    fn test_and_abs() {}

    #[test]
    fn test_rol_abs() {}

    // HI 3
    #[test]
    fn test_bmi() {}

    #[test]
    fn test_and_ind_y() {}

    #[test]
    fn test_and_zpg_x() {}

    #[test]
    fn test_rol_zpg_x() {}

    #[test]
    fn test_sec() {}

    #[test]
    fn test_and_abs_y() {}

    #[test]
    fn test_and_abs_x() {}

    #[test]
    fn test_rol_abs_x() {}

    // HI 4
    #[test]
    fn test_rti() {}

    #[test]
    fn test_eor_ind_x() {}

    #[test]
    fn test_eor_zpg() {}

    #[test]
    fn test_lsr_zpg() {}

    #[test]
    fn test_pha() {}

    #[test]
    fn test_eor_imm() {}

    #[test]
    fn test_lsr_a() {}

    #[test]
    fn test_jmp_abs() {}

    #[test]
    fn test_eor_abs() {}

    #[test]
    fn test_lsr_abs() {}

    // HI 5
    #[test]
    fn test_bvc() {}

    #[test]
    fn test_eor_ind_y() {}

    #[test]
    fn test_eor_zpg_x() {}

    #[test]
    fn test_lsr_zpg_x() {}

    #[test]
    fn test_cli() {}

    #[test]
    fn test_eor_abs_y() {}

    #[test]
    fn test_eor_abs_x() {}

    #[test]
    fn test_lsr_abs_x() {}

    // HI 6
    #[test]
    fn test_rts() {}

    #[test]
    fn test_adc_ind_x() {}

    #[test]
    fn test_adc_zpg() {}

    #[test]
    fn test_ror_zpg() {}

    #[test]
    fn test_pla() {}

    #[test]
    fn test_adc_imm() {}

    #[test]
    fn test_ror_a() {}

    #[test]
    fn test_jmp_ind() {}

    #[test]
    fn test_adc_abs() {}

    #[test]
    fn test_ror_abs() {}

    // HI 7
    #[test]
    fn test_bvs() {}

    #[test]
    fn test_adc_ind_y() {}

    #[test]
    fn test_adc_zpg_x() {}

    #[test]
    fn test_ror_zpg_x() {}

    #[test]
    fn test_sei() {}

    #[test]
    fn test_adc_abs_y() {}

    #[test]
    fn test_adc_abs_x() {}

    #[test]
    fn test_ror_abs_x() {}

    // HI 8
    #[test]
    fn test_sta_ind_x() {}

    #[test]
    fn test_sty_zpg() {}

    #[test]
    fn test_sta_zpg() {}

    #[test]
    fn test_stx_zpg() {}

    #[test]
    fn test_dey() {}

    #[test]
    fn test_txa() {}

    #[test]
    fn test_sty_abs() {}

    #[test]
    fn test_sta_abs() {}

    #[test]
    fn test_stx_abs() {}

    // HI 9
    #[test]
    fn test_bcc() {}

    #[test]
    fn test_sta_ind_y() {}

    #[test]
    fn test_sty_zpg_x() {}

    #[test]
    fn test_sta_zpg_x() {}

    #[test]
    fn test_stx_zpg_y() {}

    #[test]
    fn test_tya() {}

    #[test]
    fn test_sta_abs_y() {}

    #[test]
    fn test_txs() {}

    #[test]
    fn test_sta_abs_x() {}

    // HI A
    #[test]
    fn test_ldy_imm() {}

    #[test]
    fn test_lda_ind_x() {}

    #[test]
    fn test_ldx_imm() {}

    #[test]
    fn test_ldy_zpg() {}

    #[test]
    fn test_lda_zpg() {}

    #[test]
    fn test_ldx_zpg() {}

    #[test]
    fn test_tay() {}

    #[test]
    fn test_lda_imm() {}

    #[test]
    fn test_tax() {}

    #[test]
    fn test_ldy_abs() {}

    #[test]
    fn test_lda_abs() {}

    #[test]
    fn test_ldx_abs() {}

    // HI B
    #[test]
    fn test_bcs() {}

    #[test]
    fn test_lda_ind_y() {}

    #[test]
    fn test_ldy_zpg_x() {}

    #[test]
    fn test_lda_zpg_x() {}

    #[test]
    fn test_ldx_zpg_y() {}

    #[test]
    fn test_clv() {}

    #[test]
    fn test_lda_abs_y() {}

    #[test]
    fn test_tsx() {}

    #[test]
    fn test_ldy_abs_x() {}

    #[test]
    fn test_lda_abs_x() {}

    #[test]
    fn test_ldx_abs_y() {}

    // HI C
    #[test]
    fn test_cpy_imm() {}

    #[test]
    fn test_cmp_ind_x() {}

    #[test]
    fn test_cpy_zpg() {}

    #[test]
    fn test_cmp_zpg() {}

    #[test]
    fn test_dec_zpg() {}

    #[test]
    fn test_iny() {}

    #[test]
    fn test_cmp_imm() {}

    #[test]
    fn test_dex() {}

    #[test]
    fn test_cpy_abs() {}

    #[test]
    fn test_cmp_abs() {}

    #[test]
    fn test_dec_abs() {}

    // HI D
    #[test]
    fn test_bne() {}

    #[test]
    fn test_cmp_ind_y() {}

    #[test]
    fn test_cmp_zpg_x() {}

    #[test]
    fn test_dec_zpg_x() {}

    #[test]
    fn test_cld() {}

    #[test]
    fn test_cmp_abs_y() {}

    #[test]
    fn test_cmp_abs_x() {}

    #[test]
    fn test_dec_abs_x() {}

    // HI E
    #[test]
    fn test_cpx_imm() {}

    #[test]
    fn test_sbc_ind_x() {}

    #[test]
    fn test_cpx_zpg() {}

    #[test]
    fn test_sbc_zpg() {}

    #[test]
    fn test_inc_zpg() {}

    #[test]
    fn test_inx() {}

    #[test]
    fn test_sbc_imm() {}

    #[test]
    fn test_nop() {}

    #[test]
    fn test_cpx_abs() {}

    #[test]
    fn test_sbc_abs() {}

    #[test]
    fn test_inc_abs() {}

    // HI F
    #[test]
    fn test_beq() {}

    #[test]
    fn test_sbc_ind_y() {}

    #[test]
    fn test_sbc_zpg_x() {}

    #[test]
    fn test_inc_zpg_x() {}

    #[test]
    fn test_sed() {}

    #[test]
    fn test_sbc_abs_y() {}

    #[test]
    fn test_sbc_abs_x() {}

    #[test]
    fn test_inc_abs_x() {}
}
