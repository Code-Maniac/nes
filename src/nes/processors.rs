use super::memory::CpuMemory;
use super::memory::Memory;
use super::opcodes::*;

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
    cpu_mem: &'a mut CpuMemory,
}

impl<'a> Ricoh2A03<'a> {
    pub fn new(clockspeed: f64, cpu_mem: &'a mut CpuMemory) -> Self {
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

    fn process_opcode(&mut self) {
        let opcode = self.cpu_mem.read(self.pc);

        match opcode {
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
            CLC_IMP => {
                self.clc_imp();
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
            BMI_REL => {
                self.bmi_rel();
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
            BVC_REL => {
                self.bvc_rel();
            }
            EOR_IN_Y => {
                self.eor_in_y();
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
            BVS_REL => {
                self.bvs_rel();
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
            BCC_REL => {
                self.bcc_rel();
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
            BCS_REL => {
                self.bcs_rel();
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
            BNE_REL => {
                self.bne_rel();
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
            BEQ_REL => {
                self.beq_rel();
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

    // HI 0
    fn brk(&mut self) {}
    fn ora_ind_x(&mut self) {}
    fn ora_zpg(&mut self) {}
    fn asl_zpg(&mut self) {}
    fn php(&mut self) {}
    fn ora_imm(&mut self) {}
    fn asl_a(&mut self) {}
    fn ora_abs(&mut self) {}
    fn asl_abs(&mut self) {}

    // HI 1
    fn bpl(&mut self) {}
    fn ora_ind_y(&mut self) {}
    fn ora_zpg_x(&mut self) {}
    fn asl_zpg_x(&mut self) {}
    fn clc_imp(&mut self) {}
    fn ora_abs_y(&mut self) {}
    fn ora_abs_x(&mut self) {}
    fn asl_abs_x(&mut self) {}

    // HI 2
    fn jsr_abs(&mut self) {}
    fn and_ind_x(&mut self) {}
    fn bit_zpg(&mut self) {}
    fn and_zpg(&mut self) {}
    fn rol_zpg(&mut self) {}
    fn plp(&mut self) {}
    fn and_imm(&mut self) {}
    fn rol_a(&mut self) {}
    fn bit_abs(&mut self) {}
    fn and_abs(&mut self) {}
    fn rol_abs(&mut self) {}

    // HI 3
    fn bmi_rel(&mut self) {}
    fn and_ind_y(&mut self) {}
    fn and_zpg_x(&mut self) {}
    fn rol_zpg_x(&mut self) {}
    fn sec(&mut self) {}
    fn and_abs_y(&mut self) {}
    fn and_abs_x(&mut self) {}
    fn rol_abs_x(&mut self) {}

    // HI 4
    fn rti(&mut self) {}
    fn eor_ind_x(&mut self) {}
    fn eor_zpg(&mut self) {}
    fn lsr_zpg(&mut self) {}
    fn pha(&mut self) {}
    fn eor_imm(&mut self) {}
    fn lsr_a(&mut self) {}
    fn jmp_abs(&mut self) {}
    fn eor_abs(&mut self) {}
    fn lsr_abs(&mut self) {}

    // HI 5
    fn bvc_rel(&mut self) {}
    fn eor_in_y(&mut self) {}
    fn eor_zpg_x(&mut self) {}
    fn lsr_zpg_x(&mut self) {}
    fn cli(&mut self) {}
    fn eor_abs_y(&mut self) {}
    fn eor_abs_x(&mut self) {}
    fn lsr_abs_x(&mut self) {}

    // HI 6
    fn rts(&mut self) {}
    fn adc_ind_x(&mut self) {}
    fn adc_zpg(&mut self) {}
    fn ror_zpg(&mut self) {}
    fn pla(&mut self) {}
    fn adc_imm(&mut self) {}
    fn ror_a(&mut self) {}
    fn jmp_ind(&mut self) {}
    fn adc_abs(&mut self) {}
    fn ror_abs(&mut self) {}

    // HI 7
    fn bvs_rel(&mut self) {}
    fn adc_ind_y(&mut self) {}
    fn adc_zpg_x(&mut self) {}
    fn ror_zpg_x(&mut self) {}
    fn sei(&mut self) {}
    fn adc_abs_y(&mut self) {}
    fn adc_abs_x(&mut self) {}
    fn ror_abs_x(&mut self) {}

    // HI 8
    fn sta_ind_x(&mut self) {}
    fn sty_zpg(&mut self) {}
    fn sta_zpg(&mut self) {}
    fn stx_zpg(&mut self) {}
    fn dey(&mut self) {}
    fn txa(&mut self) {}
    fn sty_abs(&mut self) {}
    fn sta_abs(&mut self) {}
    fn stx_abs(&mut self) {}

    // HI 9
    fn bcc_rel(&mut self) {}
    fn sta_ind_y(&mut self) {}
    fn sty_zpg_x(&mut self) {}
    fn sta_zpg_x(&mut self) {}
    fn stx_zpg_y(&mut self) {}
    fn tya(&mut self) {}
    fn sta_abs_y(&mut self) {}
    fn txs(&mut self) {}
    fn sta_abs_x(&mut self) {}

    // HI A
    fn ldy_imm(&mut self) {}
    fn lda_ind_x(&mut self) {}
    fn ldx_imm(&mut self) {}
    fn ldy_zpg(&mut self) {}
    fn lda_zpg(&mut self) {}
    fn ldx_zpg(&mut self) {}
    fn tay(&mut self) {}
    fn lda_imm(&mut self) {}
    fn tax(&mut self) {}
    fn ldy_abs(&mut self) {}
    fn lda_abs(&mut self) {}
    fn ldx_abs(&mut self) {}

    // HI B
    fn bcs_rel(&mut self) {}
    fn lda_ind_y(&mut self) {}
    fn ldy_zpg_x(&mut self) {}
    fn lda_zpg_x(&mut self) {}
    fn ldx_zpg_y(&mut self) {}
    fn clv(&mut self) {}
    fn lda_abs_y(&mut self) {}
    fn tsx(&mut self) {}
    fn ldy_abs_x(&mut self) {}
    fn lda_abs_x(&mut self) {}
    fn ldx_abs_y(&mut self) {}

    // HI C
    fn cpy_imm(&mut self) {}
    fn cmp_ind_x(&mut self) {}
    fn cpy_zpg(&mut self) {}
    fn cmp_zpg(&mut self) {}
    fn dec_zpg(&mut self) {}
    fn iny(&mut self) {}
    fn cmp_imm(&mut self) {}
    fn dex(&mut self) {}
    fn cpy_abs(&mut self) {}
    fn cmp_abs(&mut self) {}
    fn dec_abs(&mut self) {}

    // HI D
    fn bne_rel(&mut self) {}
    fn cmp_ind_y(&mut self) {}
    fn cmp_zpg_x(&mut self) {}
    fn dec_zpg_x(&mut self) {}
    fn cld(&mut self) {}
    fn cmp_abs_y(&mut self) {}
    fn cmp_abs_x(&mut self) {}
    fn dec_abs_x(&mut self) {}

    // HI E
    fn cpx_imm(&mut self) {}
    fn sbc_ind_x(&mut self) {}
    fn cpx_zpg(&mut self) {}
    fn sbc_zpg(&mut self) {}
    fn inc_zpg(&mut self) {}
    fn inx(&mut self) {}
    fn sbc_imm(&mut self) {}
    fn nop(&mut self) {}
    fn cpx_abs(&mut self) {}
    fn sbc_abs(&mut self) {}
    fn inc_abs(&mut self) {}

    // HI F
    fn beq_rel(&mut self) {}
    fn sbc_ind_y(&mut self) {}
    fn sbc_zpg_x(&mut self) {}
    fn inc_zpg_x(&mut self) {}
    fn sed(&mut self) {}
    fn sbc_abs_y(&mut self) {}
    fn sbc_abs_x(&mut self) {}
    fn inc_abs_x(&mut self) {}
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
    fn test_ora_ind_x() {
        let mut mem = get_test_mem();
        let mut cpu = get_test_2a03(&mut mem);

        // construct some test data as vec of tuples:
        // (opcode, accum_val, zp_addr, xregister, addr_at_zp_ind_x, val_at_ind_x_addr)
        let test_data: Vec<(u8, u8, u8, u8, u16, u8)> = vec![
            (ORA_IND_X, 0x20, 0x00, 0x00, 0x007f, 0x11),
            (ORA_IND_X, 0x20, 0x00, 0x00, 0x007f, 0x11),
            (ORA_IND_X, 0x20, 0x00, 0x00, 0x007f, 0x11),
            (ORA_IND_X, 0x20, 0x00, 0x00, 0x007f, 0x11),
            (ORA_IND_X, 0x20, 0x00, 0x00, 0x007f, 0x11),
            (ORA_IND_X, 0x20, 0x00, 0x00, 0x007f, 0x11),
        ];

        for i in 0..test_data.len() {
            // setup the opcode
            let data = &test_data[i];
            cpu.cpu_mem.write(CPU_ADDR_PRG_ROM as u16, data.0);
            cpu.cpu_mem.write(CPU_ADDR_PRG_ROM as u16 + 1, data.2);
            cpu.accum = data.1;
            cpu.irx = data.3;
            cpu.cpu_mem.write_zp_idx(data.2, data.3, data.4 as u8);
            cpu.cpu_mem
                .write_zp_idx(data.2 + 1, data.3, (data.4 >> 8) as u8);
            cpu.cpu_mem.write(data.4, data.5);

            cpu.process();

            // expected value is the value at the address | accumulator
            let expected_val = data.5 | data.1;
            println!("{:#02x} == {:#02x}", data.4, expected_val);
            assert!(cpu.cpu_mem.read(data.4) == expected_val)
        }
    }

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
}
