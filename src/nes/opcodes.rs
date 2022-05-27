// HI 0
/// Break / Interrupt
pub const BRK: u8 = 0x00;
/// OR accumulator with indirect X
pub const ORA_IND_X: u8 = 0x01;
/// OR accumulator with zero page
pub const ORA_ZPG: u8 = 0x05;
/// Arithmetic left shift zero page
pub const ASL_ZPG: u8 = 0x06;
/// Push processor status onto stack
pub const PHP: u8 = 0x08;
/// OR accumulator with immediate
pub const ORA_IMM: u8 = 0x09;
/// Arithmetic left shift the accumulator
pub const ASL_A: u8 = 0x0A;
/// OR accumulator with absolute
pub const ORA_ABS: u8 = 0x0D;
/// Arithmetic left shift absolute
pub const ASL_ABS: u8 = 0x0E;

// HI 1
pub const BPL: u8 = 0x10;
pub const ORA_IND_Y: u8 = 0x11;
pub const ORA_ZPG_X: u8 = 0x15;
pub const ASL_ZPG_X: u8 = 0x16;
pub const CLC: u8 = 0x18;
pub const ORA_ABS_Y: u8 = 0x19;
pub const ORA_ABS_X: u8 = 0x1D;
pub const ASL_ABS_X: u8 = 0x1E;

// HI 2
pub const JSR_ABS: u8 = 0x20;
pub const AND_IND_X: u8 = 0x21;
pub const BIT_ZPG: u8 = 0x24;
pub const AND_ZPG: u8 = 0x25;
pub const ROL_ZPG: u8 = 0x26;
pub const PLP: u8 = 0x28;
pub const AND_IMM: u8 = 0x29;
pub const ROL_A: u8 = 0x2A;
pub const BIT_ABS: u8 = 0x2C;
pub const AND_ABS: u8 = 0x2D;
pub const ROL_ABS: u8 = 0x2E;

// HI 3
pub const BMI: u8 = 0x30;
pub const AND_IND_Y: u8 = 0x31;
pub const AND_ZPG_X: u8 = 0x35;
pub const ROL_ZPG_X: u8 = 0x36;
pub const SEC: u8 = 0x38;
pub const AND_ABS_Y: u8 = 0x39;
pub const AND_ABS_X: u8 = 0x3D;
pub const ROL_ABS_X: u8 = 0x3E;

// HI 4
pub const RTI: u8 = 0x40;
pub const EOR_IND_X: u8 = 0x41;
pub const EOR_ZPG: u8 = 0x45;
pub const LSR_ZPG: u8 = 0x46;
pub const PHA: u8 = 0x48;
pub const EOR_IMM: u8 = 0x49;
pub const LSR_A: u8 = 0x4A;
pub const JMP_ABS: u8 = 0x4C;
pub const EOR_ABS: u8 = 0x4D;
pub const LSR_ABS: u8 = 0x4E;

// HI 5
pub const BVC: u8 = 0x50;
pub const EOR_IND_Y: u8 = 0x51;
pub const EOR_ZPG_X: u8 = 0x55;
pub const LSR_ZPG_X: u8 = 0x56;
pub const CLI: u8 = 0x58;
pub const EOR_ABS_Y: u8 = 0x59;
pub const EOR_ABS_X: u8 = 0x5D;
pub const LSR_ABS_X: u8 = 0x5E;

// HI 6
pub const RTS: u8 = 0x60;
pub const ADC_IND_X: u8 = 0x61;
pub const ADC_ZPG: u8 = 0x65;
pub const ROR_ZPG: u8 = 0x66;
pub const PLA: u8 = 0x68;
pub const ADC_IMM: u8 = 0x69;
pub const ROR_A: u8 = 0x6A;
pub const JMP_IND: u8 = 0x6C;
pub const ADC_ABS: u8 = 0x6D;
pub const ROR_ABS: u8 = 0x6E;

// HI 7
pub const BVS: u8 = 0x70;
pub const ADC_IND_Y: u8 = 0x71;
pub const ADC_ZPG_X: u8 = 0x75;
pub const ROR_ZPG_X: u8 = 0x76;
pub const SEI: u8 = 0x78;
pub const ADC_ABS_Y: u8 = 0x79;
pub const ADC_ABS_X: u8 = 0x7D;
pub const ROR_ABS_X: u8 = 0x7E;

// HI 8
pub const STA_IND_X: u8 = 0x81;
pub const STY_ZPG: u8 = 0x84;
pub const STA_ZPG: u8 = 0x85;
pub const STX_ZPG: u8 = 0x86;
pub const DEY: u8 = 0x88;
pub const TXA: u8 = 0x8A;
pub const STY_ABS: u8 = 0x8C;
pub const STA_ABS: u8 = 0x8D;
pub const STX_ABS: u8 = 0x8E;

// HI 9
pub const BCC: u8 = 0x90;
pub const STA_IND_Y: u8 = 0x91;
pub const STY_ZPG_X: u8 = 0x94;
pub const STA_ZPG_X: u8 = 0x95;
pub const STX_ZPG_Y: u8 = 0x96;
pub const TYA: u8 = 0x98;
pub const STA_ABS_Y: u8 = 0x99;
pub const TXS: u8 = 0x9A;
pub const STA_ABS_X: u8 = 0x9D;

// HI A
pub const LDY_IMM: u8 = 0xA0;
pub const LDA_IND_X: u8 = 0xA1;
pub const LDX_IMM: u8 = 0xA2;
pub const LDY_ZPG: u8 = 0xA4;
pub const LDA_ZPG: u8 = 0xA5;
pub const LDX_ZPG: u8 = 0xA6;
pub const TAY: u8 = 0xA8;
pub const LDA_IMM: u8 = 0xA9;
pub const TAX: u8 = 0xAA;
pub const LDY_ABS: u8 = 0xAC;
pub const LDA_ABS: u8 = 0xAD;
pub const LDX_ABS: u8 = 0xAF;

// HI B
pub const BCS: u8 = 0xB0;
pub const LDA_IND_Y: u8 = 0xB1;
pub const LDY_ZPG_X: u8 = 0xB4;
pub const LDA_ZPG_X: u8 = 0xB5;
pub const LDX_ZPG_Y: u8 = 0xB6;
pub const CLV: u8 = 0xB8;
pub const LDA_ABS_Y: u8 = 0xB9;
pub const TSX: u8 = 0xBA;
pub const LDY_ABS_X: u8 = 0xBC;
pub const LDA_ABS_X: u8 = 0xBD;
pub const LDX_ABS_Y: u8 = 0xBE;

// HI C
pub const CPY_IMM: u8 = 0xC0;
pub const CMP_IND_X: u8 = 0xC1;
pub const CPY_ZPG: u8 = 0xC4;
pub const CMP_ZPG: u8 = 0xC5;
pub const DEC_ZPG: u8 = 0xC6;
pub const INY: u8 = 0xC8;
pub const CMP_IMM: u8 = 0xC9;
pub const DEX: u8 = 0xCA;
pub const CPY_ABS: u8 = 0xCC;
pub const CMP_ABS: u8 = 0xCD;
pub const DEC_ABS: u8 = 0xCE;

// HI D
pub const BNE: u8 = 0xD0;
pub const CMP_IND_Y: u8 = 0xD1;
pub const CMP_ZPG_X: u8 = 0xD5;
pub const DEC_ZPG_X: u8 = 0xD6;
pub const CLD: u8 = 0xD8;
pub const CMP_ABS_Y: u8 = 0xD9;
pub const CMP_ABS_X: u8 = 0xDD;
pub const DEC_ABS_X: u8 = 0xDE;

// HI E
pub const CPX_IMM: u8 = 0xE0;
pub const SBC_IND_X: u8 = 0xE1;
pub const CPX_ZPG: u8 = 0xE4;
pub const SBC_ZPG: u8 = 0xE5;
pub const INC_ZPG: u8 = 0xE6;
pub const INX: u8 = 0xE8;
pub const SBC_IMM: u8 = 0xE9;
pub const NOP: u8 = 0xEA;
pub const CPX_ABS: u8 = 0xEC;
pub const SBC_ABS: u8 = 0xED;
pub const INC_ABS: u8 = 0xEE;

// HI F
pub const BEQ: u8 = 0xF0;
pub const SBC_IND_Y: u8 = 0xF1;
pub const SBC_ZPG_X: u8 = 0xF5;
pub const INC_ZPG_X: u8 = 0xF6;
pub const SED: u8 = 0xF8;
pub const SBC_ABS_Y: u8 = 0xF9;
pub const SBC_ABS_X: u8 = 0xFD;
pub const INC_ABS_X: u8 = 0xFE;

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;
    use std::hash::Hash;

    #[test]
    fn test_opcode_values() {
        assert_eq!(BRK, 0x00);
        assert_eq!(ORA_IND_X, 0x01);
        assert_eq!(ORA_ZPG, 0x05);
        assert_eq!(ASL_ZPG, 0x06);
        assert_eq!(PHP, 0x08);
        assert_eq!(ORA_IMM, 0x09);
        assert_eq!(ASL_A, 0x0A);
        assert_eq!(ORA_ABS, 0x0D);
        assert_eq!(ASL_ABS, 0x0E);

        assert_eq!(BPL, 0x10);
        assert_eq!(ORA_IND_Y, 0x11);
        assert_eq!(ORA_ZPG_X, 0x15);
        assert_eq!(ASL_ZPG_X, 0x16);
        assert_eq!(CLC, 0x18);
        assert_eq!(ORA_ABS_Y, 0x19);
        assert_eq!(ORA_ABS_X, 0x1D);
        assert_eq!(ASL_ABS_X, 0x1E);

        assert_eq!(JSR_ABS, 0x20);
        assert_eq!(AND_IND_X, 0x21);
        assert_eq!(BIT_ZPG, 0x24);
        assert_eq!(AND_ZPG, 0x25);
        assert_eq!(ROL_ZPG, 0x26);
        assert_eq!(PLP, 0x28);
        assert_eq!(AND_IMM, 0x29);
        assert_eq!(ROL_A, 0x2A);
        assert_eq!(BIT_ABS, 0x2C);
        assert_eq!(AND_ABS, 0x2D);
        assert_eq!(ROL_ABS, 0x2E);

        assert_eq!(BMI, 0x30);
        assert_eq!(AND_IND_Y, 0x31);
        assert_eq!(AND_ZPG_X, 0x35);
        assert_eq!(ROL_ZPG_X, 0x36);
        assert_eq!(SEC, 0x38);
        assert_eq!(AND_ABS_Y, 0x39);
        assert_eq!(AND_ABS_X, 0x3D);
        assert_eq!(ROL_ABS_X, 0x3E);

        assert_eq!(RTI, 0x40);
        assert_eq!(EOR_IND_X, 0x41);
        assert_eq!(EOR_ZPG, 0x45);
        assert_eq!(LSR_ZPG, 0x46);
        assert_eq!(PHA, 0x48);
        assert_eq!(EOR_IMM, 0x49);
        assert_eq!(LSR_A, 0x4A);
        assert_eq!(JMP_ABS, 0x4C);
        assert_eq!(EOR_ABS, 0x4D);
        assert_eq!(LSR_ABS, 0x4E);

        assert_eq!(BVC, 0x50);
        assert_eq!(EOR_IND_Y, 0x51);
        assert_eq!(EOR_ZPG_X, 0x55);
        assert_eq!(LSR_ZPG_X, 0x56);
        assert_eq!(CLI, 0x58);
        assert_eq!(EOR_ABS_Y, 0x59);
        assert_eq!(EOR_ABS_X, 0x5D);
        assert_eq!(LSR_ABS_X, 0x5E);

        assert_eq!(RTS, 0x60);
        assert_eq!(ADC_IND_X, 0x61);
        assert_eq!(ADC_ZPG, 0x65);
        assert_eq!(ROR_ZPG, 0x66);
        assert_eq!(PLA, 0x68);
        assert_eq!(ADC_IMM, 0x69);
        assert_eq!(ROR_A, 0x6A);
        assert_eq!(JMP_IND, 0x6C);
        assert_eq!(ADC_ABS, 0x6D);
        assert_eq!(ROR_ABS, 0x6E);

        assert_eq!(BVS, 0x70);
        assert_eq!(ADC_IND_Y, 0x71);
        assert_eq!(ADC_ZPG_X, 0x75);
        assert_eq!(ROR_ZPG_X, 0x76);
        assert_eq!(SEI, 0x78);
        assert_eq!(ADC_ABS_Y, 0x79);
        assert_eq!(ADC_ABS_X, 0x7D);
        assert_eq!(ROR_ABS_X, 0x7E);

        assert_eq!(STA_IND_X, 0x81);
        assert_eq!(STY_ZPG, 0x84);
        assert_eq!(STA_ZPG, 0x85);
        assert_eq!(STX_ZPG, 0x86);
        assert_eq!(DEY, 0x88);
        assert_eq!(TXA, 0x8A);
        assert_eq!(STY_ABS, 0x8C);
        assert_eq!(STA_ABS, 0x8D);
        assert_eq!(STX_ABS, 0x8E);

        assert_eq!(BCC, 0x90);
        assert_eq!(STA_IND_Y, 0x91);
        assert_eq!(STY_ZPG_X, 0x94);
        assert_eq!(STA_ZPG_X, 0x95);
        assert_eq!(STX_ZPG_Y, 0x96);
        assert_eq!(TYA, 0x98);
        assert_eq!(STA_ABS_Y, 0x99);
        assert_eq!(TXS, 0x9A);
        assert_eq!(STA_ABS_X, 0x9D);

        assert_eq!(LDY_IMM, 0xA0);
        assert_eq!(LDA_IND_X, 0xA1);
        assert_eq!(LDX_IMM, 0xA2);
        assert_eq!(LDY_ZPG, 0xA4);
        assert_eq!(LDA_ZPG, 0xA5);
        assert_eq!(LDX_ZPG, 0xA6);
        assert_eq!(TAY, 0xA8);
        assert_eq!(LDA_IMM, 0xA9);
        assert_eq!(TAX, 0xAA);
        assert_eq!(LDY_ABS, 0xAC);
        assert_eq!(LDA_ABS, 0xAD);
        assert_eq!(LDX_ABS, 0xAF);

        assert_eq!(BCS, 0xB0);
        assert_eq!(LDA_IND_Y, 0xB1);
        assert_eq!(LDY_ZPG_X, 0xB4);
        assert_eq!(LDA_ZPG_X, 0xB5);
        assert_eq!(LDX_ZPG_Y, 0xB6);
        assert_eq!(CLV, 0xB8);
        assert_eq!(LDA_ABS_Y, 0xB9);
        assert_eq!(TSX, 0xBA);
        assert_eq!(LDY_ABS_X, 0xBC);
        assert_eq!(LDA_ABS_X, 0xBD);
        assert_eq!(LDX_ABS_Y, 0xBE);

        assert_eq!(CPY_IMM, 0xC0);
        assert_eq!(CMP_IND_X, 0xC1);
        assert_eq!(CPY_ZPG, 0xC4);
        assert_eq!(CMP_ZPG, 0xC5);
        assert_eq!(DEC_ZPG, 0xC6);
        assert_eq!(INY, 0xC8);
        assert_eq!(CMP_IMM, 0xC9);
        assert_eq!(DEX, 0xCA);
        assert_eq!(CPY_ABS, 0xCC);
        assert_eq!(CMP_ABS, 0xCD);
        assert_eq!(DEC_ABS, 0xCE);

        assert_eq!(BNE, 0xD0);
        assert_eq!(CMP_IND_Y, 0xD1);
        assert_eq!(CMP_ZPG_X, 0xD5);
        assert_eq!(DEC_ZPG_X, 0xD6);
        assert_eq!(CLD, 0xD8);
        assert_eq!(CMP_ABS_Y, 0xD9);
        assert_eq!(CMP_ABS_X, 0xDD);
        assert_eq!(DEC_ABS_X, 0xDE);

        assert_eq!(CPX_IMM, 0xE0);
        assert_eq!(SBC_IND_X, 0xE1);
        assert_eq!(CPX_ZPG, 0xE4);
        assert_eq!(SBC_ZPG, 0xE5);
        assert_eq!(INC_ZPG, 0xE6);
        assert_eq!(INX, 0xE8);
        assert_eq!(SBC_IMM, 0xE9);
        assert_eq!(NOP, 0xEA);
        assert_eq!(CPX_ABS, 0xEC);
        assert_eq!(SBC_ABS, 0xED);
        assert_eq!(INC_ABS, 0xEE);

        assert_eq!(BEQ, 0xF0);
        assert_eq!(SBC_IND_Y, 0xF1);
        assert_eq!(SBC_ZPG_X, 0xF5);
        assert_eq!(INC_ZPG_X, 0xF6);
        assert_eq!(SED, 0xF8);
        assert_eq!(SBC_ABS_Y, 0xF9);
        assert_eq!(SBC_ABS_X, 0xFD);
        assert_eq!(INC_ABS_X, 0xFE);
    }

    #[test]
    fn test_duplicates() {
        let vec = vec![
            BRK, ORA_IND_X, ORA_ZPG, ASL_ZPG, PHP, ORA_IMM, ASL_A, ORA_ABS, ASL_ABS, BPL,
            ORA_IND_Y, ORA_ZPG_X, ASL_ZPG_X, CLC, ORA_ABS_Y, ORA_ABS_X, ASL_ABS_X, JSR_ABS,
            AND_IND_X, BIT_ZPG, AND_ZPG, ROL_ZPG, PLP, AND_IMM, ROL_A, BIT_ABS, AND_ABS, ROL_ABS,
            BMI, AND_IND_Y, AND_ZPG_X, ROL_ZPG_X, SEC, AND_ABS_Y, AND_ABS_X, ROL_ABS_X, RTI,
            EOR_IND_X, EOR_ZPG, LSR_ZPG, PHA, EOR_IMM, LSR_A, JMP_ABS, EOR_ABS, LSR_ABS, BVC,
            EOR_IND_Y, EOR_ZPG_X, LSR_ZPG_X, CLI, EOR_ABS_Y, EOR_ABS_X, LSR_ABS_X, RTS, ADC_IND_X,
            ADC_ZPG, ROR_ZPG, PLA, ADC_IMM, ROR_A, JMP_IND, ADC_ABS, ROR_ABS, BVS, ADC_IND_Y,
            ADC_ZPG_X, ROR_ZPG_X, SEI, ADC_ABS_Y, ADC_ABS_X, ROR_ABS_X, STA_IND_X, STY_ZPG,
            STA_ZPG, STX_ZPG, DEY, TXA, STY_ABS, STA_ABS, STX_ABS, BCC, STA_IND_Y, STY_ZPG_X,
            STA_ZPG_X, STX_ZPG_Y, TYA, STA_ABS_Y, TXS, STA_ABS_X, LDY_IMM, LDA_IND_X, LDX_IMM,
            LDY_ZPG, LDA_ZPG, LDX_ZPG, TAY, LDA_IMM, TAX, LDY_ABS, LDA_ABS, LDX_ABS, BCS,
            LDA_IND_Y, LDY_ZPG_X, LDA_ZPG_X, LDX_ZPG_Y, CLV, LDA_ABS_Y, TSX, LDY_ABS_X, LDA_ABS_X,
            LDX_ABS_Y, CPY_IMM, CMP_IND_X, CPY_ZPG, CMP_ZPG, DEC_ZPG, INY, CMP_IMM, DEX, CPY_ABS,
            CMP_ABS, DEC_ABS, BNE, CMP_IND_Y, CMP_ZPG_X, DEC_ZPG_X, CLD, CMP_ABS_Y, CMP_ABS_X,
            DEC_ABS_X, CPX_IMM, SBC_IND_X, CPX_ZPG, SBC_ZPG, INC_ZPG, INX, SBC_IMM, NOP, CPX_ABS,
            SBC_ABS, INC_ABS, BEQ, SBC_IND_Y, SBC_ZPG_X, INC_ZPG_X, SED, SBC_ABS_Y, SBC_ABS_X,
            INC_ABS_X,
        ];

        let val = has_unique_elements(vec);
        println!("{}", val);
        assert!(val);
    }

    fn has_unique_elements<T>(iter: T) -> bool
    where
        T: IntoIterator,
        T::Item: Eq + Hash,
    {
        let mut uniq = HashSet::new();
        iter.into_iter().all(move |x| uniq.insert(x))
    }
}
