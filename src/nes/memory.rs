use std::num::Wrapping;

// constants defining the cpu memory map
pub const CPU_MEM_SIZE: usize = 0x10000;
pub const CPU_MEM_PAGE_SIZE: usize = 0x100;
pub const CPU_ADDR_RAM: usize = 0x0000;
pub const CPU_ADDR_IO_REG: usize = 0x2000;
pub const CPU_ADDR_EXP_ROM: usize = 0x4020;
pub const CPU_ADDR_SRAM: usize = 0x6000;
pub const CPU_ADDR_PRG_ROM: usize = 0x8000;

// constants definiting the ppu memory map
pub const PPU_MEM_SIZE: usize = 0x4000;
pub const PPU_ADDR_PATTERN_TABLES: usize = 0x0000;
pub const PPU_ADDR_NAME_TABLES: usize = 0x2000;
pub const PPU_ADDR_PALETTES: usize = 0x3F00;

pub trait Memory {
    fn read(&self, addr: u16) -> u8;
    fn write(&mut self, addr: u16, val: u8);
}

pub struct CpuMemory {
    memory: [u8; CPU_MEM_SIZE],
}

impl CpuMemory {
    pub fn new() -> Self {
        CpuMemory {
            memory: [0; CPU_MEM_SIZE],
        }
    }

    // calculate the zero page index address
    fn get_zp_idx(&self, addr: u8, idx: u8) -> u16 {
        let real_addr = Wrapping(addr) + Wrapping(idx);
        real_addr.0 as u16
    }

    // calculate absolute addr from the 2 operands
    fn get_abs(&self, addr1: u8, addr2: u8) -> u16 {
        //NOTE: add wrapping
        (addr1 as u16) | (addr2 as u16) << 8
    }

    // calculate the indexed absolute addr form the 2 operands + index
    fn get_abs_idx(&self, addr1: u8, addr2: u8, idx: u8) -> u16 {
        let abs_addr = self.get_abs(addr1, addr2);
        let abs_idx = Wrapping(abs_addr) + Wrapping(idx as u16);

        abs_idx.0
    }

    // read byte from the addr in zero page
    pub fn read_zp(&self, addr: u8) -> u8 {
        self.read(addr as u16)
    }

    // write byte to addr in zero page
    pub fn write_zp(&mut self, addr: u8, val: u8) {
        self.write(addr as u16, val);
    }

    // read byte from indexed address in zero page
    pub fn read_zp_idx(&self, addr: u8, idx: u8) -> u8 {
        self.read(self.get_zp_idx(addr, idx))
    }

    // write byte to indexed address in zero page
    pub fn write_zp_idx(&mut self, addr: u8, idx: u8, val: u8) {
        self.write(self.get_zp_idx(addr, val), val);
    }

    // read absolute byte given 2 operands of the instruction
    pub fn read_abs(&self, addr1: u8, addr2: u8) -> u8 {
        let addr = self.get_abs(addr1, addr2);
        self.read(addr)
    }

    // write to absolute byte given 2 operands of the instruction and the value
    pub fn write_abs(&mut self, addr1: u8, addr2: u8, val: u8) {
        let addr = self.get_abs(addr1, addr2);
        self.write(addr, val);
    }

    // read absolute byte given 2 operands of instruction and index
    pub fn read_abs_idx(&self, addr1: u8, addr2: u8, idx: u8) -> u8 {
        let addr = self.get_abs_idx(addr1, addr2, idx);
        self.read(addr)
    }

    // write absolute byte given 2 operands of instruction and index
    pub fn write_abs_idx(&mut self, addr1: u8, addr2: u8, idx: u8, val: u8) {
        let addr = self.get_abs_idx(addr1, addr2, idx);
        self.write(addr, val);
    }

    // read from indexed indirect memory location given the operand and the
    // register values, the address contains the address to read from
    pub fn read_idx_ind(&self, addr1: u8, addr2: u8) -> u8 {
        let idx_addr = self.get_abs(addr1, addr2);
        let real_addr1 = self.read(idx_addr);
        let real_addr2 = self.read(idx_addr + 1);
        self.read_abs(real_addr1, real_addr2)
    }

    // write to the indexed indirect memory location given the operand and the
    // register values
    pub fn write_idx_ind(&mut self, addr: u8, reg: u8, val: u8) {
        let idx_addr = self.get_abs(addr, reg);
        let real_addr1 = self.read(idx_addr);
        let real_addr2 = self.read(idx_addr + 1);
        self.write_abs(real_addr1, real_addr2, val);
    }

    // read value indirect indexed memory location given the operand and the
    // register values
    pub fn read_ind_idx(&self, addr: u8, reg: u8) -> u8 {
        let ind_addr1 = self.read_zp(addr); // needs wrapping
        let ind_addr2 = self.read_zp(addr + 1);
        let ind_addr = self.get_abs(ind_addr1, ind_addr2);
        let idx_addr = ind_addr + reg as u16;
        self.read(idx_addr)
    }

    // write value to the indirect indexed memory location given the operand
    // and the register values
    pub fn write_ind_idx(&mut self, addr: u8, reg: u8, val: u8) {
        let ind_addr1 = self.read_zp(addr); // needs wrapping
        let ind_addr2 = self.read_zp(addr + 1);
        let ind_addr = self.get_abs(ind_addr1, ind_addr2);
        let idx_addr = ind_addr + reg as u16;
        self.write(idx_addr, val);
    }
}

impl Memory for CpuMemory {
    fn read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn write(&mut self, addr: u16, val: u8) {
        self.memory[addr as usize] = val;
    }
}

pub struct PpuMemory {
    memory: [u8; PPU_MEM_SIZE],
}

impl PpuMemory {
    pub fn new() -> Self {
        PpuMemory {
            memory: [0; PPU_MEM_SIZE],
        }
    }

    fn get_mirrored_addr(&self, addr: u16) -> u16 {
        addr % (PPU_MEM_SIZE as u16)
    }
}

impl Memory for PpuMemory {
    fn read(&self, addr: u16) -> u8 {
        self.memory[self.get_mirrored_addr(addr) as usize]
    }

    fn write(&mut self, addr: u16, val: u8) {
        self.memory[self.get_mirrored_addr(addr) as usize] = val;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_init() {
        let mem = CpuMemory::new();
        for i in 0..CPU_MEM_SIZE {
            assert_eq!(mem.memory[i], 0);
        }
    }

    #[test]
    fn test_read() {
        let mut mem = CpuMemory::new();

        for i in 0..CPU_MEM_SIZE {
            let val = (i % 0x100) as u8;
            mem.memory[i] = val;
            assert_eq!(mem.read(i as u16), val)
        }
    }

    #[test]
    fn test_write() {
        let mut mem = CpuMemory::new();

        for i in 0..CPU_MEM_SIZE {
            let val = (i % 0x100) as u8;
            mem.write(i as u16, val);
            assert_eq!(mem.memory[i], val);
        }
    }

    #[test]
    fn test_read_zp() {
        let mut mem = CpuMemory::new();

        for i in 0..CPU_MEM_PAGE_SIZE {
            let val = i as u8;
            mem.memory[i] = val;
            assert_eq!(mem.read_zp(val), val);
        }
    }

    #[test]
    fn test_write_zp() {
        let mut mem = CpuMemory::new();

        for i in 0..CPU_MEM_PAGE_SIZE {
            let val = i as u8;
            mem.write_zp(val, val);
            assert_eq!(mem.memory[i], val);
        }
    }

    #[test]
    fn test_read_zp_idx() {
        let mut mem = CpuMemory::new();

        for zp_addr in 0..CPU_MEM_PAGE_SIZE {
            for reg in 0..CPU_MEM_PAGE_SIZE {
                let real_addr = Wrapping(zp_addr as u8) + Wrapping(reg as u8);
                let val = reg as u8;

                mem.memory[real_addr.0 as usize] = val;
                assert_eq!(mem.read_zp_idx(zp_addr as u8, reg as u8), val);
            }
        }
    }

    #[test]
    fn test_write_zp_idx() {
        let mut mem = CpuMemory::new();

        for zp_addr in 0..CPU_MEM_PAGE_SIZE {
            for reg in 0..CPU_MEM_PAGE_SIZE {
                let real_addr = Wrapping(zp_addr as u8) + Wrapping(reg as u8);
                let val = reg as u8;

                mem.write_zp_idx(zp_addr as u8, reg as u8, val);
                assert_eq!(mem.memory[real_addr.0 as usize], val);
            }
        }
    }

    #[test]
    fn test_read_abs() {
        let mut mem = CpuMemory::new();

        for i in 0..CPU_MEM_SIZE {
            let addr1 = (i & 0x00FF) as u8;
            let addr2 = ((i & 0xFF00) >> 8) as u8;
            let val = i as u8;
            mem.memory[i] = val;
            assert_eq!(mem.read_abs(addr1, addr2), val);
        }
    }

    #[test]
    fn test_write_abs() {
        let mut mem = CpuMemory::new();

        for i in 0..CPU_MEM_SIZE {
            let addr1 = (i & 0x00FF) as u8;
            let addr2 = ((i & 0xFF00) >> 8) as u8;
            let val = i as u8;
            mem.write_abs(addr1, addr2, val);
            assert_eq!(mem.memory[i], val);
        }
    }

    fn get_abs_idx_reg_values() -> Vec<u8> {
        let mut reg_vals: Vec<u8> = Vec::new();
        reg_vals.push(0);
        reg_vals.push(32);
        reg_vals.push(64);
        reg_vals.push(128);
        reg_vals.push(255);
        reg_vals
    }

    #[test]
    fn test_read_abs_idx() {
        let mut mem = CpuMemory::new();

        let reg_vals = get_abs_idx_reg_values();
        for i in 0..CPU_MEM_SIZE {
            for j in 0..reg_vals.len() {
                let val = reg_vals[j];
                let addr = Wrapping(i as u16) + Wrapping(val as u16);
                mem.memory[addr.0 as usize] = val;

                assert_eq!(
                    mem.read_abs_idx((i & 0x00FF) as u8, ((i & 0xFF00) >> 8) as u8, val),
                    val
                );
            }
        }
    }

    #[test]
    fn test_write_abs_idx() {
        let mut mem = CpuMemory::new();

        let reg_vals = get_abs_idx_reg_values();
        for i in 0..CPU_MEM_SIZE {
            for j in 0..reg_vals.len() {
                let val = reg_vals[j];
                let addr = Wrapping(i as u16) + Wrapping(val as u16);
                mem.write_abs_idx((i & 0x00FF) as u8, ((i & 0xFF00) >> 8) as u8, val, val);

                assert_eq!(mem.memory[addr.0 as usize], val);
            }
        }
    }
}
