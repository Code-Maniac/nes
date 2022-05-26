use std::num::Wrapping;

// constants defining the cpu memory map
pub const CPU_MEM_SIZE: usize = 0x10000;
pub const CPU_MEM_PAGE_SIZE: usize = 0x100;
pub const CPU_ADDR_STACK: usize = 0x0100;
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
    /// read from memory. Apply timing to the clock
    fn read(&self, addr: u16) -> u8;
    /// write to memory. Apply timing to the clock
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
    fn get_zpg_x(&self, addr: u8, idx: u8) -> u16 {
        let real_addr = Wrapping(addr) + Wrapping(idx);
        real_addr.0 as u16
    }

    // calculate absolute addr from the 2 operands
    fn get_abs(&self, addr1: u8, addr2: u8) -> u16 {
        //NOTE: add wrapping
        (addr1 as u16) | (addr2 as u16) << 8
    }

    // calculate the indexed absolute addr form the 2 operands + index
    fn get_abs_x(&self, addr1: u8, addr2: u8, idx: u8) -> u16 {
        let abs_addr = self.get_abs(addr1, addr2);
        let abs_idx = Wrapping(abs_addr) + Wrapping(idx as u16);

        abs_idx.0
    }

    // return the opcode given the pc. 0 reads from beggining of PRG_ROM
    pub fn read_opcode(&self, pc: u16) -> u8 {
        let addr = Wrapping(CPU_ADDR_PRG_ROM as u16) + Wrapping(pc);
        self.memory[addr.0 as usize]
    }

    // read first arg of opcode. Given pc
    pub fn read_opcode_aa(&self, pc: u16) -> u8 {
        let addr = Wrapping(CPU_ADDR_PRG_ROM as u16) + Wrapping(pc) + Wrapping(1);
        self.memory[addr.0 as usize]
    }

    pub fn read_opcode_bb(&self, pc: u16) -> u8 {
        let addr = Wrapping(CPU_ADDR_PRG_ROM as u16) + Wrapping(pc) + Wrapping(2);
        self.memory[addr.0 as usize]
    }

    // read byte from the addr in zero page
    pub fn read_zpg(&self, addr: u8) -> u8 {
        self.read(addr as u16)
    }

    // write byte to addr in zero page
    pub fn write_zpg(&mut self, addr: u8, val: u8) {
        self.write(addr as u16, val);
    }

    pub fn modify_zpg<F>(&mut self, f: &mut F, addr: u8)
    where
        F: FnMut(u8) -> u8,
    {
        self.write_zpg(addr, f(self.memory[addr as usize]));
    }

    // read byte from indexed address in zero page
    pub fn read_zpg_x(&self, addr: u8, idx: u8) -> u8 {
        self.read(self.get_zpg_x(addr, idx))
    }

    // write byte to indexed address in zero page
    pub fn write_zpg_x(&mut self, addr: u8, idx: u8, val: u8) {
        self.write(self.get_zpg_x(addr, val), val);
    }

    pub fn modify_zpg_x<F>(&mut self, f: F, addr: u8, idx: u8)
    where
        F: Fn(u8) -> u8,
    {
        let val = self.memory[self.get_zpg_x(addr, idx) as usize];
        self.write_zpg_x(addr, idx, f(val));
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

    pub fn modify_abs<F>(&mut self, f: F, addr1: u8, addr2: u8)
    where
        F: Fn(u8) -> u8,
    {
        let val = self.memory[self.get_abs(addr1, addr2) as usize];
        self.write_abs(addr1, addr2, f(val));
    }

    // read absolute byte given 2 operands of instruction and index
    pub fn read_abs_x(&self, addr1: u8, addr2: u8, idx: u8) -> u8 {
        self.read_abs_x_free(addr1, addr2, idx)
    }

    // read absolute byte given 2 operands of instruction and index
    // ignore page boundary crossing for timings
    pub fn read_abs_x_free(&self, addr1: u8, addr2: u8, idx: u8) -> u8 {
        let addr = self.get_abs_x(addr1, addr2, idx);
        self.read(addr)
    }

    // write absolute byte given 2 operands of instruction and index
    pub fn write_abs_x(&mut self, addr1: u8, addr2: u8, idx: u8, val: u8) {
        self.write_abs_x_free(addr1, addr2, idx, val);
    }

    // write absolute byte given 2 operands of instruction and index
    // ignore page boundary crossing for timings
    pub fn write_abs_x_free(&mut self, addr1: u8, addr2: u8, idx: u8, val: u8) {
        let addr = self.get_abs_x(addr1, addr2, idx);
        self.write(addr, val);
    }

    // for use with JMP ind
    pub fn read_ind(&self, pc: u16) -> u16 {
        let xx = self.read_opcode_aa(pc);
        let yy = self.read_opcode_bb(pc + 1);

        (xx as u16) | ((yy as u16) << 8)
    }

    // read from indexed indirect memory location given the operand and the
    // register values, the address contains the address to read from
    pub fn read_ind_x(&self, addr1: u8, addr2: u8) -> u8 {
        let idx_addr = self.get_abs(addr1, addr2);
        let real_addr1 = self.read(idx_addr);
        let real_addr2 = self.read(idx_addr + 1);
        self.read_abs(real_addr1, real_addr2)
    }

    // write to the indexed indirect memory location given the operand and the
    // register values
    pub fn write_ind_x(&mut self, addr: u8, reg: u8, val: u8) {
        let idx_addr = self.get_abs(addr, reg);
        let real_addr1 = self.read(idx_addr);
        let real_addr2 = self.read(idx_addr + 1);
        self.write_abs(real_addr1, real_addr2, val);
    }

    pub fn modify_ind_x<F>(&mut self, f: F, addr: u8, reg: u8)
    where
        F: Fn(u8) -> u8,
    {
        let idx_addr = self.get_abs(addr, reg);
        let real_addr1 = self.read(idx_addr);
        let real_addr2 = self.read(idx_addr + 1);
        let val = self.memory[self.get_abs(real_addr1, real_addr2) as usize];

        self.write_ind_x(addr, reg, f(val));
    }

    // read value indirect indexed memory location given the operand and the
    // register values
    pub fn read_ind_y(&self, addr: u8, reg: u8) -> u8 {
        self.read_ind_y_free(addr, reg)
    }

    // read value indirect indexed memory location given the operand and the
    // register values
    // ignore page boundary crossing for timings
    pub fn read_ind_y_free(&self, addr: u8, reg: u8) -> u8 {
        let ind_addr1 = self.read_zpg(addr); // needs wrapping
        let ind_addr2 = self.read_zpg(addr + 1);
        let ind_addr = self.get_abs(ind_addr1, ind_addr2);
        let idx_addr = ind_addr + reg as u16;
        self.read(idx_addr)
    }

    // write value to the indirect indexed memory location given the operand
    // and the register values
    pub fn write_ind_y(&mut self, addr: u8, reg: u8, val: u8) {
        self.write_ind_y_free(addr, reg, val)
    }

    // write value to the indirect indexed memory location given the operand
    // and the register values
    // ignore page boundary crossing for timings
    pub fn write_ind_y_free(&mut self, addr: u8, reg: u8, val: u8) {
        let ind_addr1 = self.read_zpg(addr); // needs wrapping
        let ind_addr2 = self.read_zpg(addr + 1);
        let ind_addr = self.get_abs(ind_addr1, ind_addr2);
        let idx_addr = ind_addr + reg as u16;
        self.write(idx_addr, val);
    }

    // read byte from the location given the stack pointer value
    pub fn read_stack(&self, sp: u8) -> u8 {
        self.memory[CPU_ADDR_STACK + sp as usize]
    }

    // write byte to the location given the stack pointer value and value
    pub fn write_stack(&mut self, sp: u8, val: u8) {
        self.memory[CPU_ADDR_STACK + sp as usize] = val;
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
    fn test_read_zpg() {
        let mut mem = CpuMemory::new();

        for i in 0..CPU_MEM_PAGE_SIZE {
            let val = i as u8;
            mem.memory[i] = val;
            assert_eq!(mem.read_zpg(val), val);
        }
    }

    #[test]
    fn test_write_zpg() {
        let mut mem = CpuMemory::new();

        for i in 0..CPU_MEM_PAGE_SIZE {
            let val = i as u8;
            mem.write_zpg(val, val);
            assert_eq!(mem.memory[i], val);
        }
    }

    #[test]
    fn test_read_zpg_x() {
        let mut mem = CpuMemory::new();

        for zp_addr in 0..CPU_MEM_PAGE_SIZE {
            for reg in 0..CPU_MEM_PAGE_SIZE {
                let real_addr = Wrapping(zp_addr as u8) + Wrapping(reg as u8);
                let val = reg as u8;

                mem.memory[real_addr.0 as usize] = val;
                assert_eq!(mem.read_zpg_x(zp_addr as u8, reg as u8), val);
            }
        }
    }

    #[test]
    fn test_write_zpg_x() {
        let mut mem = CpuMemory::new();

        for zp_addr in 0..CPU_MEM_PAGE_SIZE {
            for reg in 0..CPU_MEM_PAGE_SIZE {
                let real_addr = Wrapping(zp_addr as u8) + Wrapping(reg as u8);
                let val = reg as u8;

                mem.write_zpg_x(zp_addr as u8, reg as u8, val);
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

    fn get_abs_x_reg_values() -> Vec<u8> {
        let mut reg_vals: Vec<u8> = Vec::new();
        reg_vals.push(0);
        reg_vals.push(32);
        reg_vals.push(64);
        reg_vals.push(128);
        reg_vals.push(255);
        reg_vals
    }

    #[test]
    fn test_read_abs_x() {
        let mut mem = CpuMemory::new();

        let reg_vals = get_abs_x_reg_values();
        for i in 0..CPU_MEM_SIZE {
            for j in 0..reg_vals.len() {
                let val = reg_vals[j];
                let addr = Wrapping(i as u16) + Wrapping(val as u16);
                mem.memory[addr.0 as usize] = val;

                assert_eq!(
                    mem.read_abs_x((i & 0x00FF) as u8, ((i & 0xFF00) >> 8) as u8, val),
                    val
                );
            }
        }
    }

    #[test]
    fn test_write_abs_idx() {
        let mut mem = CpuMemory::new();

        let reg_vals = get_abs_x_reg_values();
        for i in 0..CPU_MEM_SIZE {
            for j in 0..reg_vals.len() {
                let val = reg_vals[j];
                let addr = Wrapping(i as u16) + Wrapping(val as u16);
                mem.write_abs_x((i & 0x00FF) as u8, ((i & 0xFF00) >> 8) as u8, val, val);

                assert_eq!(mem.memory[addr.0 as usize], val);
            }
        }
    }
}
