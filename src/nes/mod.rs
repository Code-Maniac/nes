mod addressing;
mod audio;
mod memory;
mod opcodes;
mod processors;
mod video;

pub fn start() {
    let mut cpu_mem = memory::CpuMemory::new();
    let cpu = processors::Ricoh2A03::new(1.0, &mut cpu_mem);
}
