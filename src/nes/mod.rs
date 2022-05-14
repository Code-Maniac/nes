mod addressing;
mod audio;
mod memory;
mod processors;
mod video;

pub fn start() {
    let cpu_mem = memory::CpuMemory::new();
    let cpu = processors::Ricoh2A03::new(1.0, &cpu_mem);
}
