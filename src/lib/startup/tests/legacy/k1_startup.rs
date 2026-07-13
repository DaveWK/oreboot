// Verbatim copy of ore-edk-boot-opi 78479e21ba oreboot/src/mainboard/spacemit/k1x/bt0/src/startup.rs, for the
// equivalence test; its //! header line is // here (the file is include!d).

// Linker-delimited startup operations, independently exercised on poisoned RAM.
/// Initialize BSS and initialized data. The live stack is in a distinct linker section.
/// # Safety
/// All ranges must be valid, ordered, and BSS/data must exclude the live stack.
/// Data is either loaded in place or has a nonoverlapping load address.
pub unsafe fn initialize(
    sbss: *mut u8,
    ebss: *mut u8,
    sdata: *mut u8,
    edata: *mut u8,
    sidata: *const u8,
) {
    core::ptr::write_bytes(sbss, 0, ebss as usize - sbss as usize);
    // In-place .data is already initialized by the ROM image copy. Copying it
    // to itself with copy_nonoverlapping would be undefined behaviour.
    // Linker symbols may alias even though LLVM treats distinct extern statics
    // as distinct objects. Force comparison of their resolved machine addresses;
    // otherwise LTO can remove this guard and emit a self-copy anyway.
    let source = core::hint::black_box(sidata as usize);
    let destination = core::hint::black_box(sdata as usize);
    if source != destination {
        core::ptr::copy_nonoverlapping(sidata, sdata, edata as usize - sdata as usize);
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn poisoned_bss_excludes_stack_and_initialized_data() {
        let mut memory = [0xa5u8; 128];
        let initial = [0x5au8; 16];
        unsafe {
            let p = memory.as_mut_ptr();
            initialize(
                p.add(16),
                p.add(48),
                p.add(96),
                p.add(112),
                initial.as_ptr(),
            );
        }
        assert_eq!(&memory[..16], &[0xa5; 16]);
        assert_eq!(&memory[16..48], &[0; 32]);
        assert_eq!(&memory[48..96], &[0xa5; 48]);
        assert_eq!(&memory[96..112], &initial);
        assert_eq!(&memory[112..], &[0xa5; 16]);
    }
    #[test]
    fn in_place_and_empty_data() {
        let mut m = [0xa5; 16];
        unsafe {
            let p = m.as_mut_ptr();
            initialize(p, p.add(8), p.add(8), p.add(16), p.add(8));
        }
        assert_eq!(
            m,
            [0, 0, 0, 0, 0, 0, 0, 0, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5]
        );
    }
}
