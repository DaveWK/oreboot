//! LPDDR5 bring-up through Allwinner's closed DRAM library.
//!
//! The library (`libdram-lpddr5.a`, ARMv7-A soft-float EABI) exports
//! `init_DRAM(int type, void *para)` and imports a handful of helpers that
//! we provide here with C linkage: `udelay`, `printf_dram`,
//! `set_ddr_voltage_ext`, `sunxi_smc_en_with_glitch_workaround`, plus
//! `memcpy`/`memset`/`__aeabi_*` which come from compiler_builtins.

use core::ffi::{c_char, c_int, c_void, CStr, VaList};

use crate::axp::Axp;
use crate::soc;

unsafe extern "C" {
    fn init_DRAM(ty: c_int, para: *mut c_void) -> c_int;
}

/// One `dram_para_t` as the vendor library expects it: 32 words in
/// `sys_config.fex` key order (clk, type, dx_odt, dx_dri, ca_dri, para0..2,
/// mr0..6, mr11, mr12, mr13, mr14, mr16, mr17, mr22, tpr0..3, tpr6,
/// tpr10..14).
pub struct ParamSet {
    pub name: &'static str,
    pub para: [u32; 32],
}

/// Build a Cubie A7A LPDDR5 set from the words that differ between them.
#[allow(clippy::too_many_arguments)]
const fn lpddr5(
    name: &'static str,
    clk: u32,
    para1: u32,
    para2: u32,
    mr11: u32,
    mr14: u32,
    mr17: u32,
    tpr2: u32,
    tpr10: u32,
    tpr13: u32,
) -> ParamSet {
    ParamSet {
        name,
        para: [
            clk, 9, 0x0e0e_0e0e, 0x0f0f_0f0f, 0xec03_0e0f, // clk, LPDDR5, odt/drive
            0, para1, para2, // para0..2
            0, 0, 0, 0x6, 0, 0, 0, // mr0..mr6
            mr11, 0x44, 0, mr14, 0, mr17, 0, // mr11, mr12, mr13, mr14, mr16, mr17, mr22
            0x4040, 0, tpr2, 0x3800, 0x3514, // tpr0..3, tpr6
            tpr10, 0, 0, tpr13, 0, // tpr10..14
        ],
    }
}

/// The Cubie A7A DRAM sets from Radxa's `configs/cubie_a7a/sys_config.fex`
/// (radxa/allwinner-device 6d911d15, sha256 d0764c8b...). Vendor boot0
/// picks one at run time (`select_mode = 3`: PH16/PH17 + GPADC channel 1).
/// SyterKit's single A7A set is not used: it mixes the CXMT and SK hynix
/// values and its tpr2 (0x0170b070) is the vendor value shifted by 4 bits.
pub static PARAM_SETS: [ParamSet; 5] = [
    // [dram_para]  "cxmt 4g ddr"
    lpddr5("CXMT 4GB", 2400, 0xa0fa, 0x1001, 0x13, 0x2e, 0x06, 0x170b_0703, 0x325f_0000, 0x65),
    // [dram_para1] "cxmt 6g ddr"
    lpddr5("CXMT 6GB", 2400, 0xa10a, 0x1001, 0x13, 0x2e, 0x06, 0x170b_0703, 0x325f_0000, 0x1_0065),
    // [dram_para2] "cxmt 8g ddr"
    lpddr5("CXMT 8GB", 1800, 0xa10a, 0x1001, 0x1b, 0x1d, 0x86, 0x1108_0503, 0xb25f_0000, 0x65),
    // [dram_para3] "cxmt 12g ddr"
    lpddr5("CXMT 12GB", 1800, 0xa11a, 0x1001, 0x12, 0x34, 0x86, 0x1108_0503, 0xb25f_0000, 0x1_0065),
    // [dram_para7] "skhnix 4g ddr"
    lpddr5("SK hynix 4GB", 2400, 0xa10a, 0x0001, 0x12, 0x34, 0x06, 0x170b_0703, 0x325f_0000, 0x65),
];
const CXMT_4G: usize = 0;
const HYNIX_4G: usize = 4;

/// DDR-PARA-SEL-ADC strap voltages from the A7A V1.10 schematic (sheet 6)
/// and the set each one selects. The SK hynix 4 GB board leaves the
/// divider unpopulated (pulled up to 1.8 V); 1569 mV is "DDR PARA 7".
const ADC_STRAPS: [(u32, usize); 6] = [
    (163, CXMT_4G),
    (382, 1),
    (608, 2),
    (811, 3),
    (1569, HYNIX_4G),
    (1800, HYNIX_4G),
];
const DDR_SEL_ADC_CHANNEL: u32 = 1;

/// Choose the parameter set like vendor boot0: read the PH16/PH17 strap
/// GPIOs (both 0 on every A7A variant in the schematic) and the GPADC
/// strap voltage, and take the nearest table entry.
pub fn select() -> &'static ParamSet {
    use crate::gpio::{self, Port, Pull};
    for pin in [16, 17] {
        gpio::set_function(Port::H, pin, 0);
        gpio::set_pull(Port::H, pin, Pull::Up);
    }
    soc::udelay(100);
    let gpios = (gpio::read(Port::H, 17) as u32) << 1 | gpio::read(Port::H, 16) as u32;
    for pin in [16, 17] {
        gpio::set_pull(Port::H, pin, Pull::None);
    }
    if gpios != 0 {
        println!("[bt0] DRAM: strap GPIOs PH17:PH16 = {gpios:02b}, not an A7A set; using the ADC only");
    }
    let Some(mv) = crate::gpadc::read_mv(DDR_SEL_ADC_CHANNEL) else {
        println!("[bt0] DRAM: GPADC did not convert; assuming {}", PARAM_SETS[CXMT_4G].name);
        return &PARAM_SETS[CXMT_4G];
    };
    let &(strap, idx) = ADC_STRAPS.iter().min_by_key(|(s, _)| s.abs_diff(mv)).unwrap();
    println!("[bt0] DRAM: strap {mv} mV (nearest {strap} mV) -> {}", PARAM_SETS[idx].name);
    &PARAM_SETS[idx]
}

/// Working copy handed to the library, which writes back into it.
static mut DRAM_PARA: [u32; 32] = [0; 32];

static mut PMIC: Option<Axp<'static>> = None;

/// Run the vendor DRAM init with `set`, retrying with the other 4 GB set
/// if training fails. Returns the DRAM size in MiB (0 on failure).
pub fn init(pmic: Option<Axp<'static>>, set: &'static ParamSet) -> u32 {
    unsafe { *core::ptr::addr_of_mut!(PMIC) = pmic };
    let fallback = if core::ptr::eq(set, &PARAM_SETS[CXMT_4G]) {
        Some(&PARAM_SETS[HYNIX_4G])
    } else if core::ptr::eq(set, &PARAM_SETS[HYNIX_4G]) {
        Some(&PARAM_SETS[CXMT_4G])
    } else {
        None
    };
    for s in core::iter::once(set).chain(fallback) {
        println!("[bt0] DRAM: init with {} parameters ({} MHz)", s.name, s.para[0]);
        let size = unsafe {
            let para = core::ptr::addr_of_mut!(DRAM_PARA);
            *para = s.para;
            init_DRAM(0, para as *mut c_void)
        };
        if size > 0 {
            return size as u32;
        }
        println!("[bt0] DRAM: {} failed", s.name);
    }
    0
}

/// Quick sanity check: unique pattern at power-of-two strides, then read
/// back, catching dead or aliased address lines.
pub fn test(base: usize, size: usize) -> Result<(), (usize, u32, u32)> {
    let pattern = |a: usize| (a as u32) ^ 0x5a5a_a5a5;
    let mut off = 0x1000;
    let mut addrs = [0usize; 32];
    let mut n = 0;
    addrs[n] = base;
    n += 1;
    while off < size && n < addrs.len() {
        addrs[n] = base + off - 4;
        n += 1;
        off <<= 1;
    }
    for &a in &addrs[..n] {
        util::mmio::write32(a, pattern(a));
    }
    for &a in &addrs[..n] {
        let v = util::mmio::read32(a);
        if v != pattern(a) {
            return Err((a, pattern(a), v));
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Imports of the vendor library.

#[unsafe(no_mangle)]
pub extern "C" fn udelay(us: u32) {
    soc::udelay(us as u64);
}

#[unsafe(no_mangle)]
pub extern "C" fn sunxi_smc_en_with_glitch_workaround() {}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn set_ddr_voltage_ext(name: *const c_char, mv: c_int, on: c_int) -> c_int {
    let name = unsafe { CStr::from_ptr(name) }.to_str().unwrap_or("?");
    let on = match on {
        o if o < 0 => None,
        0 => Some(false),
        _ => Some(true),
    };
    println!("[bt0] DRAM: set {name} to {mv} mV, on={on:?}");
    match unsafe { (*core::ptr::addr_of!(PMIC)).as_ref() } {
        Some(pmic) => match pmic.set_voltage(name, mv.max(0) as u32, on) {
            Ok(()) => 0,
            Err(e) => {
                println!("[bt0] DRAM: {name}: {e}");
                -1
            }
        },
        None => {
            println!("[bt0] DRAM: no PMIC, voltage request ignored");
            0
        }
    }
}

/// The official Tina SDK library (dramlib sun60iw2p1/spl_libdram/libdram)
/// imports `printf`; SyterKit's repackaged copy renames it `printf_dram`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn printf(fmt: *const c_char, mut args: ...) -> c_int {
    let fmt = unsafe { CStr::from_ptr(fmt) }.to_bytes();
    let mut out = LineBuf::new();
    unsafe { format(&mut out, fmt, &mut args) };
    out.flush();
    0
}

/// Minimal printf for the vendor library's log output: %d %i %u %x %X %p
/// %s %c %%, with optional `-`/`0` flags, width and `l`/`ll`/`h`/`z`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn printf_dram(fmt: *const c_char, mut args: ...) -> c_int {
    let fmt = unsafe { CStr::from_ptr(fmt) }.to_bytes();
    let mut out = LineBuf::new();
    unsafe { format(&mut out, fmt, &mut args) };
    out.flush();
    0
}

struct LineBuf {
    buf: [u8; 128],
    len: usize,
}

impl LineBuf {
    fn new() -> Self {
        Self {
            buf: [0; 128],
            len: 0,
        }
    }

    fn push(&mut self, b: u8) {
        if self.len == self.buf.len() {
            self.flush();
        }
        self.buf[self.len] = b;
        self.len += 1;
    }

    fn flush(&mut self) {
        let s = core::str::from_utf8(&self.buf[..self.len]).unwrap_or("<non-utf8>");
        print!("{s}");
        self.len = 0;
    }
}

unsafe fn format(out: &mut LineBuf, fmt: &[u8], args: &mut VaList) {
    let mut i = 0;
    while i < fmt.len() {
        let c = fmt[i];
        i += 1;
        if c != b'%' {
            out.push(c);
            continue;
        }
        let mut left = false;
        let mut zero = false;
        while i < fmt.len() && (fmt[i] == b'-' || fmt[i] == b'0') {
            left |= fmt[i] == b'-';
            zero |= fmt[i] == b'0';
            i += 1;
        }
        let mut width = 0usize;
        while i < fmt.len() && fmt[i].is_ascii_digit() {
            width = width * 10 + (fmt[i] - b'0') as usize;
            i += 1;
        }
        let mut long_long = false;
        while i < fmt.len() && matches!(fmt[i], b'l' | b'h' | b'z') {
            if fmt[i] == b'l' && i + 1 < fmt.len() && fmt[i + 1] == b'l' {
                long_long = true;
                i += 1;
            }
            i += 1;
        }
        let Some(&conv) = fmt.get(i) else { break };
        i += 1;

        let mut digits = [0u8; 24];
        let text: &[u8] = match conv {
            b'%' => b"%",
            b'c' => {
                digits[0] = (unsafe { args.next_arg::<c_int>() }) as u8;
                &digits[..1]
            }
            b's' => {
                let p = unsafe { args.next_arg::<*const c_char>() };
                if p.is_null() {
                    b"(null)"
                } else {
                    unsafe { CStr::from_ptr(p) }.to_bytes()
                }
            }
            b'd' | b'i' => {
                let v = if long_long {
                    unsafe { args.next_arg::<i64>() }
                } else {
                    (unsafe { args.next_arg::<i32>() }) as i64
                };
                let n = utoa(v.unsigned_abs(), 10, false, &mut digits[1..]);
                if v < 0 {
                    let start = digits.len() - n - 1;
                    digits[start] = b'-';
                    &digits[start..]
                } else {
                    &digits[digits.len() - n..]
                }
            }
            b'u' | b'x' | b'X' | b'p' => {
                let v = if long_long {
                    unsafe { args.next_arg::<u64>() }
                } else {
                    (unsafe { args.next_arg::<u32>() }) as u64
                };
                let base = if conv == b'u' { 10 } else { 16 };
                let n = utoa(v, base, conv == b'X', &mut digits);
                &digits[digits.len() - n..]
            }
            _ => {
                out.push(b'%');
                out.push(conv);
                continue;
            }
        };
        let pad = width.saturating_sub(text.len());
        if !left {
            for _ in 0..pad {
                out.push(if zero { b'0' } else { b' ' });
            }
        }
        for &b in text {
            out.push(b);
        }
        if left {
            for _ in 0..pad {
                out.push(b' ');
            }
        }
    }
}

/// Write `v` right-aligned into `buf`, returning the number of digits.
fn utoa(mut v: u64, base: u64, upper: bool, buf: &mut [u8]) -> usize {
    let hex = if upper {
        b"0123456789ABCDEF"
    } else {
        b"0123456789abcdef"
    };
    let mut n = 0;
    loop {
        n += 1;
        let idx = buf.len() - n;
        buf[idx] = hex[(v % base) as usize];
        v /= base;
        if v == 0 {
            break;
        }
    }
    n
}
