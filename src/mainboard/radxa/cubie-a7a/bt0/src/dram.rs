//! LPDDR5 bring-up through Allwinner's closed DRAM library.
//!
//! The library (`libdram-lpddr5.a`, ARMv7-A soft-float EABI) exports
//! `init_DRAM(int type, void *para)` and imports a handful of helpers that
//! we provide here with C linkage: `udelay`, `printf_dram`,
//! `set_ddr_voltage_ext`, `sunxi_smc_en_with_glitch_workaround`, plus
//! `memcpy`/`memset`/`__aeabi_*` which come from compiler_builtins.

use core::ffi::{c_char, c_int, c_void, CStr, VaList};

use util::mmio::{read32, write32};

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
/// (radxa/allwinner-device 6d911d15, sha256 d0764c8b..., the revision
/// radxa-pkg/u-boot-aw2501 builds boot0 from). The 4 GB sets are unchanged
/// at the newest device-a733-v1.4.6 tip (79d54f4), which retunes 8/12 GB to
/// 2040 MHz together with boot0 v0.6034; those are not taken here. Vendor boot0
/// picks one at run time (`select_mode = 3`: PH16/PH17 + GPADC channel 1).
/// SyterKit's single A7A set is not used: it mixes the CXMT and SK hynix
/// values and its tpr2 (0x0170b070) is the vendor value shifted by 4 bits.
pub static PARAM_SETS: [ParamSet; 6] = [
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
    // Last resort for 4 GB boards: [dram_para] of device-a733-v1.4.8
    // (Nov 2025 release, "stock DRAM 1800 MHz" per linux-sunxi). Not
    // selected by the strap; only tried if both 2400 MHz sets fail.
    lpddr5("4GB @1800 (v1.4.8)", 1800, 0xa10a, 0x1001, 0x12, 0x34, 0x86, 0x1108_0503, 0x325f_0000, 0x1_0065),
];
const CXMT_4G: usize = 0;
const HYNIX_4G: usize = 4;
const SAFE_4G: usize = 5;

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

/// Run the vendor DRAM init with `set`. For the 4 GB sets, retry with the
/// other vendor's set and then the conservative 1800 MHz set if training
/// fails. Returns the DRAM size in MiB and the clock in MHz of the set that
/// trained, or `None`.
/// The PMIC handed to [`init`], for whatever runs after DRAM.
pub fn pmic() -> Option<&'static Axp<'static>> {
    unsafe { (*core::ptr::addr_of!(PMIC)).as_ref() }
}

pub fn init(pmic: Option<Axp<'static>>, set: &'static ParamSet) -> Option<(u32, u32)> {
    unsafe { *core::ptr::addr_of_mut!(PMIC) = pmic };
    let fallback: &[&'static ParamSet] = if core::ptr::eq(set, &PARAM_SETS[CXMT_4G]) {
        &[&PARAM_SETS[HYNIX_4G], &PARAM_SETS[SAFE_4G]]
    } else if core::ptr::eq(set, &PARAM_SETS[HYNIX_4G]) {
        &[&PARAM_SETS[CXMT_4G], &PARAM_SETS[SAFE_4G]]
    } else {
        &[]
    };
    for s in core::iter::once(set).chain(fallback.iter().copied()) {
        println!("[bt0] DRAM: init with {} parameters ({} MHz)", s.name, s.para[0]);
        let size = unsafe {
            let para = core::ptr::addr_of_mut!(DRAM_PARA);
            *para = s.para;
            init_DRAM(0, para as *mut c_void)
        };
        if size > 0 {
            return Some((size as u32, s.para[0]));
        }
        println!("[bt0] DRAM: {} failed", s.name);
    }
    None
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

// `udelay` for the library goes through a trampoline that passes the
// caller's return address, so the log shows how far the library got: one
// line each time the calling site changes. The last site also goes to an
// RTC register, which survives the watchdog reset after a hang.
core::arch::global_asm!(
    ".section .text.udelay, \"ax\"",
    ".arm",
    ".global udelay",
    ".type udelay, %function",
    "udelay:",
    "mov r1, lr",
    "b {traced}",
    traced = sym udelay_traced,
);

/// RTC GP_DATA_REG4/5: last udelay caller and its time (ms since bt0) in
/// the DRAM library, kept across a watchdog reset.
pub const TRACE_LR_GPR: usize = 0x0709_0110;
pub const TRACE_MS_GPR: usize = 0x0709_0114;
const TRACE_MAX_LINES: u32 = 160;

static mut TRACE_LAST: u32 = 0;
static mut TRACE_REPEAT: u32 = 0;
static mut TRACE_LINES: u32 = 0;

extern "C" fn udelay_traced(us: u32, lr: u32) {
    let ms = (soc::ticks() / (soc::TIMER_HZ / 1000)) as u32;
    unsafe {
        let (last, repeat) = (TRACE_LAST, TRACE_REPEAT);
        if lr != last {
            if TRACE_LINES < TRACE_MAX_LINES {
                TRACE_LINES += 1;
                println!(
                    "[dram] {ms} ms: udelay({us}) from {lr:08x} (previous site {last:08x} x{})",
                    repeat + 1
                );
            }
            TRACE_LAST = lr;
            TRACE_REPEAT = 0;
        } else {
            TRACE_REPEAT += 1;
        }
    }
    write32(TRACE_LR_GPR, lr);
    write32(TRACE_MS_GPR, ms);
    soc::udelay(us as u64);
}

/// What the previous boot's trace registers held; call before [`init`].
pub fn report_previous_trace() {
    let (lr, ms) = (read32(TRACE_LR_GPR), read32(TRACE_MS_GPR));
    if lr != 0 {
        println!("[bt0] previous boot: last DRAM library udelay from {lr:08x} at {ms} ms");
    }
    write32(TRACE_LR_GPR, 0);
    write32(TRACE_MS_GPR, 0);
}

/// Clock, reset and PHY state around DRAM init. Only registers the library
/// itself touches before training, each read once.
pub fn dump_clocks(when: &str) {
    const REGS: [(&str, usize); 10] = [
        ("PLL_REF", 0x0200_2000),
        ("PLL_DDR", 0x0200_2020),
        ("PLL_DDR_PAT0", 0x0200_2028),
        ("NSI_CLK", 0x0200_2580),
        ("NSI_BGR", 0x0200_2584),
        ("MBUS_CLK", 0x0200_2588),
        ("DRAM_CLK", 0x0200_2c00),
        ("DRAM_BGR", 0x0200_2c0c),
        ("R_PRCM+250", 0x0701_0250),
        ("SYS+160", 0x0300_0160),
    ];
    print!("[dram] {when}:");
    for (name, addr) in REGS {
        print!(" {name}={:08x}", read32(addr));
    }
    println!();
}

// ---------------------------------------------------------------------------
// PHY firmware execution (replaces the library's
// dwc_ddrphy_phyinit_G_execFW, which the build weakens): the same register
// sequence, with a timeout and a report instead of an endless poll.

/// Synopsys PHY APB space at 0x0a90_0000, 16-bit registers at 2 * address.
const PHY_BASE: usize = 0x0a90_0000;
const fn phy(apb: usize) -> usize {
    PHY_BASE + 2 * apb
}
const MICRO_CONT_MUX_SEL: usize = phy(0xd0000);
const UCT_SHADOW_REGS: usize = phy(0xd0004);
const DCT_WRITE_PROT: usize = phy(0xd0031);
const UCT_WRITE_ONLY_SHADOW: usize = phy(0xd0032);
const D0033: usize = phy(0xd0033);
const UCT_DAT_WRITE_ONLY_SHADOW: usize = phy(0xd0034);
const MICRO_RESET: usize = phy(0xd0099);

/// Pstate 0 (2400 MHz) training takes ~3.7 s on the Cubie A7A.
const TRAIN_TIMEOUT_MS: u64 = 10_000;

fn w16(a: usize, v: u16) {
    util::mmio::write16(a, v);
}
fn r16(a: usize) -> u16 {
    util::mmio::read16(a)
}

fn phy_report(why: &str) {
    println!(
        "[dram] {why}: UctShadowRegs={:04x} UctWriteOnlyShadow={:04x} UctDat={:04x} MicroReset={:04x} MuxSel={:04x}",
        r16(UCT_SHADOW_REGS),
        r16(UCT_WRITE_ONLY_SHADOW),
        r16(UCT_DAT_WRITE_ONLY_SHADOW),
        r16(MICRO_RESET),
        r16(MICRO_CONT_MUX_SEL),
    );
    dump_clocks(why);
}

#[unsafe(no_mangle)]
pub extern "C" fn dwc_ddrphy_phyinit_G_execFW(ps: c_int) -> u32 {
    crate::recovery::kick();
    let t0 = soc::ticks();
    let ms = |t: u64| (t - t0) / (soc::TIMER_HZ / 1000);
    println!("[dram] PHY firmware: start (pstate loop index {ps})");
    w16(MICRO_CONT_MUX_SEL, 1);
    w16(MICRO_RESET, 9);
    w16(MICRO_RESET, 1);
    w16(MICRO_RESET, 0);
    w16(DCT_WRITE_PROT, 1);
    w16(D0033, 1);
    let mut next_report = 1000;
    while r16(UCT_SHADOW_REGS) & 1 != 0 {
        let now = ms(soc::ticks());
        if now >= TRAIN_TIMEOUT_MS {
            phy_report("PHY firmware: no mailbox message, giving up");
            w16(MICRO_RESET, 1);
            return 0xdead;
        }
        if now >= next_report {
            next_report += 1000;
            phy_report("PHY firmware: waiting");
        }
    }
    let mail = r16(UCT_WRITE_ONLY_SHADOW) as u32 | (r16(UCT_DAT_WRITE_ONLY_SHADOW) as u32) << 16;
    println!("Training result is = {mail:x} ({} ms)", ms(soc::ticks()));
    w16(DCT_WRITE_PROT, 0);
    let t1 = soc::ticks();
    while r16(UCT_SHADOW_REGS) == 0 {
        if ms(soc::ticks()) - ms(t1) > 1000 {
            phy_report("PHY firmware: mailbox ack timed out");
            break;
        }
    }
    w16(DCT_WRITE_PROT, 1);
    w16(MICRO_RESET, 1);
    mail
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

// ---------------------------------------------------------------------------
// Controller programming after PHY training (replaces the library's
// mctl_com_set_controller_after_phy, which the build weakens): the same
// writes, with a timeout and a report on the uMCTL2 quasi-dynamic
// programming handshake (SWCTL.sw_done / SWSTAT.sw_done_ack) instead of an
// endless poll.

/// uMCTL2 controller of channel 0; channel 1 is 4 MiB above.
const MCTL_CH0: usize = 0x0a11_0000;
const MCTL_CH_STRIDE: usize = 0x40_0000;
const SWCTL: usize = 0xc80;
const SWSTAT: usize = 0xc84;

fn mctl(ch: usize, off: usize) -> usize {
    MCTL_CH0 + ch * MCTL_CH_STRIDE + off
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn mctl_com_set_controller_after_phy(para: *const u32) {
    let tpr13 = unsafe { para.add(30).read_volatile() };
    let channels = if tpr13 & 0x40 != 0 { 2 } else { 1 };
    println!("[dram] controller after PHY: {channels} channel(s)");
    for ch in 0..channels {
        write32(mctl(ch, SWCTL), 0);
    }
    for ch in 0..channels {
        write32(mctl(ch, 0x208), 0);
    }
    for ch in 0..channels {
        let v = read32(mctl(ch, 0x510));
        write32(mctl(ch, 0x510), (v & !0xc000) | 0x4000);
    }
    for ch in 0..channels {
        write32(mctl(ch, SWCTL), 1);
    }
    for ch in 0..channels {
        let t0 = soc::ticks();
        while read32(mctl(ch, SWSTAT)) & 1 == 0 {
            if soc::ticks() - t0 > soc::TIMER_HZ {
                println!(
                    "[dram] channel {ch}: SWSTAT.sw_done_ack never set (SWSTAT {:08x}, SWCTL {:08x}, +0x510 {:08x})",
                    read32(mctl(ch, SWSTAT)),
                    read32(mctl(ch, SWCTL)),
                    read32(mctl(ch, 0x510)),
                );
                break;
            }
        }
    }
    println!("[dram] controller after PHY: handshake done");
    if channels == 2 {
        write32(0x0202_4818, 0x8002);
        write32(0x0202_481c, 0xffff_ffff);
        write32(0x0a03_0018, 0x8002);
        write32(0x0a04_0018, 0x8002);
        write32(0x0a03_0068, 0xffff_ffff);
        write32(0x0a04_0068, 0xffff_ffff);
        write32(0x0202_4838, 2);
        write32(0x0202_448c, 2);
        write32(0x0202_468c, 2);
        for off in [0x000, 0x200, 0x400, 0x600, 0x800] {
            write32(0x0202_3a04 + off, 0xffff_ffff);
        }
        write32(0x0202_4490, 0x1010_2010);
        write32(0x0202_4494, 0x0808_1008);
        write32(0x0202_4690, 0x1010_2010);
        write32(0x0202_4694, 0x0808_1008);
    } else {
        write32(0x0202_44a8, 1);
        write32(0x0202_46a8, 1);
        write32(0x0a03_0008, 3);
        write32(0x0a04_0008, 3);
    }
    println!("[dram] controller after PHY: done");
}

// ---------------------------------------------------------------------------
// LPDDR5 mode register access through the controller (replaces the
// library's lp5_sw_mr_wr / lp5_sw_mr_rd, which the build weakens): the same
// MRCTRL0/MRCTRL1/MRSTAT sequence, with timeouts and a log line per access.

const MRCTRL0: usize = 0x080;
const MRCTRL1: usize = 0x084;
const MRSTAT: usize = 0x090;
const MR_TIMEOUT_TICKS: u64 = soc::TIMER_HZ / 10;

/// Wait until `cond` holds; false (with a report) after 100 ms.
fn mr_wait(what: &str, ch: usize, mut cond: impl FnMut() -> bool) -> bool {
    let t0 = soc::ticks();
    while !cond() {
        if soc::ticks() - t0 > MR_TIMEOUT_TICKS {
            println!(
                "[dram] ch{ch}: {what} timed out (MRCTRL0 {:08x} MRSTAT {:08x})",
                read32(mctl(ch, MRCTRL0)),
                read32(mctl(ch, MRSTAT))
            );
            return false;
        }
    }
    true
}

fn mr_idle(ch: usize) -> bool {
    mr_wait("MRSTAT.mr_wr_busy", ch, || read32(mctl(ch, MRSTAT)) & 1 == 0)
}

fn mr_done(ch: usize) -> bool {
    mr_wait("MRCTRL0.mr_wr", ch, || read32(mctl(ch, MRCTRL0)) & (1 << 31) == 0)
}

/// Write MR `mr` of both ranks of channel `ch` (rank 0 gets `v0`, rank 1 `v1`).
#[unsafe(no_mangle)]
pub extern "C" fn lp5_sw_mr_wr(ch: u32, mr: u32, v0: u32, v1: u32) {
    let ch = ch as usize;
    println!("[dram] ch{ch}: MR{mr} <- {v0:#x}/{v1:#x}");
    for (rank_bit, v) in [(0x10, v0), (0x20, v1)] {
        mr_idle(ch);
        write32(mctl(ch, MRCTRL0), rank_bit);
        write32(mctl(ch, MRCTRL1), (mr << 8) | v);
        write32(mctl(ch, MRCTRL0), (1 << 31) | rank_bit);
        mr_done(ch);
    }
}

/// Read MR `mr` of rank `rank` of channel `ch`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn lp5_sw_mr_rd(para: *const u32, ch: u32, rank: u32, mr: u32) -> u32 {
    let ty = unsafe { para.add(1).read_volatile() };
    let c = ch as usize;
    let rank_bit = 1u32 << (rank + 4);
    mr_idle(c);
    write32(mctl(c, MRCTRL0), rank_bit | 1);
    write32(mctl(c, MRCTRL1), mr << 8);
    write32(mctl(c, MRCTRL0), (1 << 31) | rank_bit | 1);
    mr_done(c);
    let v = if ty == 8 && ch == 1 {
        read32(0x0a02_0040) & 0xff
    } else if ch == 0 {
        (read32(0x0a02_002c) >> 8) & 0xff
    } else {
        (read32(0x0a02_0040) >> 8) & 0xff
    };
    println!("[dram] ch{ch} rank{rank}: MR{mr} = {v:#04x}");
    v
}
