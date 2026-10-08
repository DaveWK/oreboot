//! The Cubie A7A's DRAM parameter sets (vendor parameters).
//!
//! From Radxa's `configs/cubie_a7a/sys_config.fex` (radxa/allwinner-device
//! 6d911d15, sha256 d0764c8b..., the revision radxa-pkg/u-boot-aw2501 builds
//! boot0 from). The 4 GB sets are unchanged at the newest device-a733-v1.4.6
//! tip (79d54f4), which retunes 8/12 GB to 2040 MHz together with boot0
//! v0.6034; those are not taken here. Vendor boot0 picks one at run time
//! (`select_mode = 3`: PH16/PH17 + GPADC channel 1).

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
            clk,
            9,
            0x0e0e_0e0e,
            0x0f0f_0f0f,
            0xec03_0e0f, // clk, LPDDR5, odt/drive
            0,
            para1,
            para2, // para0..2
            0,
            0,
            0,
            0x6,
            0,
            0,
            0, // mr0..mr6
            mr11,
            0x44,
            0,
            mr14,
            0,
            mr17,
            0, // mr11, mr12, mr13, mr14, mr16, mr17, mr22
            0x4040,
            0,
            tpr2,
            0x3800,
            0x3514, // tpr0..3, tpr6
            tpr10,
            0,
            0,
            tpr13,
            0, // tpr10..14
        ],
    }
}

pub static PARAM_SETS: [ParamSet; 6] = [
    // [dram_para]  "cxmt 4g ddr"
    lpddr5(
        "CXMT 4GB",
        2400,
        0xa0fa,
        0x1001,
        0x13,
        0x2e,
        0x06,
        0x170b_0703,
        0x325f_0000,
        0x65,
    ),
    // [dram_para1] "cxmt 6g ddr"
    lpddr5(
        "CXMT 6GB",
        2400,
        0xa10a,
        0x1001,
        0x13,
        0x2e,
        0x06,
        0x170b_0703,
        0x325f_0000,
        0x1_0065,
    ),
    // [dram_para2] "cxmt 8g ddr"
    lpddr5(
        "CXMT 8GB",
        1800,
        0xa10a,
        0x1001,
        0x1b,
        0x1d,
        0x86,
        0x1108_0503,
        0xb25f_0000,
        0x65,
    ),
    // [dram_para3] "cxmt 12g ddr"
    lpddr5(
        "CXMT 12GB",
        1800,
        0xa11a,
        0x1001,
        0x12,
        0x34,
        0x86,
        0x1108_0503,
        0xb25f_0000,
        0x1_0065,
    ),
    // [dram_para7] "skhnix 4g ddr"
    lpddr5(
        "SK hynix 4GB",
        2400,
        0xa10a,
        0x0001,
        0x12,
        0x34,
        0x06,
        0x170b_0703,
        0x325f_0000,
        0x65,
    ),
    // Last resort for 4 GB boards: [dram_para] of device-a733-v1.4.8
    // (Nov 2025 release, "stock DRAM 1800 MHz" per linux-sunxi). Not
    // selected by the strap; only tried if both 2400 MHz sets fail.
    lpddr5(
        "4GB @1800 (v1.4.8)",
        1800,
        0xa10a,
        0x1001,
        0x12,
        0x34,
        0x86,
        0x1108_0503,
        0x325f_0000,
        0x1_0065,
    ),
];
pub const CXMT_4G: usize = 0;
pub const HYNIX_4G: usize = 4;
pub const SAFE_4G: usize = 5;
