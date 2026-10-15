use log::{print, println};
use util::mem::dump_block;
use util::mmio::{read32, write32};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Timeout {
        stage: &'static str,
        address: usize,
        last: u32,
    },
    Training(u32),
    Geometry,
    Rank,
}
type Result<T> = core::result::Result<T, Error>;

// A finite poll budget also stops a broken/stalled timer from extending a wait.
const POLL_LIMIT: u32 = 10_000_000;
fn wait_mask(address: usize, mask: u32, expected: u32, stage: &'static str) -> Result<()> {
    poll(|| read32(address), mask, expected, POLL_LIMIT).map_err(|last| Error::Timeout {
        stage,
        address,
        last,
    })
}
fn poll(
    mut read: impl FnMut() -> u32,
    mask: u32,
    expected: u32,
    limit: u32,
) -> core::result::Result<(), u32> {
    let mut last = 0;
    for _ in 0..limit {
        last = read();
        if last & mask == expected {
            return Ok(());
        }
    }
    Err(last)
}

// U-Boot  drivers/ddr/spacemit/k1x/lpddr4_silicon_init.c (SpacemiT uboot-2022.10
// d0e22fd5b0, GPL-2.0+; see doc/K1-DRAM-PROVENANCE.md in ore-edk-boot-opi)

const DDR_TRAINING_INFO: usize = 0xC080_0000;

// data rate in mega transfers, currently hardcoded
pub const DATA_RATE: u32 = 2400;
// DRAM ranks when the board EEPROM has no TLV_CODE_DDR_CSNUM (U-Boot SPL
// falls back to its DT the same way). Set per board at build time with
// K1_CS_NUM=1 or 2; the OrangePi R2S (2 GiB) and RV2 (4 GiB) both have a
// single rank. Defaults to 2 as before.
pub const CS_NUM: u32 = match option_env!("K1_CS_NUM") {
    Some(v) if v.len() == 1 && v.as_bytes()[0] == b'1' => 1,
    Some(v) if v.len() == 1 && v.as_bytes()[0] == b'2' => 2,
    _ => 0,
};
const DDRC_BASE: usize = 0xc000_0000;

const DFI_PHY_USER_COMMAND_0: usize = DDRC_BASE + 0x13D0;

const DPHY0_BASE_OFFSET: usize = 0x0004_0000;
const BAR_BASE_OFFSET: usize = 0x0005_0000;
const FOO_BASE_OFFSET: usize = 0x0005_8000;

// U-Boot  drivers/ddr/spacemit/k1x/ddr_init_asic.h
// memory controller channel 0
const MC_CH0_BASE_OFFSET: usize = 0x0200;
const MC_CH0_PHY_BASE_OFFSET: usize = 0x1000;

const COMMON_OFFSET: usize = 0x3000;
const SUBPHY_A_OFFSET: usize = 0x0000;
const SUBPHY_B_OFFSET: usize = 0x0200;
const FREQ_POINT_OFFSET: usize = 0x4000;

const OTHER_CONTROL_OFFSET: usize = 0x10000;

const DDR_TRAINING_INFO_MAGIC: usize = 0x54524444; // DDRT
const DDR_TRAINING_INFO_VERSION: usize = 0x0001_0000;

const PLL_BASE: usize = 0xd428_2800;

const DDRC_PLL5: usize = PLL_BASE + 0x0004;
const DDRC_PLL4: usize = PLL_BASE + 0x00b0;
const DDRC_PLL3: usize = PLL_BASE + 0x00e8;
const DDRC_PLL: usize = PLL_BASE + 0x03b4;
const DDRC_PLL2: usize = PLL_BASE + 0x039c;

const BOOT_PP: u32 = 0;

fn enable_pll() -> Result<()> {
    let v = read32(DDRC_PLL);
    write32(DDRC_PLL, v & 0xFFFF_FCFF);
    let v = read32(DDRC_PLL);
    write32(DDRC_PLL, v | (1 << 11) | (1 << 9) | (1 << 8));
    /* wait pll stable */
    let mask = 0x0003_0000;
    wait_mask(DDRC_PLL, mask, mask, "PLL stable")?;
    Ok(())
}

fn mck6_sw_fc_top(freq_no: u32) -> Result<()> {
    let freq = match freq_no {
        0 => 0x0000_3B50, // 1200MT
        1 => 0x0000_3B04, // 1600MT
        2 => 0x0000_3B40, // 2400MT
        3 => 0x0000_3B00, // 3200MT
        4 => {
            println!("DDR SW frequency change to ext clk");
            0x0000_3B02
        }
        _ => {
            return Err(Error::Geometry);
        }
    };
    write32(DDRC_PLL, freq);
    write32(DDRC_PLL4, 0x4060_0400);
    let v = 0x0400_0000;
    write32(DDRC_PLL5, v);
    wait_mask(DDRC_PLL5, v, 0, "software frequency change")?;
    Ok(())
}

fn top_common_config() -> Result<()> {
    let v = read32(DDRC_PLL2);
    write32(DDRC_PLL2, v & 0xFFFF00FF);
    let v = read32(DDRC_PLL2);
    write32(DDRC_PLL2, v | (0x3B << 8));

    enable_pll()?;
    mck6_sw_fc_top(BOOT_PP)?;

    let v = read32(DDRC_PLL3);
    write32(DDRC_PLL3, v & 0xFFFFFFFC);
    let v = read32(DDRC_PLL3);
    write32(DDRC_PLL3, v | 0x3);
    Ok(())
}

fn fp_timing_init(ddrc_base: usize) {
    let mc_ch0_base: usize = ddrc_base + MC_CH0_BASE_OFFSET;
    let mc_ch0_phy_base: usize = ddrc_base + MC_CH0_PHY_BASE_OFFSET;

    write32(mc_ch0_base + 0x0104, 0xF0800400);
    write32(mc_ch0_base + 0x0100, 0x00000E20);
    write32(mc_ch0_base + 0x010c, 0x19194314);
    write32(mc_ch0_base + 0x0110, 0x20440000);
    write32(mc_ch0_base + 0x0114, 0x20440000);
    write32(mc_ch0_base + 0x018c, 0x00000030);
    write32(mc_ch0_base + 0x0190, 0x06400030);
    write32(mc_ch0_base + 0x0194, 0x80e001c0);
    write32(mc_ch0_base + 0x01fc, 0x000C005E);
    write32(mc_ch0_base + 0x0198, 0x01CC01CC);
    write32(mc_ch0_base + 0x019c, 0x00181818);
    write32(mc_ch0_base + 0x01a0, 0x08180C0C);
    write32(mc_ch0_base + 0x01a4, 0x00000003);
    write32(mc_ch0_base + 0x01a8, 0x00000217);
    write32(mc_ch0_base + 0x01ac, 0x30651D44);
    write32(mc_ch0_base + 0x01b0, 0x1120080F);
    write32(mc_ch0_base + 0x01b4, 0x08001000);
    write32(mc_ch0_base + 0x01b8, 0x00000C00);
    write32(mc_ch0_base + 0x01bc, 0x02020404);
    write32(mc_ch0_base + 0x01c0, 0x10000004);
    write32(mc_ch0_base + 0x01c4, 0x00000006);
    write32(mc_ch0_base + 0x01d8, 0x00010190);
    write32(mc_ch0_base + 0x014c, 0x000c4090);
    write32(mc_ch0_phy_base + 0x03e4, 0x15000A02);
    write32(mc_ch0_phy_base + 0x03ec, 0x0000046c);
    write32(mc_ch0_base + 0x0104, 0xA0800400);
    write32(mc_ch0_base + 0x0100, 0x00000C18);
    write32(mc_ch0_base + 0x010c, 0x9d194314);
    write32(mc_ch0_base + 0x0110, 0x00440000);
    write32(mc_ch0_base + 0x0114, 0x00440000);
    write32(mc_ch0_base + 0x018c, 0x00430000);
    write32(mc_ch0_base + 0x0190, 0x05350028);
    write32(mc_ch0_base + 0x0194, 0x80A80151);
    write32(mc_ch0_base + 0x01fc, 0x000C005E);
    write32(mc_ch0_base + 0x0198, 0x017F017F);
    write32(mc_ch0_base + 0x019c, 0x00141414);
    write32(mc_ch0_base + 0x01a0, 0x07140A0A);
    write32(mc_ch0_base + 0x01a4, 0x00000003);
    write32(mc_ch0_base + 0x01a8, 0x00000213);
    write32(mc_ch0_base + 0x01ac, 0x36541838);
    write32(mc_ch0_base + 0x01b0, 0x1c180a18);
    write32(mc_ch0_base + 0x01b4, 0x08000E00);
    write32(mc_ch0_base + 0x01b8, 0x00000E00);
    write32(mc_ch0_base + 0x01bc, 0x02020404);
    write32(mc_ch0_base + 0x01c0, 0x10000004);
    write32(mc_ch0_base + 0x01c4, 0x00000004);
    write32(mc_ch0_base + 0x01d8, 0x0000D94E);
    write32(mc_ch0_base + 0x014c, 0x0007204a);
    write32(mc_ch0_phy_base + 0x03e4, 0x13000802);
    write32(mc_ch0_phy_base + 0x03ec, 0x00000450);
    write32(mc_ch0_base + 0x0104, 0x50800400);
    write32(mc_ch0_base + 0x0100, 0x0000080e);
    write32(mc_ch0_base + 0x010c, 0x9d194314);
    write32(mc_ch0_base + 0x0110, 0x00440000);
    write32(mc_ch0_base + 0x0114, 0x00440000);
    write32(mc_ch0_base + 0x018c, 0x00280018);
    write32(mc_ch0_base + 0x0190, 0x03200018);
    write32(mc_ch0_base + 0x0194, 0x807000e0);
    write32(mc_ch0_base + 0x01fc, 0x000C005E);
    write32(mc_ch0_base + 0x0198, 0x00e600e6);
    write32(mc_ch0_base + 0x019c, 0x000c0c0c);
    write32(mc_ch0_base + 0x01a0, 0x050c0606);
    write32(mc_ch0_base + 0x01a4, 0x00000003);
    write32(mc_ch0_base + 0x01a8, 0x0000020c);
    write32(mc_ch0_base + 0x01ac, 0x18330f22);
    write32(mc_ch0_base + 0x01b0, 0x110f080f);
    write32(mc_ch0_base + 0x01b4, 0x08000800);
    write32(mc_ch0_base + 0x01b8, 0x00000600);
    write32(mc_ch0_base + 0x01bc, 0x02020404);
    write32(mc_ch0_base + 0x01c0, 0x00000003);
    write32(mc_ch0_base + 0x01c4, 0x00000003);
    write32(mc_ch0_base + 0x01d8, 0x00008190);
    write32(mc_ch0_base + 0x014c, 0x00030848);
    write32(mc_ch0_phy_base + 0x03e4, 0x0a000402);
    write32(mc_ch0_phy_base + 0x03ec, 0x00000480);
    write32(mc_ch0_base + 0x0104, 0x00800400);
    write32(mc_ch0_base + 0x0100, 0x0000080e);
    write32(mc_ch0_base + 0x010c, 0x9d194314);
    write32(mc_ch0_base + 0x0110, 0x00440000);
    write32(mc_ch0_base + 0x0114, 0x00440000);
    write32(mc_ch0_base + 0x018c, 0x00280018);
    write32(mc_ch0_base + 0x0190, 0x03200018);
    write32(mc_ch0_base + 0x0194, 0x805400A8);
    write32(mc_ch0_base + 0x01fc, 0x000C005E);
    write32(mc_ch0_base + 0x0198, 0x00e600e6);
    write32(mc_ch0_base + 0x019c, 0x000c0c0c);
    write32(mc_ch0_base + 0x01a0, 0x050c0606);
    write32(mc_ch0_base + 0x01a4, 0x00000003);
    write32(mc_ch0_base + 0x01a8, 0x0000020c);
    write32(mc_ch0_base + 0x01ac, 0x18330f22);
    write32(mc_ch0_base + 0x01b0, 0x110f080f);
    write32(mc_ch0_base + 0x01b4, 0x08000800);
    write32(mc_ch0_base + 0x01b8, 0x00000600);
    write32(mc_ch0_base + 0x01bc, 0x02020404);
    write32(mc_ch0_base + 0x01c0, 0x00000002);
    write32(mc_ch0_base + 0x01c4, 0x00000003);
    write32(mc_ch0_base + 0x01d8, 0x00008190);
    write32(mc_ch0_base + 0x014c, 0x00030848);
    write32(mc_ch0_phy_base + 0x03e4, 0x0a000402);
    write32(mc_ch0_phy_base + 0x03ec, 0x00000480);

    let r = mc_ch0_base + 0x0108;
    write32(r, (read32(r) & 0xF00FFFFF) | (0x10 << 20));
}

fn fp_sel(ddrc_base: usize, fp: u32) {
    let mc_ch0_base: usize = ddrc_base + MC_CH0_BASE_OFFSET;

    let r = mc_ch0_base + 0x0104;
    write32(r, (read32(r) & !(0xf << 28)) | (fp << 28) | (fp << 30));

    println!("ADDR[0x{:08x}]=0x{:08x}", r, read32(r));
}

fn top_ddr_mc_init(ddrc_base: usize, fp: u32) {
    let mc_ch0_base: usize = ddrc_base + MC_CH0_BASE_OFFSET;
    let mc_ch0_phy_base: usize = ddrc_base + MC_CH0_PHY_BASE_OFFSET;
    write32(ddrc_base + 0x44, 0x00040300);
    write32(ddrc_base + 0x48, 0x00000001);
    write32(ddrc_base + 0x64, 0x100d0803);
    write32(ddrc_base + 0x50, 0x000000ff);
    write32(ddrc_base + 0x58, 0x3fd53fd5);
    write32(ddrc_base + 0x180, 0x00010200);
    write32(mc_ch0_base, 0x100001);
    write32(mc_ch0_base + 0x4, 0x0);
    write32(mc_ch0_base + 0x8, 0x100001);
    write32(mc_ch0_base + 0xc, 0x1);
    write32(ddrc_base + 0x0080, 0x00000000);
    write32(ddrc_base + 0x0a00, 0x00000000);
    write32(ddrc_base + 0x0ac0, 0x00000000);
    write32(ddrc_base + 0x0acc, 0xffffffff);
    write32(mc_ch0_base + 0x20, 0x05030732);
    write32(mc_ch0_base + 0x24, 0x05030732);
    write32(mc_ch0_base + 0xc0, 0x14008000);
    write32(mc_ch0_base + 0xc4, 0x000000b8);
    write32(mc_ch0_base + 0xc8, 0x0000FFFF);
    write32(mc_ch0_base + 0xcc, 0x200);

    fp_timing_init(ddrc_base);
    fp_sel(ddrc_base, fp);

    write32(mc_ch0_base + 0x0180, 0x30D400);
    write32(mc_ch0_base + 0x0184, 0x4E200);
    write32(mc_ch0_base + 0x0188, 0xC800000);

    let r = mc_ch0_phy_base + 0x03e0;
    write32(r, read32(r) | (fp << 2));
}

fn top_ddr_wr_ds_odt_vref(dphy0_base: usize, combination: u32) {
    let common_base = dphy0_base + COMMON_OFFSET;
    let offset_a = common_base + SUBPHY_A_OFFSET;
    let offset_b = common_base + SUBPHY_B_OFFSET;

    match combination {
        2 => {
            let d_reg2 = 0xd8;
            let data = read32(common_base + 0xc);
            let data = (data & 0xFFFF00FF) | (d_reg2 << 8);

            write32(offset_a + FREQ_POINT_OFFSET * 0 + 0xc, data);
            write32(offset_a + FREQ_POINT_OFFSET * 1 + 0xc, data);
            write32(offset_a + FREQ_POINT_OFFSET * 2 + 0xc, data);
            write32(offset_a + FREQ_POINT_OFFSET * 3 + 0xc, data);

            write32(offset_b + FREQ_POINT_OFFSET * 0 + 0xc, data);
            write32(offset_b + FREQ_POINT_OFFSET * 1 + 0xc, data);
            write32(offset_b + FREQ_POINT_OFFSET * 2 + 0xc, data);
            write32(offset_b + FREQ_POINT_OFFSET * 3 + 0xc, data);
        }
        _ => {
            println!("top_DDR_wr_ds_odt_vref: combination {combination} not supported");
        }
    }
}

fn top_ddr_rx_ds_odt_vref(dphy0_base: usize, combination: u32) {
    let common_base = dphy0_base + COMMON_OFFSET;
    let offset_a = common_base + SUBPHY_A_OFFSET;
    let offset_b = common_base + SUBPHY_B_OFFSET;

    match combination {
        2 => {
            let d_reg3 = 0xE4;
            let data = read32(common_base + 0xc);
            let data = (data & 0xFF00FFFF) | (d_reg3 << 16);

            write32(offset_a + FREQ_POINT_OFFSET * 0 + 0xc, data);
            write32(offset_a + FREQ_POINT_OFFSET * 1 + 0xc, data);
            write32(offset_a + FREQ_POINT_OFFSET * 2 + 0xc, data);
            write32(offset_a + FREQ_POINT_OFFSET * 3 + 0xc, data);

            write32(offset_b + FREQ_POINT_OFFSET * 0 + 0xc, data);
            write32(offset_b + FREQ_POINT_OFFSET * 1 + 0xc, data);
            write32(offset_b + FREQ_POINT_OFFSET * 2 + 0xc, data);
            write32(offset_b + FREQ_POINT_OFFSET * 3 + 0xc, data);

            let rx_ref_d1 = 0x55;
            let rx_ref_d2 = 0x55;
            let data = read32(common_base + 0x4);
            let data = (data & 0x0000FFFF) | (rx_ref_d1 << 16) | (rx_ref_d2 << 24);

            write32(offset_a + FREQ_POINT_OFFSET * 0 + 0x4, data);
            write32(offset_a + FREQ_POINT_OFFSET * 1 + 0x4, data);
            write32(offset_a + FREQ_POINT_OFFSET * 2 + 0x4, data);
            write32(offset_a + FREQ_POINT_OFFSET * 3 + 0x4, data);

            write32(offset_b + FREQ_POINT_OFFSET * 0 + 0x4, data);
            write32(offset_b + FREQ_POINT_OFFSET * 1 + 0x4, data);
            write32(offset_b + FREQ_POINT_OFFSET * 2 + 0x4, data);
            write32(offset_b + FREQ_POINT_OFFSET * 3 + 0x4, data);
        }
        _ => {
            println!("top_DDR_rx_ds_odt_vref: combination {combination} not supported");
        }
    }
}

fn top_ddr_amble_config(dphy0_base: usize) {
    let common_base = dphy0_base + COMMON_OFFSET;
    let offset_a = common_base + SUBPHY_A_OFFSET;
    let offset_b = common_base + SUBPHY_B_OFFSET;

    let data = read32(common_base + 0x4);
    let data = (data & 0xFFFF0FFF) | (1 << 11) | (1 << 13) | (1 << 15);

    write32(offset_a + FREQ_POINT_OFFSET * 0 + 0x4, data);
    write32(offset_a + FREQ_POINT_OFFSET * 1 + 0x4, data);
    write32(offset_a + FREQ_POINT_OFFSET * 2 + 0x4, data);
    write32(offset_a + FREQ_POINT_OFFSET * 3 + 0x4, data);

    write32(offset_b + FREQ_POINT_OFFSET * 0 + 0x4, data);
    write32(offset_b + FREQ_POINT_OFFSET * 1 + 0x4, data);
    write32(offset_b + FREQ_POINT_OFFSET * 2 + 0x4, data);
    write32(offset_b + FREQ_POINT_OFFSET * 3 + 0x4, data);
}

fn top_ddr_phy_init(ddrc_base: usize, fp: u32) {
    let dphy0_base = ddrc_base + DPHY0_BASE_OFFSET;
    let common_base = dphy0_base + COMMON_OFFSET;
    let offset_a = common_base + SUBPHY_A_OFFSET;
    let offset_b = common_base + SUBPHY_B_OFFSET;

    let r = PLL_BASE + 0x3A4;
    let v = read32(r);
    write32(r, (v & 0xFFFF00FF) | (0xF << 8));

    let r = PLL_BASE + 0x398;
    let v = read32(r);
    write32(r, v | (0x3 << 10));

    write32(offset_a, 0x0);
    write32(offset_b, 0x0);
    write32(offset_a, 0x1);
    write32(offset_b, 0x1);

    write32(dphy0_base + FREQ_POINT_OFFSET * 0 + 0x64, 0x4349);
    write32(dphy0_base + FREQ_POINT_OFFSET * 1 + 0x64, 0x4349);
    write32(dphy0_base + FREQ_POINT_OFFSET * 2 + 0x64, 0x4349);
    write32(dphy0_base + FREQ_POINT_OFFSET * 3 + 0x64, 0x4349);

    top_ddr_amble_config(dphy0_base);
    top_ddr_wr_ds_odt_vref(dphy0_base, 2);
    top_ddr_rx_ds_odt_vref(dphy0_base, 2);

    let data = read32(common_base + 0x14);
    let data = (data & 0xFF9FFFEF | (0x3 << 21));

    write32(offset_a + FREQ_POINT_OFFSET * 0 + 0x14, data);
    write32(offset_b + FREQ_POINT_OFFSET * 0 + 0x14, data);
    write32(offset_a + FREQ_POINT_OFFSET * 1 + 0x14, data);
    write32(offset_b + FREQ_POINT_OFFSET * 1 + 0x14, data);
    write32(offset_a + FREQ_POINT_OFFSET * 2 + 0x14, data);
    write32(offset_b + FREQ_POINT_OFFSET * 2 + 0x14, data);
    write32(offset_a + FREQ_POINT_OFFSET * 3 + 0x14, data);
    write32(offset_b + FREQ_POINT_OFFSET * 3 + 0x14, data);

    let data = read32(common_base + 0x10);
    let data = data | 0x10000000;

    write32(offset_a + FREQ_POINT_OFFSET * 0 + 0x10, data);
    write32(offset_b + FREQ_POINT_OFFSET * 0 + 0x10, data);
    write32(offset_a + FREQ_POINT_OFFSET * 1 + 0x10, data);
    write32(offset_b + FREQ_POINT_OFFSET * 1 + 0x10, data);
    write32(offset_a + FREQ_POINT_OFFSET * 2 + 0x10, data);
    write32(offset_b + FREQ_POINT_OFFSET * 2 + 0x10, data);
    write32(offset_a + FREQ_POINT_OFFSET * 3 + 0x10, data);
    write32(offset_b + FREQ_POINT_OFFSET * 3 + 0x10, data);

    write32(common_base + 0x30, 0x1077);
    write32(dphy0_base + OTHER_CONTROL_OFFSET + 0x24, 0x0);

    let r = dphy0_base + OTHER_CONTROL_OFFSET;
    let v = read32(r);
    write32(r, v | 0x1);
}

fn top_ddr_mc_phy_device_init(ddrc_base: usize, cs_val: u32, fp: u32) -> Result<()> {
    let cs_num = if cs_val == 1 { 1 } else { 3 };

    top_ddr_mc_init(ddrc_base, fp);
    top_ddr_phy_init(ddrc_base, fp);

    let mc_ch0_phy_base: usize = ddrc_base + MC_CH0_PHY_BASE_OFFSET;

    // same as Marvell
    write32(DFI_PHY_USER_COMMAND_0, 0x1300_0001);
    let m = 0x8000_0000;
    wait_mask(mc_ch0_phy_base + 0x3fc, m, m, "PHY init")?;
    println!("PHY INIT done");

    write32(DFI_PHY_USER_COMMAND_0, 0x1300_0100);
    write32(DDRC_BASE + 0x20, 0x10000001 | (cs_num << 24));

    println!("wait DRAM INIT");
    wait_mask(ddrc_base + 0x8, 0x11, 0x11, "DRAM init")?;
    println!("DRAM INIT done");

    write32(ddrc_base + 0x24, 0x10020001 | (cs_num << 24));
    write32(ddrc_base + 0x24, 0x10020002 | (cs_num << 24));
    write32(ddrc_base + 0x24, 0x1002000d | (cs_num << 24));
    write32(ddrc_base + 0x24, 0x10020003 | (cs_num << 24));
    write32(ddrc_base + 0x24, 0x10020016 | (cs_num << 24));

    write32(ddrc_base + 0x20, 0x11002000);
    write32(ddrc_base + 0x20, 0x11001000);

    if (cs_val != 1) {
        write32(ddrc_base + 0x20, 0x12002000);
        write32(ddrc_base + 0x20, 0x12001000);
    }

    write32(ddrc_base + 0x24, 0x1002000C | (cs_num << 24));
    write32(ddrc_base + 0x24, 0x1002000E | (cs_num << 24));
    write32(ddrc_base + 0x24, 0x1002000B | (cs_num << 24));
    write32(ddrc_base + 0x24, 0x10020017 | (cs_num << 24));

    println!("DRAM Mode register Init done.");
    Ok(())
}

fn mem_read<'a, T>(addr: usize) -> &'a T {
    unsafe { (addr as *const T).as_ref().unwrap() }
}

const DDR_MR_DATA: usize = (DDRC_BASE + 0x370);
const DDR_MR_REG: usize = (DDRC_BASE + 0x24);

fn mode_register_read(mr: u32, ch: u32, cs: u32) -> Result<u32> {
    write32(DDR_MR_REG, 0x10010000 + ((cs + 1) << 24) + (ch << 18) + mr);
    wait_mask(DDR_MR_DATA, 0x80000000, 0x80000000, "mode register read")?;
    Ok(read32(DDRC_BASE + 0x234) & 0xFF)
}

// MR8 density in MiB per die (two independent 16-bit channels): the decode
// and the "every rank and channel reports the same MR8" rule are
// jedec_lpddr::lp4::density (ore-edk2-boot); the MR8 reads and the totals the
// K1 accepts stay here.
fn channel_size(mr8: u32) -> Result<u32> {
    jedec_lpddr::lp4::density::channel_mib(mr8 as u8).map_err(|_| Error::Geometry)
}
fn ddr_get_density(cs_num: u32) -> Result<u32> {
    let mut mr8s = [0u8; 4];
    let mut n = 0;
    for cs in 0..cs_num.min(2) {
        for ch in 0..2 {
            mr8s[n] = mode_register_read(8, ch, cs)? as u8;
            n += 1;
        }
    }
    let size = jedec_lpddr::lp4::density::total_mib(&mr8s[..n]).map_err(|_| Error::Geometry)?;
    if !matches!(size, 1024 | 2048 | 4096 | 8192) {
        return Err(Error::Geometry);
    }
    Ok(size)
}

// include/configs/k1-x.h
const DDR_CS_NUM: u32 = 1;

#[repr(u32)]
enum Bank {
    B2 = 0,
    B4,
    B8,
    Reserved,
}

#[repr(u32)]
enum Row {
    R11 = 1,
    R12,
    R13,
    R14,
    R15,
    R16,
    R17,
    R18,
}

#[repr(u32)]
enum Column {
    C8 = 1,
    C9,
    C10,
    C11,
    C12,
}

fn adjust_mapping(ddrc_base: usize, cs_num: u32, size_mb: u32, mr8_value: u32) -> Result<()> {
    let area_length_mb = size_mb / cs_num;
    // area_length_mb = size_mb >> (cs_num -1);
    let area_length_cfg = match area_length_mb {
        1024 => 0xE,
        2048 => 0xF,
        4096 => 0x10,
        8192 => 0x11,
        16384 => 0x12,
        _ => {
            return Err(Error::Geometry);
        }
    };

    let cs1_start_addr_l = (area_length_mb >> 3) & 0x1FF;
    let cs1_start_addr_h = (area_length_mb >> 12) & 0xFFFFFFFF;

    let io_width = mr8_value >> 6;
    let density = (mr8_value >> 2) & 0xf;

    const IO_X16: u32 = 0;
    const IO_X8: u32 = 1;

    let row = match (io_width, density) {
        (IO_X16, 12) => Row::R13,
        (IO_X16, 0) => Row::R14,
        (IO_X16, 1) => Row::R15,
        (IO_X16, 2) => Row::R15,
        (IO_X16, 3) => Row::R16,
        (IO_X16, 4) => Row::R16,
        (IO_X16, 5) => Row::R17,
        (IO_X16, 6) => Row::R17,
        (IO_X8, 12) => Row::R14,
        (IO_X8, 0) => Row::R15,
        (IO_X8, 1) => Row::R16,
        (IO_X8, 2) => Row::R16,
        (IO_X8, 3) => Row::R17,
        (IO_X8, 4) => Row::R17,
        (IO_X8, 5) => Row::R18,
        (IO_X8, 6) => Row::R18,
        _ => return Err(Error::Geometry),
    };
    let bank = Bank::B8;
    let column = Column::C10;

    let mc_ch0_base = ddrc_base + MC_CH0_BASE_OFFSET;

    let v = read32(mc_ch0_base);
    write32(mc_ch0_base, (v & 0x0060FFFF) | (area_length_cfg << 16));

    write32(mc_ch0_base + 0x4, 0x0);

    let v = read32(mc_ch0_base + 0x8);
    write32(
        mc_ch0_base + 0x8,
        (v & 0x0060FFFF | (cs1_start_addr_l << 23) | (area_length_cfg << 16)),
    );

    write32(mc_ch0_base + 0xc, cs1_start_addr_h);

    let a = ((row as u32) << 8) | ((column as u32) << 4) | (bank as u32);
    let mask = 0xFFFFF00C;

    let v = read32(mc_ch0_base + 0x20);
    write32(mc_ch0_base + 0x20, (v & mask) | a);

    let v = read32(mc_ch0_base + 0x24);
    write32(mc_ch0_base + 0x24, (v & mask) | a);

    let reg = mc_ch0_base;
    println!("DEBUG-ADDR[0x{reg:08x}]:0x{:08x}", read32(reg));
    let reg = mc_ch0_base + 0x4;
    println!("DEBUG-ADDR[0x{reg:08x}]:0x{:08x}", read32(reg));
    let reg = mc_ch0_base + 0x8;
    println!("DEBUG-ADDR[0x{reg:08x}]:0x{:08x}", read32(reg));
    let reg = mc_ch0_base + 0xc;
    println!("DEBUG-ADDR[0x{reg:08x}]:0x{:08x}", read32(reg));
    let reg = mc_ch0_base + 0x20;
    println!("DEBUG-ADDR[0x{reg:08x}]:0x{:08x}", read32(reg));
    let reg = mc_ch0_base + 0x24;
    println!("DEBUG-ADDR[0x{reg:08x}]:0x{:08x}", read32(reg));
    Ok(())
}

fn ddr_dfc_table_init(ddrc_base: usize) {
    write32(ddrc_base + 0x74, 0x00040303);
    write32(ddrc_base + 0x78, 0x00000044);
    write32(ddrc_base + 0x70, 0x00000000);

    write32(ddrc_base + 0x74, 0x13000008);
    write32(ddrc_base + 0x78, 0x00000020);
    write32(ddrc_base + 0x70, 0x00000001);

    write32(ddrc_base + 0x74, 0x13010000);
    write32(ddrc_base + 0x78, 0x00000028);
    write32(ddrc_base + 0x70, 0x00000002);

    write32(ddrc_base + 0x74, 0x1302000d);
    write32(ddrc_base + 0x78, 0x00000024);
    write32(ddrc_base + 0x70, 0x00000003);

    write32(ddrc_base + 0x74, 0x13020001);
    write32(ddrc_base + 0x78, 0x00000024);
    write32(ddrc_base + 0x70, 0x00000004);

    write32(ddrc_base + 0x74, 0x13020002);
    write32(ddrc_base + 0x78, 0x00000024);
    write32(ddrc_base + 0x70, 0x00000005);
    write32(ddrc_base + 0x74, 0x13020003);
    write32(ddrc_base + 0x78, 0x00000024);
    write32(ddrc_base + 0x70, 0x00000006);
    write32(ddrc_base + 0x74, 0x1302000b);
    write32(ddrc_base + 0x78, 0x00000024);
    write32(ddrc_base + 0x70, 0x00000007);
    write32(ddrc_base + 0x74, 0x1302000c);
    write32(ddrc_base + 0x78, 0x00000024);
    write32(ddrc_base + 0x70, 0x00000008);
    write32(ddrc_base + 0x74, 0x1302000e);
    write32(ddrc_base + 0x78, 0x00000024);
    write32(ddrc_base + 0x70, 0x00000009);
    write32(ddrc_base + 0x74, 0x13020016);
    write32(ddrc_base + 0x78, 0x00000024);
    write32(ddrc_base + 0x70, 0x0000000a);
    write32(ddrc_base + 0x74, 0x13008000);
    write32(ddrc_base + 0x78, 0x00000028);
    write32(ddrc_base + 0x70, 0x0000000b);
    write32(ddrc_base + 0x74, 0x1302000d);
    write32(ddrc_base + 0x78, 0x00000024);
    write32(ddrc_base + 0x70, 0x0000000c);
    write32(ddrc_base + 0x74, 0x13000010);
    write32(ddrc_base + 0x78, 0x00000020);
    write32(ddrc_base + 0x70, 0x0000000d);
    write32(ddrc_base + 0x74, 0x00000002);
    write32(ddrc_base + 0x78, 0x00002008);
    write32(ddrc_base + 0x70, 0x0000000e);
    write32(ddrc_base + 0x74, 0x00000002);
    write32(ddrc_base + 0x78, 0x00002008);
    write32(ddrc_base + 0x70, 0x0000000f);
    write32(ddrc_base + 0x74, 0x13000001);
    write32(ddrc_base + 0x78, 0x000013d0);
    write32(ddrc_base + 0x70, 0x00000010);
    write32(ddrc_base + 0x74, 0x00008000);
    write32(ddrc_base + 0x78, 0x000033fc);
    write32(ddrc_base + 0x70, 0x00000011);
    write32(ddrc_base + 0x74, 0x00000000);
    write32(ddrc_base + 0x78, 0x000033fc);
    write32(ddrc_base + 0x70, 0x00000012);
    write32(ddrc_base + 0x74, 0x00040303);
    write32(ddrc_base + 0x78, 0x00010044);
    write32(ddrc_base + 0x70, 0x00000013);
    write32(ddrc_base + 0x74, 0x10000100);
    write32(ddrc_base + 0x78, 0x000013d0);
    write32(ddrc_base + 0x70, 0x00000014);
    write32(ddrc_base + 0x74, 0x00008000);
    write32(ddrc_base + 0x78, 0x000033fc);
    write32(ddrc_base + 0x70, 0x00000015);
    write32(ddrc_base + 0x74, 0x00008000);
    write32(ddrc_base + 0x78, 0x000033fc);
    write32(ddrc_base + 0x70, 0x00000016);
    write32(ddrc_base + 0x74, 0x13000004);
    write32(ddrc_base + 0x78, 0x00000020);
    write32(ddrc_base + 0x70, 0x00000017);
    write32(ddrc_base + 0x74, 0x1302000d);
    write32(ddrc_base + 0x78, 0x00000024);
    write32(ddrc_base + 0x70, 0x00000018);
    write32(ddrc_base + 0x74, 0x00000002);
    write32(ddrc_base + 0x78, 0x00002008);
    write32(ddrc_base + 0x70, 0x00000019);
    write32(ddrc_base + 0x74, 0x00000000);
    write32(ddrc_base + 0x78, 0x00002008);
    write32(ddrc_base + 0x70, 0x0000001a);
    write32(ddrc_base + 0x74, 0x00040380);
    write32(ddrc_base + 0x78, 0x00020044);
    write32(ddrc_base + 0x70, 0x0000001b);
    write32(ddrc_base + 0x74, 0x00040380);
    write32(ddrc_base + 0x78, 0x00020044);
    write32(ddrc_base + 0x70, 0x0000012e);
    write32(ddrc_base + 0x74, 0x00040b43);
    write32(ddrc_base + 0x78, 0x00000044);
    write32(ddrc_base + 0x70, 0x00000180);
    write32(ddrc_base + 0x74, 0x13000010);
    write32(ddrc_base + 0x78, 0x00000020);
    write32(ddrc_base + 0x70, 0x00000181);
    write32(ddrc_base + 0x74, 0x00000002);
    write32(ddrc_base + 0x78, 0x00002008);
    write32(ddrc_base + 0x70, 0x00000182);
    write32(ddrc_base + 0x74, 0x00000002);
    write32(ddrc_base + 0x78, 0x00002008);
    write32(ddrc_base + 0x70, 0x00000183);
    write32(ddrc_base + 0x74, 0x13000001);
    write32(ddrc_base + 0x78, 0x000013d0);
    write32(ddrc_base + 0x70, 0x00000184);
    write32(ddrc_base + 0x74, 0x00008000);
    write32(ddrc_base + 0x78, 0x000033fc);
    write32(ddrc_base + 0x70, 0x00000185);
    write32(ddrc_base + 0x74, 0x00000000);
    write32(ddrc_base + 0x78, 0x000033fc);
    write32(ddrc_base + 0x70, 0x00000186);
    write32(ddrc_base + 0x74, 0x00040b43);
    write32(ddrc_base + 0x78, 0x00010044);
    write32(ddrc_base + 0x70, 0x00000187);
    write32(ddrc_base + 0x74, 0x10000100);
    write32(ddrc_base + 0x78, 0x000013d0);
    write32(ddrc_base + 0x70, 0x00000188);
    write32(ddrc_base + 0x74, 0x00008000);
    write32(ddrc_base + 0x78, 0x000033fc);
    write32(ddrc_base + 0x70, 0x00000189);
    write32(ddrc_base + 0x74, 0x00008000);
    write32(ddrc_base + 0x78, 0x000033fc);
    write32(ddrc_base + 0x70, 0x0000018a);
    write32(ddrc_base + 0x74, 0x1302000d);
    write32(ddrc_base + 0x78, 0x00000024);
    write32(ddrc_base + 0x70, 0x0000018b);
    write32(ddrc_base + 0x74, 0x00000002);
    write32(ddrc_base + 0x78, 0x00002008);
    write32(ddrc_base + 0x70, 0x0000018c);
    write32(ddrc_base + 0x74, 0x00000000);
    write32(ddrc_base + 0x78, 0x00002008);
    write32(ddrc_base + 0x70, 0x0000018d);
    write32(ddrc_base + 0x74, 0x00040b00);
    write32(ddrc_base + 0x78, 0x00020044);
    write32(ddrc_base + 0x70, 0x0000018e);
}

const ADDRS: [usize; 22] = [
    0x0048,
    0x0054,
    0x0058,
    0x0060,
    0x0064,
    0x0148,
    0x014c,
    MC_CH0_BASE_OFFSET + 0x0000,
    MC_CH0_BASE_OFFSET + 0x0004,
    MC_CH0_BASE_OFFSET + 0x0008,
    MC_CH0_BASE_OFFSET + 0x000c,
    MC_CH0_BASE_OFFSET + 0x0020,
    MC_CH0_BASE_OFFSET + 0x0024,
    MC_CH0_BASE_OFFSET + 0x00c4,
    MC_CH0_BASE_OFFSET + 0x00c0,
    MC_CH0_BASE_OFFSET + 0x0180,
    MC_CH0_BASE_OFFSET + 0x0184,
    MC_CH0_BASE_OFFSET + 0x0188,
    0x80,
    0xa00,
    0xac0,
    0xacc,
];

const ADDRS2: [usize; 24] = [
    MC_CH0_BASE_OFFSET + 0x0100,
    MC_CH0_BASE_OFFSET + 0x010c,
    MC_CH0_BASE_OFFSET + 0x0110,
    MC_CH0_BASE_OFFSET + 0x0114,
    MC_CH0_BASE_OFFSET + 0x018c,
    MC_CH0_BASE_OFFSET + 0x0190,
    MC_CH0_BASE_OFFSET + 0x0194,
    MC_CH0_BASE_OFFSET + 0x0198,
    MC_CH0_BASE_OFFSET + 0x019c,
    MC_CH0_BASE_OFFSET + 0x01a0,
    MC_CH0_BASE_OFFSET + 0x01a4,
    MC_CH0_BASE_OFFSET + 0x01a8,
    MC_CH0_BASE_OFFSET + 0x01ac,
    MC_CH0_BASE_OFFSET + 0x01b0,
    MC_CH0_BASE_OFFSET + 0x01b4,
    MC_CH0_BASE_OFFSET + 0x01b8,
    MC_CH0_BASE_OFFSET + 0x01bc,
    MC_CH0_BASE_OFFSET + 0x01c0,
    MC_CH0_BASE_OFFSET + 0x01c4,
    MC_CH0_BASE_OFFSET + 0x0200,
    MC_CH0_BASE_OFFSET + 0x01d8,
    MC_CH0_BASE_OFFSET + 0x014c,
    MC_CH0_PHY_BASE_OFFSET + 0x03e4,
    MC_CH0_PHY_BASE_OFFSET + 0x03ec,
];

fn init_table_mc_tim(ddrc_base: usize, idx: &mut u32) {
    for a in ADDRS2 {
        let v = read32(ddrc_base + a);
        write32(ddrc_base + 0x0074, v);
        write32(ddrc_base + 0x0078, a as u32);
        write32(ddrc_base + 0x0070, *idx);
        *idx += 1;
    }
}

fn init_table_mc_a0(ddrc_base: usize) {
    let mut idx = 0x200;
    let dphy0_base = ddrc_base + DPHY0_BASE_OFFSET;
    let mc_cfg2_offset = MC_CH0_BASE_OFFSET + 0x0104;
    let mc_cfg2_addr = ddrc_base + mc_cfg2_offset;

    let mc_ctl0_org = read32(ddrc_base + 0x44);
    write32(ddrc_base + 0x74, mc_ctl0_org | (1 << 2) | (1 << 12));
    write32(ddrc_base + 0x78, 0x00000044 | (0x1 << 16));
    write32(ddrc_base + 0x70, idx);
    idx += 1;

    for a in ADDRS {
        write32(ddrc_base + 0x74, read32(ddrc_base + a));
        write32(ddrc_base + 0x78, a as u32 & 0xffff);
        write32(ddrc_base + 0x70, idx);
        idx += 1;
    }

    let mc_cfg2_org = read32(mc_cfg2_addr);
    let temp_data = mc_cfg2_org & !(0xf << 28);

    let mc_cfg2_fp = temp_data;
    write32(mc_cfg2_addr, mc_cfg2_fp);
    write32(ddrc_base + 0x0074, mc_cfg2_fp);
    write32(ddrc_base + 0x0078, mc_cfg2_offset as u32);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;
    init_table_mc_tim(ddrc_base, &mut idx);

    let mc_cfg2_fp = temp_data | (0x5 << 28);
    write32(mc_cfg2_addr, mc_cfg2_fp);
    let mc_cfg2_fp = mc_cfg2_fp & !(0x3 << 28);
    write32(ddrc_base + 0x0074, mc_cfg2_fp);
    write32(ddrc_base + 0x0078, mc_cfg2_offset as u32);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;
    init_table_mc_tim(ddrc_base, &mut idx);

    let mc_cfg2_fp = temp_data | (0xa << 28);
    write32(mc_cfg2_addr, mc_cfg2_fp);
    let mc_cfg2_fp = mc_cfg2_fp & !(0x3 << 28);
    write32(ddrc_base + 0x0074, mc_cfg2_fp);
    write32(ddrc_base + 0x0078, mc_cfg2_offset as u32);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;
    init_table_mc_tim(ddrc_base, &mut idx);

    let mc_cfg2_fp = temp_data | (0xf << 28);
    write32(mc_cfg2_addr, mc_cfg2_fp);
    let mc_cfg2_fp = mc_cfg2_fp & !(0x3 << 28);
    write32(ddrc_base + 0x0074, mc_cfg2_fp);
    write32(ddrc_base + 0x0078, mc_cfg2_offset as u32);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;
    init_table_mc_tim(ddrc_base, &mut idx);

    write32(ddrc_base + 0x0074, 0x00020200);
    write32(ddrc_base + 0x0078, 0x000013e0);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x13000010);
    write32(ddrc_base + 0x0078, 0x000013d0);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x00010000);
    write32(ddrc_base + 0x0078, 0x000033fc);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x00010000);
    write32(ddrc_base + 0x0078, 0x000033fc);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x13000008);
    write32(ddrc_base + 0x0078, 0x00000020);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x13000004);
    write32(ddrc_base + 0x0078, 0x00000020);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x13020000);
    write32(ddrc_base + 0x0078, 0x00000028);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x13000001);
    write32(ddrc_base + 0x0078, 0x000013d0);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x00008000);
    write32(ddrc_base + 0x0078, 0x000033fc);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x00008000);
    write32(ddrc_base + 0x0078, 0x000033fc);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x10000100);
    write32(ddrc_base + 0x0078, 0x000013d0);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x00008000);
    write32(ddrc_base + 0x0078, 0x000033fc);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x00008000);
    write32(ddrc_base + 0x0078, 0x000033fc);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x1302000d);
    write32(ddrc_base + 0x0078, 0x00000024);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, 0x13020003);
    write32(ddrc_base + 0x0078, 0x00000024);
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(ddrc_base + 0x0074, mc_ctl0_org);
    write32(ddrc_base + 0x0078, 0x44 | (0x1 << 17));
    write32(ddrc_base + 0x0070, idx);
    idx += 1;

    write32(dphy0_base + 0x10104, 0x00001100);
    write32(dphy0_base + 0x10108, 0x00010000);
    write32(dphy0_base + 0x10100, 0x00000020);
    write32(dphy0_base + 0x10104, 0x000000ff);
    write32(dphy0_base + 0x10108, 0x0001001c);
    write32(dphy0_base + 0x10100, 0x00000021);
    write32(dphy0_base + 0x10104, 0x00000000);
    write32(dphy0_base + 0x10108, 0x0005001c);
    write32(dphy0_base + 0x10100, 0x00000022);

    write32(mc_cfg2_addr, mc_cfg2_org);
}

const MC_REQ_TABLE_NUM: u32 = 3;
const FREQ_PLL_CHG_MODE: u32 = 9;
const DCLK_BYPASS_DIV: u32 = 16;
const DCLK_BYPASS_RST: u32 = 21;
const DCLK_BYPASS_CLK_EN: u32 = 22;
const DDRPHY0_EN: u32 = 30;

fn ddr_dfc(ddrc_base: usize, freq_no: u32) -> Result<()> {
    let r = PLL_BASE + 0x098;
    write32(r, read32(r) | 0x10);
    write32(ddrc_base + 0x148, 0x80ac0000);

    const BASE_CFG: u32 = (1 << DDRPHY0_EN) | (1 << DCLK_BYPASS_RST);
    fn get_cfg1(r: u32) -> u32 {
        BASE_CFG | (1 << FREQ_PLL_CHG_MODE) | (r << MC_REQ_TABLE_NUM)
    }

    const FREQ_REG1: usize = PLL_BASE + 0x3b4;
    const FREQ_REG2: usize = PLL_BASE + 0x0b0;

    match freq_no {
        0 => {
            println!("change to 1200");
            write32(FREQ_REG1, 0x00003B50);
            write32(FREQ_REG2, get_cfg1(0));
        }
        1 => {
            println!("change to 1600");
            write32(FREQ_REG1, 0x00003B04);
            write32(FREQ_REG2, get_cfg1(1));
        }
        2 => {
            println!("change to 2400");
            write32(FREQ_REG1, 0x00003B40);
            write32(FREQ_REG2, get_cfg1(2));
        }
        3 => {
            println!("change to 3200");
            write32(FREQ_REG1, 0x00003B00);
            write32(FREQ_REG2, get_cfg1(3));
        }
        4 => {
            println!("change to ext clock");
            let cfg = BASE_CFG | (1 << DCLK_BYPASS_CLK_EN) | (1 << DCLK_BYPASS_DIV);
            write32(FREQ_REG1, 0x00003B02);
            write32(FREQ_REG2, cfg);
        }
        x => {
            return Err(Error::Geometry);
        }
    }

    const DDR_FREQ_CHG_REQ: u32 = 22;
    let r = PLL_BASE + 0x0004;
    write32(r, 1 << DDR_FREQ_CHG_REQ);
    wait_mask(r, 0x400000, 0, "DDR frequency change")?;

    println!("frequency change done!");
    Ok(())
}

// The translated vendor DRAM training (BLOB-03) is not in this tree: the
// K1 bt0 trains with our own code (train_own.rs; feature own-training, the
// default here).

/// Only the two explicitly supported board capacities are accepted for boot.
pub fn check_profile_size(measured: u32, expected: u32) -> Result<()> {
    if !matches!(expected, 2048 | 4096) || measured != expected {
        return Err(Error::Geometry);
    }
    Ok(())
}
#[cfg(test)]
mod profile_size_tests {
    use super::*;
    #[test]
    fn known_board_sizes_do_not_cross_accept() {
        for (actual, expected) in [(2048, 2048), (4096, 4096)] {
            assert_eq!(check_profile_size(actual, expected), Ok(()));
        }
        for (actual, expected) in [
            (4096, 2048),
            (2048, 4096),
            (0, 2048),
            (8192, 4096),
            (2048, 0),
        ] {
            assert_eq!(check_profile_size(actual, expected), Err(Error::Geometry));
        }
    }
}

/// Bring DRAM up with `cs_num` ranks and train it; returns its size in MiB
/// from the mode registers.
pub fn init(cs_num: u32) -> Result<u32> {
    if !matches!(cs_num, 1 | 2) {
        return Err(Error::Rank);
    }

    top_common_config()?;
    top_ddr_mc_phy_device_init(DDRC_BASE, cs_num, 0)?;

    let size_mb = ddr_get_density(cs_num)?;
    let mr8_value = mode_register_read(8, 0, 0)? & 0xff;
    println!("DDR size (density): {size_mb}MB");
    println!("MR 8: {mr8_value}");
    adjust_mapping(DDRC_BASE, cs_num, size_mb, mr8_value)?;

    ddr_dfc_table_init(0xF0000000);
    init_table_mc_a0(0xF0000000);

    let fp = 0;
    train_fp(DDRC_BASE, cs_num, fp)?;

    let fp = 1;
    ddr_dfc(DDRC_BASE, fp)?;
    train_fp(DDRC_BASE, cs_num, fp)?;

    let fp = 2;
    ddr_dfc(DDRC_BASE, fp)?;
    train_fp(DDRC_BASE, cs_num, fp)?;

    let data_rate = DATA_RATE;
    /* change dram frequency */
    match data_rate {
        1600 => ddr_dfc(DDRC_BASE, 1)?,
        2400 => ddr_dfc(DDRC_BASE, 2)?,
        1200 | _ => ddr_dfc(DDRC_BASE, 0)?,
    }

    Ok(size_mb)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn finite_poll_exact_success_and_timeout() {
        let mut n = 0;
        assert_eq!(
            poll(
                || {
                    n += 1;
                    if n == 3 {
                        6
                    } else {
                        2
                    }
                },
                6,
                6,
                3
            ),
            Ok(())
        );
        assert_eq!(n, 3);
        let mut n = 0;
        assert_eq!(
            poll(
                || {
                    n += 1;
                    2
                },
                6,
                6,
                7
            ),
            Err(2)
        );
        assert_eq!(n, 7);
        assert_eq!(poll(|| panic!("zero budget read"), 1, 1, 0), Err(0));
    }
    #[test]
    fn channel_geometry_rejects_reserved_width_density() {
        assert_eq!(channel_size(0x10), Ok(1024));
        assert_eq!(channel_size(0x18), Ok(2048));
        assert_eq!(channel_size(0x50), Ok(2048));
        assert!(channel_size(0x80).is_err());
        assert!(channel_size(0x1c).is_err());
    }
    #[test]
    fn full_driver_r2s_rv2_mr14_and_no_low_address_reads() {
        for (mr8, size) in [(0x10, 2048), (0x18, 4096)] {
            util::mmio::MODEL.with(|m| {
                *m.borrow_mut() = util::mmio::Model {
                    mr8,
                    ..Default::default()
                }
            });
            assert_eq!(init(1), Ok(size));
            util::mmio::MODEL.with(|m| {
                let m = m.borrow();
                let commands = m
                    .writes
                    .iter()
                    .filter(|&&(a, v)| a == DDRC_BASE + 0x24 && v == 0x1302000e)
                    .count();
                assert_eq!(commands, 3 * 17);
            });
        }
    }
    #[test]
    fn full_driver_timeout_never_reaches_training_consumers() {
        for address in [DDRC_PLL, DDRC_BASE + 0x13fc, DDRC_BASE + 8, DDR_MR_DATA] {
            util::mmio::MODEL.with(|m| {
                *m.borrow_mut() = util::mmio::Model {
                    fail: Some(address),
                    mr8: 0x10,
                    ..Default::default()
                }
            });
            assert!(matches!(init(1),Err(Error::Timeout{address:a,..})if a==address));
            util::mmio::MODEL.with(|m| {
                let m = m.borrow();
                let setup_reads = if address == DDRC_PLL { 2 } else { 0 };
                assert_eq!(m.limit_reads, POLL_LIMIT + setup_reads);
                assert!(!m.writes.iter().any(|&(a, _)| a == 0xf0000070));
            });
        }
        assert_eq!(init(0), Err(Error::Rank));
        assert_eq!(init(3), Err(Error::Rank));
    }
    #[test]
    fn hardware_error_status_propagates() {
        for error in [2, 4, 8, 16] {
            util::mmio::MODEL.with(|m| {
                *m.borrow_mut() = util::mmio::Model {
                    mr8: 0x10,
                    training_error: error,
                    ..Default::default()
                }
            });
            assert_eq!(init(1), Err(Error::Training(error)));
        }
    }
}

#[cfg(test)]
mod readiness_tests {
    use super::*;
    fn stalled(address: usize, value: u32) {
        util::mmio::MODEL.with(|m| {
            *m.borrow_mut() = util::mmio::Model {
                mr8: 0x10,
                fail: Some(address),
                fail_value: value,
                ..Default::default()
            }
        });
    }
    fn timeout(r: Result<()>, address: usize) {
        assert!(matches!(r,Err(Error::Timeout{address:a,..})if a==address));
        util::mmio::MODEL.with(|m| assert_eq!(m.borrow().limit_reads, POLL_LIMIT));
    }
    #[test]
    fn pll_switch_and_dfc_timeouts() {
        stalled(DDRC_PLL5, 1 << 26);
        timeout(mck6_sw_fc_top(0), DDRC_PLL5);
        stalled(PLL_BASE + 4, 1 << 22);
        timeout(ddr_dfc(DDRC_BASE, 0), PLL_BASE + 4);
    }
    #[test]
    fn invalid_density_propagates_before_mapping() {
        stalled(0, 0);
        util::mmio::MODEL.with(|m| m.borrow_mut().mr8 = 0xff);
        assert_eq!(init(1), Err(Error::Geometry));
    }
}

// Training dispatch. Appended at the end of the file so that the default
// (translated) build keeps its code and panic locations byte for byte.

/// Mode register read for the training backend (`HwPhy`).
#[cfg(any(feature = "own-training", feature = "train-report"))]
fn mrr(mr: u32, ch: u32, cs: u32) -> Option<u32> {
    mode_register_read(mr, ch, cs).ok()
}

/// Either training, followed by the same per-point report (cargo feature
/// `train-report`) so the two can be compared side by side.
#[cfg(any(feature = "own-training", feature = "train-report"))]
fn train_fp(ddrc_base: usize, cs_num: u32, fp: u32) -> Result<()> {
    #[cfg(feature = "train-report")]
    let start = crate::train_report::cycles();
    let r = train_core(ddrc_base, cs_num, fp);
    #[cfg(feature = "train-report")]
    if r.is_ok() {
        crate::train_report::after_fp(ddrc_base, cs_num, fp, start, mrr);
    }
    r
}

/// Our own training (cargo feature `own-training`; doc/K1-OWN-TRAINING-PLAN.md).
#[cfg(feature = "own-training")]
fn train_core(ddrc_base: usize, cs_num: u32, fp: u32) -> Result<()> {
    use crate::train_engine::{Error as E, HwPhy, PHY_STATUS};
    let mut phy = HwPhy::new(ddrc_base, fp, cs_num, mrr);
    phy.apply_params(&crate::train_own::PARAMS);
    #[cfg(feature = "train-probe")]
    crate::train_probe::before_training(&mut phy);
    let r = crate::train_own::train_fp(&mut phy, &crate::train_own::CONFIG);
    #[cfg(feature = "train-probe")]
    crate::train_probe::after_training(&mut phy, &r);
    r.map(|_| ()).map_err(|e| {
        println!("[k1t] fp{fp}: own training failed: {e:?}");
        match e {
            E::Timeout { engine, last, .. } => Error::Timeout {
                stage: engine.name(),
                address: ddrc_base + PHY_STATUS,
                last,
            },
            E::Mrr => Error::Timeout {
                stage: "mode register read",
                address: DDR_MR_DATA,
                last: 0,
            },
            E::Gate { status, .. } => Error::Training(status),
            E::Closed { fp, which } => Error::Training(0x1_0000 | fp << 8 | which as u32),
            E::Sanity { fp, which } => Error::Training(0x3_0000 | fp << 8 | which as u32),
            E::Margin { fp, which, worst } => {
                Error::Training(0x2_0000 | fp << 8 | (which as u32) << 7 | worst as u32 & 0x7f)
            }
        }
    })
}

// Host tests of our own training through the K1 backend (`HwPhy`) against the
// MMIO model in tests/k1_host.rs: `make test-training`.
#[cfg(all(test, feature = "own-training"))]
mod own_training_tests {
    use super::*;
    use crate::train_engine::{cs_mask, HwPhy, Phy, PHY_PARAM};
    use util::mmio::{Model, MODEL};
    fn model(m: Model) {
        MODEL.with(|x| *x.borrow_mut() = m);
    }
    fn writes() -> Vec<(usize, u32)> {
        MODEL.with(|m| m.borrow().writes.clone())
    }
    fn reg(a: usize) -> u32 {
        MODEL.with(|m| m.borrow().regs.get(&a).copied().unwrap_or(0))
    }
    const RX_REGS: [usize; 2] = [0xc0043004, 0xc0043204];

    #[test]
    fn own_training_full_driver_r2s_rv2() {
        for (mr8, size) in [(0x10, 2048), (0x18, 4096)] {
            model(Model {
                mr8,
                ..Default::default()
            });
            assert_eq!(init(1), Ok(size));
            let w = writes();
            // RX: every point centred at code 7 (not the first peak, 6), DQS
            // nibbles and the low 16 bits as the GPL init left them.
            for fp in 0..3 {
                for r in RX_REGS {
                    assert_eq!(reg(r + fp * 0x4000), 0x5757_a800, "fp{fp} 0x{r:x}");
                }
            }
            // TX: MR14 at the model's peak, bit 23 and the other fields kept.
            assert_eq!(reg(0xc000030c) & 0x00ff_0000, 0x001e_0000);
            assert_eq!(reg(0xc000030c) & 0xff00_ffff, 0x9d00_4314);
            // MRW MR14 for one rank only; never the two-rank mask.
            assert!(w.iter().any(|&(a, v)| a == 0xc0000024 && v == 0x1102_000e));
            assert!(!w.iter().any(|&(a, v)| a == 0xc0000024 && v == 0x1302_000e));
            // Nothing the translation writes and we do not, once training has
            // started: engine parameters (0x13ec also has GPL init values),
            // the other-control set-up register, self refresh commands.
            let start = w
                .iter()
                .position(|&(a, v)| a == 0xc00013d0 && v & 0x0078_0000 != 0)
                .unwrap();
            for a in PHY_PARAM
                .map(|r| DDRC_BASE + r)
                .into_iter()
                .chain([0xc0050028])
            {
                assert!(!w[start..].iter().any(|&(x, _)| x == a), "wrote 0x{a:x}");
            }
            assert!(!w
                .iter()
                .any(|&(a, v)| a == 0xc0000020 && (v & 0xf0 == 0x40 || v & 0xf0 == 0x80)));
            // Engine order per point: WL, gate, read, write; rank 0 only.
            let cmds: Vec<u32> = w
                .iter()
                .filter(|&&(a, v)| a == 0xc00013d0 && v & 0x0078_0000 != 0)
                .map(|&(_, v)| v)
                .collect();
            assert!(cmds.iter().all(|v| v & 0x0f00_0000 == 0x0100_0000));
            let first = |bit: u32| cmds.iter().position(|v| v & bit != 0).unwrap();
            assert!(first(1 << 22) < first(1 << 20));
            assert!(first(1 << 20) < first(1 << 21));
            assert!(first(1 << 21) < first(1 << 19));
            // Read eye scans all 16 codes, plus one re-run, per point.
            assert_eq!(cmds.iter().filter(|v| **v & (1 << 21) != 0).count(), 3 * 17);
        }
    }

    #[test]
    fn own_training_two_ranks_use_both_selects() {
        model(Model {
            mr8: 0x10,
            ..Default::default()
        });
        assert_eq!(init(2), Ok(4096));
        let w = writes();
        assert!(w.iter().any(|&(a, v)| a == 0xc00013d0 && v == 0x1240_0000));
        assert!(w.iter().any(|&(a, v)| a == 0xc0000024 && v == 0x1302_000e));
        assert_eq!(cs_mask(1), 1);
        assert_eq!(cs_mask(2), 3);
    }

    #[test]
    fn own_training_engine_timeout_and_gate_fail() {
        // PHY init done (bit 31) but no engine ever reports done.
        model(Model {
            mr8: 0x10,
            fail: Some(0xc00013fc),
            fail_value: 0x8000_0000,
            ..Default::default()
        });
        assert!(matches!(
            init(1),
            Err(Error::Timeout {
                stage: "write-leveling",
                address: 0xc00013fc,
                ..
            })
        ));
        MODEL.with(|m| assert_eq!(m.borrow().limit_reads, 1 + POLL_LIMIT));
        // Gate done (bit 1) without its pass bit 2.
        model(Model {
            mr8: 0x10,
            fail: Some(0xc00013fc),
            fail_value: 0x8000_0003,
            ..Default::default()
        });
        assert_eq!(init(1), Err(Error::Training(0x8000_0003)));
    }

    #[test]
    fn own_training_closed_eye_fails_and_mr14_mismatch_only_warns() {
        // A read eye closed at every code: the v1.0.0 default policy's
        // knob-response check reports it (all-fail saturation, 0x3_0000);
        // the legacy policy (pre-v1.0.0 behaviour) reports a closed eye.
        model(Model {
            mr8: 0x10,
            rx_closed: true,
            ..Default::default()
        });
        assert_eq!(init(1), Err(Error::Training(0x3_0000)));
        model(Model {
            mr8: 0x10,
            rx_closed: true,
            ..Default::default()
        });
        let mut phy = HwPhy::new(DDRC_BASE, 0, 1, |_, _, _| Some(0));
        let legacy = crate::train_own::Config {
            policy: dram_train::vref_sweep::SweepPolicy::legacy(),
            ..crate::train_own::CONFIG
        };
        assert_eq!(
            crate::train_own::train_fp(&mut phy, &legacy).map(|_| ()),
            Err(crate::train_engine::Error::Closed { fp: 0, which: 0 })
        );
        model(Model {
            mr8: 0x10,
            mrr_wrong: true,
            ..Default::default()
        });
        assert_eq!(init(1), Ok(2048));
    }

    /// The R2S fp2 write eye of the Oct 14 f run (MR14 range 0 settings
    /// 0..=50 as recorded), wider than the scan: range 1 continues at 6.
    fn r2s_fp2_flat_write_eye() -> Vec<u8> {
        let mut p = vec![
            7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
            9, 9, 9, 9, 8, 9, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6,
        ];
        p.resize(81, 6);
        p
    }

    #[test]
    fn own_training_flat_write_eye_plateau_vs_legacy() {
        // v1.0.0 default (REQ-K1-01): the eye is wider than the scan, so the
        // chosen MR14 is on the peak plateau (width 9). The legacy
        // half-height centre takes the middle of the scan (axis 40, width 8),
        // the R2S f result.
        for (policy, want_width, want_op) in [
            (dram_train::vref_sweep::SweepPolicy::DEFAULT, 9, None),
            (dram_train::vref_sweep::SweepPolicy::legacy(), 8, Some(40u8)),
        ] {
            model(Model {
                mr8: 0x10,
                tx_profile: r2s_fp2_flat_write_eye(),
                ..Default::default()
            });
            write32(0xc000030c, 0x9d19_4314);
            let mut phy = HwPhy::new(DDRC_BASE, 2, 1, |mr, ch, cs| {
                mode_register_read(mr, ch, cs).ok()
            });
            let cfg = crate::train_own::Config {
                policy,
                ..crate::train_own::CONFIG
            };
            let r = crate::train_own::train_fp(&mut phy, &cfg).unwrap();
            assert_eq!(r.tx_worst, want_width, "{policy:?}");
            if let Some(op) = want_op {
                assert_eq!(r.mr14, op);
            }
            assert!(!r.mr14_mismatch);
        }
    }

    #[test]
    fn own_training_hwphy_fields_and_params() {
        model(Model::default());
        let mut phy = HwPhy::new(DDRC_BASE, 1, 1, |_, _, _| Some(0));
        write32(0xc0047004, 0x5a5a_1234);
        phy.set_rx_vref(0x13);
        assert_eq!(reg(0xc0047004), 0x535a_1234 & 0xf0f0_ffff | 0x0303_0000);
        assert_eq!(phy.rx_vref(), 3);
        // MR14 OP[5:0] in [21:16], OP[6] (range) in bit 23, bit 22 kept.
        write32(0xc000030c, 0);
        phy.set_tx_mr14(0x45);
        assert_eq!(reg(0xc000030c), 0x0085_0000);
        assert_eq!(phy.tx_mr14(), 0x45);
        write32(0xc000030c, 0xffff_ffff);
        phy.set_tx_mr14(0x05);
        assert_eq!(reg(0xc000030c), 0xff45_ffff);
        assert_eq!(phy.tx_mr14(), 0x05);
        for op in 0..0x80u8 {
            let v = crate::train_engine::mr14_into_register(0x9d19_4314, op);
            assert_eq!(crate::train_engine::mr14_from_register(v), op);
            assert_eq!(v & 0xff40_ffff, 0x9d00_4314);
        }
        phy.apply_params(&[Some(0x1234), None, Some(0)]);
        assert_eq!(reg(0xc00013e8), 0x1234);
        assert!(!writes().iter().any(|&(a, _)| a == 0xc00013ec));
        assert_eq!(reg(0xc00013f8), 0);
        assert_eq!(crate::train_own::env_hex(Some("0x0A64")), Some(0xa64));
        assert_eq!(crate::train_own::env_hex(Some("ff")), Some(0xff));
        assert_eq!(crate::train_own::env_hex(None), None);
    }
}
