//! Running the PHY training firmware and reading its mailbox.

use super::*;
use crate::hw::{Error, Hw};

/// Pstate 0 (2400 MHz) training takes about 3.7 s on the Cubie A7A.
pub const TRAIN_TIMEOUT_US: u64 = 10_000_000;
/// Independent software budgets stop stalled clocks; physical latency and
/// appropriate production margins still require board acceptance.
pub const TRAIN_MAX_READS: u32 = 100_000_000;
pub const ACK_TIMEOUT_US: u64 = 1_000_000;
pub const ACK_MAX_READS: u32 = 10_000_000;
/// Mailbox major message: training finished successfully.
pub const MSG_TRAINING_COMPLETE: u32 = 0x07;
pub const MSG_TRAINING_FAILED: u32 = 0xff;

/// Start the firmware loaded in IMEM/DMEM, wait for its final mailbox
/// message and stop it again. Returns the message.
pub fn run<H: Hw>(hw: &mut H, pstate: u8) -> Result<u32, Error> {
    hw.kick_watchdog();
    csr_access(hw, false);
    // Reset and stall the microcontroller, then release both.
    hw.write16(MICRO_RESET, 9);
    hw.write16(MICRO_RESET, 1);
    hw.write16(MICRO_RESET, 0);
    hw.write16(DCT_WRITE_PROT, 1);
    hw.write16(UCT_WRITE_PROT, 1);
    let t0 = hw.now_us();
    let mut next_report = 1_000_000;
    // UctShadowRegs [0] low: the firmware posted a message.
    let mut posted = false;
    for _ in 0..TRAIN_MAX_READS {
        if hw.read16(UCT_SHADOW_REGS) & 1 == 0 {
            posted = true;
            break;
        }
        let t = hw.now_us().wrapping_sub(t0);
        if t >= TRAIN_TIMEOUT_US {
            hw.log(format_args!(
                "[dram] pstate {pstate}: PHY firmware posted no message in {} s\n",
                t / 1_000_000
            ));
            hw.write16(MICRO_RESET, 1);
            return Err(Error::Timeout("PHY training mailbox"));
        }
        if t >= next_report {
            next_report += 1_000_000;
            hw.log(format_args!(
                "[dram] pstate {pstate}: training, {} s\n",
                t / 1_000_000
            ));
        }
    }
    if !posted {
        hw.write16(MICRO_RESET, 1);
        return Err(Error::Timeout("PHY training mailbox read budget"));
    }
    let mail = hw.read16(UCT_WRITE_ONLY_SHADOW) as u32
        | (hw.read16(UCT_DAT_WRITE_ONLY_SHADOW) as u32) << 16;
    // Acknowledge, and wait for the firmware to take the message back.
    hw.write16(DCT_WRITE_PROT, 0);
    let t1 = hw.now_us();
    let mut acknowledged = false;
    for _ in 0..ACK_MAX_READS {
        // The readiness handshake uses bit0 in both directions. Unrelated
        // status bits are not acknowledgement; no status register is written.
        if hw.read16(UCT_SHADOW_REGS) & 1 != 0 {
            acknowledged = true;
            break;
        }
        if hw.now_us().wrapping_sub(t1) >= ACK_TIMEOUT_US {
            break;
        }
    }
    hw.write16(DCT_WRITE_PROT, 1);
    hw.write16(MICRO_RESET, 1);
    if !acknowledged {
        hw.log(format_args!(
            "[dram] pstate {pstate}: mailbox ack timed out\n"
        ));
        return Err(Error::Timeout("PHY training mailbox acknowledgement"));
    }
    Ok(mail)
}
