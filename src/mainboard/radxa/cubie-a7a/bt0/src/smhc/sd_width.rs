// SPDX-License-Identifier: GPL-2.0-only

// SD R1 error bits, including card locked, but not state/status bits such as
// READY_FOR_DATA, APP_CMD and CARD_ECC_DISABLED.
const R1_ERROR: u32 = 0xfff9_a008;
const R1_APP_CMD: u32 = 1 << 5;

/// The card and controller both start at one bit. A rejected command leaves
/// them there; a missing response leaves receipt of ACMD6 ambiguous.
pub(super) fn switch<E>(
    mut command: impl FnMut(u32, u32) -> Result<u32, E>,
    rca: u32,
    host_four_bit: impl FnOnce(),
) -> Result<bool, E> {
    let r1 = command(55, rca << 16)?;
    if r1 & (R1_ERROR | R1_APP_CMD) != R1_APP_CMD {
        return Ok(false);
    }
    if command(6, 2)? & R1_ERROR != 0 {
        return Ok(false);
    }
    host_four_bit();
    Ok(true)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::{vec, vec::Vec};

    const TRANSFER_READY: u32 = (4 << 9) | (1 << 8);

    type Attempt = (Result<bool, ()>, u8, Vec<(u32, u32)>);

    fn attempt(cmd55: u32, acmd6: Result<u32, ()>) -> Attempt {
        let mut seen = Vec::new();
        let mut host = 1;
        let result = switch(
            |idx, arg| {
                seen.push((idx, arg));
                match idx {
                    55 => Ok(cmd55),
                    6 => acmd6,
                    _ => unreachable!(),
                }
            },
            0x1234,
            || host = 4,
        );
        (result, host, seen)
    }

    #[test]
    fn accepts_only_a_confirmed_switch() {
        assert_eq!(
            attempt(TRANSFER_READY | R1_APP_CMD, Ok(TRANSFER_READY)),
            (Ok(true), 4, vec![(55, 0x1234_0000), (6, 2)])
        );
    }

    #[test]
    fn bad_prefix_never_sends_acmd6() {
        for bad in [TRANSFER_READY, TRANSFER_READY | R1_APP_CMD | (1 << 22)] {
            assert_eq!(
                attempt(bad, Ok(TRANSFER_READY)),
                (Ok(false), 1, vec![(55, 0x1234_0000)])
            );
        }
    }

    #[test]
    fn rejected_switch_keeps_host_at_one_bit() {
        assert_eq!(
            attempt(TRANSFER_READY | R1_APP_CMD, Ok(TRANSFER_READY | (1 << 22))),
            (Ok(false), 1, vec![(55, 0x1234_0000), (6, 2)])
        );
    }

    #[test]
    fn missing_response_requests_reidentification_without_switching_host() {
        assert_eq!(
            attempt(TRANSFER_READY | R1_APP_CMD, Err(())),
            (Err(()), 1, vec![(55, 0x1234_0000), (6, 2)])
        );
    }
}
