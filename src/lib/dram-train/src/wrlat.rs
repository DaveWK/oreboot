// SPDX-License-Identifier: GPL-2.0-only
//! Write-latency (cycle slip) check with a framed pattern.
//!
//! Write leveling aligns the strobe to the clock but cannot tell whole
//! cycles apart. Writing a recognisable frame between a known pre-fill and
//! post-fill and reading the three bursts back shows by how many beats the
//! write landed early or late. Method: AMD PG150 v1.4 p54 and p852-855, UG586
//! v4.2 p134-135 (cited, not copied); the frame bytes are the ones PG150
//! names. If no frame is found at all, a single write read back many times
//! tells a failing write path (all reads agree) from a failing read path
//! (reads differ).

/// The frame, one byte per beat on a byte lane.
pub const FRAME: [u8; 8] = [0xff, 0x00, 0xaa, 0x55, 0x55, 0xaa, 0x99, 0x66];
/// Pre-fill and post-fill bytes around the frame.
pub const PRE_FILL: u8 = 0x00;
pub const POST_FILL: u8 = 0xff;

/// The beats to write for one lane: `burst` beats of pre-fill, the frame
/// padded with post-fill to `burst` beats, `burst` beats of post-fill.
/// Returns the beats written (3 x `burst`); `out` must hold them and
/// `burst` must be at least the frame length.
pub fn fill(out: &mut [u8], burst: usize) -> Option<usize> {
    if burst < FRAME.len() || out.len() < 3 * burst {
        return None;
    }
    for (i, b) in out[..3 * burst].iter_mut().enumerate() {
        *b = if i < burst {
            PRE_FILL
        } else if i < burst + FRAME.len() {
            FRAME[i - burst]
        } else {
            POST_FILL
        };
    }
    Some(3 * burst)
}

/// Where the frame came back.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameShift {
    /// At the expected beat.
    Aligned,
    /// The write landed this many beats early: add the latency.
    Early(u8),
    /// This many beats late (normally a board or setup error).
    Late(u8),
    /// No frame in the read-back.
    NoMatch,
}

impl FrameShift {
    /// Whole clock cycles of a beat shift at `beats_per_ck` (2 for DDR at
    /// the command clock); None if not a whole number of cycles.
    pub const fn cycles(beats: u8, beats_per_ck: u8) -> Option<u8> {
        if beats_per_ck == 0 || !beats.is_multiple_of(beats_per_ck) {
            None
        } else {
            Some(beats / beats_per_ck)
        }
    }
}

/// Find the frame in `readback` (the three bursts read back from where
/// [`fill`] wrote them); it is expected at beat `burst`.
pub fn frame_shift(readback: &[u8], burst: usize) -> FrameShift {
    let n = FRAME.len();
    if readback.len() < n {
        return FrameShift::NoMatch;
    }
    let mut found: Option<usize> = None;
    for k in 0..=readback.len() - n {
        if readback[k..k + n] == FRAME {
            // Prefer the match nearest the expected position.
            let better = match found {
                None => true,
                Some(f) => k.abs_diff(burst) < f.abs_diff(burst),
            };
            if better {
                found = Some(k);
            }
        }
    }
    match found {
        None => FrameShift::NoMatch,
        Some(k) if k == burst => FrameShift::Aligned,
        Some(k) if k < burst => FrameShift::Early((burst - k).min(255) as u8),
        Some(k) => FrameShift::Late((k - burst).min(255) as u8),
    }
}

/// Which path failed when no frame came back.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NoMatchCause {
    /// Every read of one write agrees: the data in the DRAM is wrong.
    WritePath,
    /// The reads differ: reading is unstable.
    ReadPath,
    /// Fewer than two reads.
    Unknown,
}

/// Classify a [`FrameShift::NoMatch`] from several reads of one write
/// (PG150 uses 20).
pub fn classify_no_match(reads: &[&[u8]]) -> NoMatchCause {
    if reads.len() < 2 {
        return NoMatchCause::Unknown;
    }
    if reads.iter().all(|r| *r == reads[0]) {
        NoMatchCause::WritePath
    } else {
        NoMatchCause::ReadPath
    }
}
