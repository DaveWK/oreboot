// SPDX-License-Identifier: GPL-2.0-only
//! Original normalization of an explicitly contracted 64-row interval record.
//! This is software geometry, not a physical measurement or A733 backend.
use crate::search::{
    matrix::{MatrixDomain, MatrixEye, Rectangle},
    Cause, Point,
};

/// Caller-reviewed encoding contract. Width and the exact reversed empty-row
/// marker must be supplied; neither 255/0 nor 127/0 is a physical default.
#[derive(Clone, Copy, Debug)]
pub struct Contract {
    pub width: u16,
    pub empty: [u8; 2],
}
/// An immutable 132-byte record: 64 inclusive (low, high) pairs followed by four
/// opaque bytes. The metadata/result bytes are never interpreted or modified.
/// Validation does not establish acquisition freshness, bit coverage, units,
/// physical topology or that every point between captured endpoints passed.
pub struct Table<'a> {
    record: &'a [u8; 132],
    contract: Contract,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Geometry {
    pub passing_points: u32,
    /// Rounded arithmetic point mean, halves upward, for linear axes only.
    pub centroid: Point,
    /// A centroid can fall in a gap. It is never accepted merely by averaging.
    pub centroid_passes: bool,
}
impl<'a> Table<'a> {
    pub fn new(record: &'a [u8; 132], contract: Contract) -> Result<Self, Cause> {
        if !(1..=256).contains(&contract.width) || contract.empty[0] <= contract.empty[1] {
            return Err(Cause::InvalidPlan);
        }
        for pair in record[..128].as_chunks::<2>().0 {
            if pair == &contract.empty {
                continue;
            }
            if pair[0] > pair[1] || u16::from(pair[1]) >= contract.width {
                return Err(Cause::InvalidPlan);
            }
        }
        Ok(Self { record, contract })
    }
    pub fn contains(&self, point: Point) -> bool {
        if point.y >= 64 || u16::from(point.x) >= self.contract.width {
            return false;
        }
        let i = 2 * usize::from(point.y);
        let lo = self.record[i];
        let hi = self.record[i + 1];
        [lo, hi] != self.contract.empty && lo <= point.x && point.x <= hi
    }
    /// Bounded 64-row closed-form arithmetic, independently tested against
    /// enumerated cells. At most 16384 points; twice-x sum <= 8355840 fits u32.
    /// This reports the centroid even outside the pass set for diagnostics;
    /// callers must not apply it unchecked. Empty geometry returns NoWindow.
    pub fn geometry(&self) -> Result<Geometry, Cause> {
        let mut count = 0u32;
        let mut twice_x = 0u32;
        let mut twice_y = 0u32;
        for (row, pair) in self.record[..128].as_chunks::<2>().0.iter().enumerate() {
            if pair == &self.contract.empty {
                continue;
            }
            let n = u32::from(pair[1]) - u32::from(pair[0]) + 1;
            count += n;
            twice_x += n * (u32::from(pair[0]) + u32::from(pair[1]));
            twice_y += n * 2 * row as u32;
        }
        if count == 0 {
            return Err(Cause::NoWindow);
        }
        let centroid = Point {
            x: ((twice_x + count) / (2 * count)) as u8,
            y: ((twice_y + count) / (2 * count)) as u8,
        };
        Ok(Geometry {
            passing_points: count,
            centroid,
            centroid_passes: self.contains(centroid),
        })
    }
    /// Feed normalized inclusive cells into the existing original all-pass
    /// rectangle policy. Both record axes are explicitly linear here; circular
    /// encodings require a separately reviewed normalization. No centroid or
    /// captured output byte is substituted for this objective. At most 16384
    /// pushes, constant memory. A physical caller still needs all-bit/repeat
    /// coverage, apply/fresh verification and whole-session recovery.
    pub fn select(&self, min_width: u16, min_rows: u16) -> Result<MatrixEye, Cause> {
        let mut rectangle = Rectangle::new(MatrixDomain {
            width: self.contract.width,
            height: 64,
            circular_timing: false,
            min_width,
            min_height: min_rows,
            // These fields validate the geometry engine only, not a probe plan.
            scan_repeats: 1,
            verify_repeats: 1,
            polls_per_probe: 1,
            probe_operations: 1,
        })?;
        for y in 0..64 {
            for x in 0..self.contract.width {
                rectangle.push(self.contains(Point { x: x as u8, y }))?;
            }
        }
        rectangle.finish()
    }
}
