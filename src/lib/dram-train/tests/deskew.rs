// SPDX-License-Identifier: GPL-2.0-only
//! P3-M4: per-bit deskew, write centring by margins and the centre from
//! partial edges, against plants.

use dram_train::deskew::{
    centre_from_edges, per_bit, split_difference, CentreFrom, Deskew, DeskewError, Edges, Params,
    Split, Target,
};

/// Bits with skewed edges: bit b's data turns valid at `edge[b] + delay`;
/// the strobe samples at `strobe`; a bit passes when the strobe is at or
/// after its edge. `noise` lists (bit, strobe, bit delay) points that fail
/// once (a flaky sample).
struct Byte {
    edge: Vec<u16>,
    delay: Vec<u16>,
    strobe: u16,
    strobe_trace: Vec<u16>,
    noise: Vec<(u8, u16, u16)>,
    probes: u32,
}
impl Byte {
    fn new(edge: &[u16]) -> Self {
        Self {
            edge: edge.to_vec(),
            delay: vec![0; edge.len()],
            strobe: 0,
            strobe_trace: Vec::new(),
            noise: Vec::new(),
            probes: 0,
        }
    }
}
impl Target for Byte {
    type Error = ();
    fn set_strobe(&mut self, d: u16) -> Result<(), ()> {
        self.strobe = d;
        self.strobe_trace.push(d);
        Ok(())
    }
    fn set_bit(&mut self, b: u8, d: u16) -> Result<(), ()> {
        self.delay[b as usize] = d;
        Ok(())
    }
    fn passes(&mut self, b: u8) -> Result<bool, ()> {
        self.probes += 1;
        let key = (b, self.strobe, self.delay[b as usize]);
        if let Some(i) = self.noise.iter().position(|&n| n == key) {
            self.noise.remove(i);
            return Ok(false);
        }
        Ok(self.strobe >= self.edge[b as usize] + self.delay[b as usize])
    }
}

fn params(bits: u8, confirm: u8) -> Params {
    Params {
        bits,
        strobe_start: 0,
        strobe_max: 200,
        bit_max: 63,
        confirm,
    }
}

fn check_aligned(b: &Byte, r: &Deskew<8>) {
    let latest = *b.edge.iter().max().unwrap();
    assert_eq!(r.strobe, latest, "strobe at the latest bit");
    for (i, (&e, &d)) in b.edge.iter().zip(&r.bit).enumerate() {
        assert_eq!(e + d, r.strobe, "bit {i} edge-aligned");
        assert_eq!(b.delay[i], d, "hardware left at the result");
    }
    assert!(
        b.strobe_trace.windows(2).all(|w| w[0] <= w[1]),
        "strobe never moved back: {:?}",
        b.strobe_trace
    );
}

#[test]
fn bits_align_to_the_latest_and_the_strobe_never_goes_back() {
    for edges in [
        &[10u16, 12, 9, 15, 11, 10, 13, 12][..],
        &[30, 2, 2, 2, 2, 2, 2, 2],    // first bit latest
        &[2, 2, 2, 2, 2, 2, 2, 30],    // last bit latest: second pass needed
        &[5, 5, 5, 5, 5, 5, 5, 5],     // no skew
        &[0, 40, 0, 40, 0, 40, 0, 40], // two groups
    ] {
        let mut b = Byte::new(edges);
        let r: Deskew<8> = per_bit(&mut b, &params(8, 0)).unwrap();
        check_aligned(&b, &r);
        assert_eq!(r.saturated, 0);
        let latest = edges.iter().max().unwrap();
        if *latest > 0 {
            assert_eq!(edges[r.latest.unwrap() as usize], *latest);
        }
    }
}

#[test]
fn flaky_fails_are_confirmed_away() {
    let edges = [10u16, 12, 9, 15, 11, 10, 13, 12];
    let mut b = Byte::new(&edges);
    // Bit 2 at strobe 15 fails once at delay 4 (edge 13 is still before 15).
    b.noise = vec![(2, 15, 4), (2, 15, 4)];
    let r: Deskew<8> = per_bit(&mut b, &params(8, 2)).unwrap();
    check_aligned(&b, &r);
}

#[test]
fn a_bit_earlier_than_the_delay_line_reaches_is_flagged() {
    let mut b = Byte::new(&[0, 100, 50, 60]);
    let mut p = params(4, 0);
    p.bit_max = 63;
    let r: Deskew<8> = per_bit(&mut b, &p).unwrap();
    assert_eq!(r.strobe, 100);
    assert_eq!(r.saturated, 1 << 0, "bit 0 needs 100 taps, the line has 63");
    assert_eq!(r.bit[0], 63);
    assert_eq!((r.bit[2], r.bit[3]), (50, 40));
}

#[test]
fn errors() {
    let mut b = Byte::new(&[10, 250]);
    assert_eq!(
        per_bit::<_, 8>(&mut b, &params(2, 0)),
        Err(DeskewError::StrobeRange(1))
    );
    assert_eq!(
        per_bit::<_, 8>(&mut Byte::new(&[0; 9]), &params(9, 0)),
        Err(DeskewError::Config)
    );
}

#[test]
fn split_difference_adds_delay_to_the_larger_margin_side() {
    // The window runs from -left to +right around the start.
    for (l, r) in [(10u32, 30u32), (30, 10), (20, 20), (0, 9), (9, 0)] {
        let s = split_difference(l, r);
        let centre = (r as i32 - l as i32) / 2;
        assert_eq!(s.dqs as i32 - s.dq as i32, centre, "l {l} r {r}");
        assert!(s.dqs == 0 || s.dq == 0, "only one side moves");
    }
    assert_eq!(split_difference(10, 30), Split { dqs: 10, dq: 0 });
    assert_eq!(split_difference(30, 10), Split { dqs: 0, dq: 10 });
}

#[test]
fn the_centre_from_any_subset_of_edges() {
    // A window [100, 160] (centre 130) and the next one [190, 250] (UI 90
    // taps), the boundary noise [160, 190]; delay range 0..=255.
    let ui = 90;
    let all = Edges {
        rise_left: Some(100),
        rise_right: Some(160),
        fall_left: Some(190),
        fall_right: Some(250),
    };
    let only = |f: fn(&mut Edges)| {
        let mut e = Edges::default();
        f(&mut e);
        e
    };
    let cases = [
        (all, 130, CentreFrom::Window),
        (
            Edges {
                rise_left: None,
                ..all
            },
            130,
            CentreFrom::Boundary,
        ),
        (
            Edges {
                rise_left: None,
                rise_right: None,
                ..all
            },
            130,
            CentreFrom::NextWindow,
        ),
        (
            Edges {
                rise_left: None,
                fall_right: None,
                ..all
            },
            130,
            CentreFrom::Boundary,
        ),
    ];
    for (e, c, from) in cases {
        assert_eq!(centre_from_edges(e, ui, 0, 255), Some((c, from)), "{e:?}");
    }
    // One edge and the limit on its open side: right limit 160 = the
    // window's own right edge gives the true centre.
    assert_eq!(
        centre_from_edges(only(|e| e.rise_left = Some(100)), ui, 0, 160),
        Some((130, CentreFrom::EdgeAndLimit))
    );
    assert_eq!(
        centre_from_edges(only(|e| e.rise_right = Some(160)), ui, 100, 255),
        Some((130, CentreFrom::EdgeAndLimit))
    );
    assert_eq!(
        centre_from_edges(only(|e| e.fall_left = Some(190)), ui, 0, 250),
        Some((130, CentreFrom::EdgeAndLimit))
    );
    assert_eq!(
        centre_from_edges(only(|e| e.fall_right = Some(250)), ui, 190, 255),
        Some((130, CentreFrom::EdgeAndLimit))
    );
    assert_eq!(centre_from_edges(Edges::default(), ui, 0, 255), None);
}
