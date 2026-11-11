// SPDX-License-Identifier: GPL-2.0-only
//! Endpoint handling on the behavioural DWC3 model: enumeration through
//! ep0 (SETUP/data/status stages, SET_ADDRESS, SET_CONFIGURATION and the
//! bulk endpoint commands it issues), stalls, bulk IN/OUT including the
//! bounce-buffered tail and multi-TRB downloads, bus resets and timeouts,
//! and a whole fastboot session over the controller.

mod model;

#[cfg(feature = "fastboot")]
use dwc3_udc::trb;
use dwc3_udc::{regs::*, Config, Descriptors, Error, Udc, XFER_MAX};
use model::{dma, enumeration, plat, setup, Host, Model, Plat, BASE, SNPSID_300A, SNPSID_330A};

const DESC: Descriptors = Descriptors {
    vendor: 0x18d1,
    product: 0x4ee0,
    class: [0xff, 0x42, 3],
    strings: ["oreboot", "yy3588", "0123", "fastboot"],
};

fn cfg() -> Config {
    Config {
        settle_us: 10,
        transfer_polls: 1000,
        ..Config::new(BASE)
    }
}

fn udc(m: Model) -> Udc<'static, Model, Plat> {
    Udc::init(m, plat(), cfg(), dma(), DESC).expect("init")
}

fn configured(extra: &[Host]) -> Udc<'static, Model, Plat> {
    let mut m = Model::default();
    m.script.extend(enumeration());
    m.script.extend(extra.iter().cloned());
    let mut u = udc(m);
    u.wait_configured().unwrap();
    settle(&mut u);
    u
}

/// SET_CONFIGURATION takes effect at its SETUP; let its status stage (and
/// anything else the host does without the device asking) finish.
fn settle(u: &mut Udc<'static, Model, Plat>) {
    let idle = u.io().allow_idle_end;
    u.io().allow_idle_end = true;
    for _ in 0..50 {
        if u.poll().is_err() {
            break;
        }
    }
    u.io().allow_idle_end = idle;
}

/// Enumeration and a download on both core revisions (3.00a on the
/// RK3588, 3.30a as first modelled).
#[test]
fn both_revisions_enumerate_and_move_data() {
    let mut seen = Vec::new();
    for id in [SNPSID_300A, SNPSID_330A] {
        let mut m = Model::with_snpsid(id);
        m.script.extend(enumeration());
        m.script.push_back(Host::BulkOut(vec![0x5a; 1000]));
        let mut u = udc(m);
        assert_eq!(u.revision(), id);
        u.wait_configured().unwrap();
        settle(&mut u);
        let mut buf = vec![0u8; 1000];
        u.recv_into(buf.as_mut_ptr() as usize, buf.len()).unwrap();
        assert!(buf.iter().all(|b| *b == 0x5a), "{id:#x}");
        let setups = enumeration()
            .iter()
            .filter(|h| matches!(h, Host::Setup(_)))
            .count();
        assert_eq!(u.io().controls.len(), setups, "{id:#x}");
        assert!(u.io().controls.iter().all(|c| !c.stalled));
        seen.push((u.io().controls.clone(), u.io().cmds.clone()));
    }
    // The same control transfers and the same endpoint commands.
    assert_eq!(seen[0].0, seen[1].0);
    assert_eq!(
        seen[0].1.iter().map(|c| (c.0, c.1)).collect::<Vec<_>>(),
        seen[1].1.iter().map(|c| (c.0, c.1)).collect::<Vec<_>>()
    );
}

#[test]
fn enumeration_runs_every_control_stage() {
    let mut u = configured(&[]);
    assert!(u.configured());
    let m = u.io();
    let c = &m.controls;
    assert_eq!(c.len(), 8);
    // GET_DESCRIPTOR(device, 64): 18 bytes, the fastboot IDs, three-stage.
    assert_eq!(c[0].data.len(), 18);
    assert_eq!(&c[0].data[8..12], &[0xd1, 0x18, 0xe0, 0x4e]);
    assert_eq!(c[0].status_type, 4);
    assert!(!c[0].stalled);
    // SET_ADDRESS: two-stage, DCFG.DevAddr = 7.
    assert!(c[1].data.is_empty());
    assert_eq!(c[1].status_type, 3);
    assert_eq!((m.regs[&DCFG] >> 3) & 0x7f, 7);
    // Configuration: first 9 bytes, then all 32 with 512-byte bulk.
    assert_eq!(c[3].data.len(), 9);
    assert_eq!(c[4].data.len(), 32);
    assert_eq!(&c[4].data[22..24], &[0, 2]);
    assert_eq!(&c[4].data[29..31], &[0, 2]);
    // Strings.
    assert_eq!(c[5].data, [4, 3, 0x09, 0x04]);
    assert_eq!(&c[6].data[..4], &[14, 3, b'y', 0]);
    // SET_CONFIGURATION(1).
    assert_eq!(c[7].status_type, 3);
    // ConnectDone (twice: two bus resets) set ep0's max packet to 64 with
    // the "modify" action.
    let cfgs = m.cmds_of(DEPCMD_SETEPCONFIG);
    let modify: Vec<_> = cfgs
        .iter()
        .filter(|c| c.2[0] & DEPCFG_ACTION_MODIFY != 0)
        .collect();
    assert_eq!(modify.len(), 4);
    for c in &modify {
        assert!(c.0 < 2);
        assert_eq!((c.2[0] >> 3) & 0x7ff, 64);
    }
    // SET_CONFIGURATION: start config with XferRscIdx 2 on ep0, then both
    // bulk endpoints: type bulk, 512, TX FIFO 1 for IN, events, number.
    let starts = m.cmds_of(DEPCMD_DEPSTARTCFG);
    assert_eq!(starts.len(), 2);
    assert_eq!(starts[0].1 >> 16, 0);
    assert_eq!((starts[1].0, starts[1].1 >> 16), (0, 2));
    let bulk: Vec<_> = cfgs.iter().filter(|c| c.0 >= 2).collect();
    assert_eq!(bulk.len(), 2);
    assert_eq!(bulk[0].2[0], (2 << 1) | (512 << 3));
    assert_eq!(bulk[0].2[1], (1 << 8) | (1 << 9) | (1 << 10) | (2 << 25));
    assert_eq!(bulk[1].2[0], (2 << 1) | (512 << 3) | (1 << 17));
    assert_eq!(bulk[1].2[1], (1 << 8) | (1 << 9) | (1 << 10) | (3 << 25));
    let res = m.cmds_of(DEPCMD_SETTRANSFRESOURCE);
    assert_eq!(res.iter().map(|c| c.0).collect::<Vec<_>>(), [0, 1, 2, 3]);
    assert!(res.iter().all(|c| c.2[0] == 1));
    assert_eq!(m.regs[&DALEPENA], 0xf);
}

#[test]
fn unsupported_requests_stall_and_the_next_one_works() {
    let mut u = configured(&[
        // A class request with an IN data stage, GET_DESCRIPTOR(BOS).
        Host::Setup(setup(0xa1, 0xfe, 0, 0, 1)),
        Host::Setup(setup(0x80, 6, 0x0f00, 0, 5)),
        Host::Setup(setup(0x80, 0, 0, 0, 2)),
        Host::Setup(setup(0x80, 8, 0, 0, 1)),
    ]);
    settle(&mut u);
    let m = u.io();
    let c = &m.controls[8..];
    assert!(c[0].stalled && c[1].stalled);
    assert_eq!((c[2].data.as_slice(), c[2].stalled), (&[1u8, 0][..], false));
    assert_eq!(c[3].data, [1]);
    assert_eq!(m.cmds_of(DEPCMD_SETSTALL).len(), 2);
}

#[test]
fn commands_and_replies_move_through_the_bounce_buffer() {
    let mut u = configured(&[Host::BulkOut(b"getvar:version".to_vec()), Host::BulkIn]);
    let mut cmd = [0u8; 64];
    let n = u.recv(&mut cmd).unwrap();
    assert_eq!(&cmd[..n], b"getvar:version");
    u.send(b"OKAY0.4").unwrap();
    let m = u.io();
    assert_eq!(m.bulk_in, [b"OKAY0.4".to_vec()]);
    assert_eq!(m.bulk_out_sizes, [512]);
    let starts = m.cmds_of(DEPCMD_STARTTRANSFER);
    let last = starts.last().unwrap();
    assert_eq!(last.0, 3);
}

#[test]
fn downloads_land_in_place_with_the_tail_bounced() {
    // 8 MiB + 512 + 3 bytes: one full TRB, one 512-byte TRB, a bounced tail.
    let len = XFER_MAX + 512 + 3;
    let data: Vec<u8> = (0..len).map(|i| (i * 7 + i / 512) as u8).collect();
    let mut u = configured(&[Host::BulkOut(data.clone())]);
    let mut buf = vec![0xeeu8; len + 16];
    u.recv_into(buf.as_mut_ptr() as usize, len).unwrap();
    assert!(buf[..len] == data[..], "download differs");
    assert!(buf[len..].iter().all(|b| *b == 0xee), "wrote past the end");
    assert_eq!(u.io().bulk_out_sizes, [XFER_MAX as u32, 512, 512]);
}

#[test]
fn a_short_download_is_a_transfer_error_and_terminal() {
    let mut u = configured(&[Host::BulkOut(vec![1; 700])]);
    let mut buf = vec![0u8; 2048];
    assert_eq!(
        u.recv_into(buf.as_mut_ptr() as usize, 2048),
        Err(Error::Transfer(700))
    );
    assert_eq!(u.io().regs[&DCTL] & DCTL_RUN_STOP, 0);
    assert_eq!(u.recv(&mut buf), Err(Error::Transfer(700)));
}

#[test]
fn a_bus_reset_ends_the_transfer_and_the_session_restarts() {
    let mut script = vec![Host::Reset];
    script.extend(enumeration());
    script.push(Host::BulkOut(b"continue".to_vec()));
    let mut m = Model::default();
    m.script.extend(enumeration());
    m.script.extend(script);
    let mut u = udc(m);
    u.wait_configured().unwrap();
    let mut cmd = [0u8; 64];
    assert_eq!(u.recv(&mut cmd), Err(Error::Reset));
    assert!(!u.configured());
    // The reset ended the bulk OUT transfer (ForceRM, its resource index).
    let ends = u.io().cmds_of(DEPCMD_ENDTRANSFER);
    assert_eq!(ends.len(), 1);
    assert_eq!(ends[0].0, 2);
    assert_ne!(ends[0].1 & DEPCMD_FORCERM, 0);
    // The reset cleared the device address.
    assert_eq!((u.io().regs[&DCFG] >> 3) & 0x7f, 0);
    u.wait_configured().unwrap();
    let n = u.recv(&mut cmd).unwrap();
    assert_eq!(&cmd[..n], b"continue");
}

#[test]
fn an_unread_reply_times_out_terminally() {
    let mut u = configured(&[]);
    u.io().allow_idle_end = true;
    assert_eq!(u.send(b"OKAY"), Err(Error::Timeout));
    assert_eq!(u.io().regs[&DCTL] & DCTL_RUN_STOP, 0);
    assert_eq!(u.send(b"OKAY"), Err(Error::Timeout));
    assert_eq!(u.poll(), Err(Error::Timeout));
}

#[test]
fn init_refuses_what_is_not_a_dwc3_and_bounds_every_wait() {
    let mut m = Model::default();
    m.regs.insert(GSNPSID, 0x4f54_280a);
    assert_eq!(
        Udc::init(m, plat(), cfg(), dma(), DESC).err(),
        Some(Error::NotDwc3(0x4f54_280a))
    );
    let mut m = Model::default();
    m.stuck_reset = true;
    assert_eq!(
        Udc::init(m, plat(), cfg(), dma(), DESC).err(),
        Some(Error::ResetTimeout)
    );
    let mut m = Model::default();
    m.never_halt = true;
    m.regs.insert(DCTL, DCTL_RUN_STOP);
    assert_eq!(
        Udc::init(m, plat(), cfg(), dma(), DESC).err(),
        Some(Error::HaltTimeout)
    );
}

#[test]
fn the_bootrom_session_is_stopped_and_its_state_dropped() {
    // Running, with an address and pending events left by the BootROM.
    let mut m = Model::default();
    m.stale_events = 3;
    m.regs.insert(DCTL, DCTL_RUN_STOP);
    m.regs.insert(DCFG, 0x0008_0000 | (5 << 3) | 4);
    let mut u = udc(m);
    let m = u.io();
    // Run/Stop cleared, the 12 stale bytes acknowledged before the halt.
    let first_dctl = m
        .evs
        .iter()
        .find_map(|e| match e {
            regtrace::Ev::W(32, a, v) if *a == (BASE + DCTL) as u64 => Some(*v),
            _ => None,
        })
        .unwrap();
    assert_eq!(first_dctl & DCTL_RUN_STOP, 0);
    assert!(m
        .evs
        .contains(&regtrace::Ev::W(32, (BASE + GEVNTCOUNT) as u64, 12)));
    // DCFG: High-Speed, address 0, LPM off; running again.
    assert_eq!(m.regs[&DCFG], 0x0008_0000);
    assert_ne!(m.regs[&DCTL] & DCTL_RUN_STOP, 0);
    // The first SETUP TRB is queued.
    let st = m.cmds_of(DEPCMD_STARTTRANSFER);
    assert_eq!(st.len(), 1);
    assert_eq!(st[0].0, 0);
}

#[test]
fn full_speed_hosts_get_64_byte_bulk_endpoints() {
    let mut m = Model::default();
    m.full_speed = true;
    m.script.extend(enumeration());
    let mut u = udc(m);
    u.wait_configured().unwrap();
    let m = u.io();
    assert_eq!(&m.controls[4].data[22..24], &[64, 0]);
    let bulk: Vec<_> = m
        .cmds_of(DEPCMD_SETEPCONFIG)
        .into_iter()
        .filter(|c| c.0 >= 2)
        .collect();
    assert!(bulk.iter().all(|c| (c.2[0] >> 3) & 0x7ff == 64));
}

#[test]
fn disconnect_quiesces_the_controller() {
    let mut u = configured(&[]);
    u.disconnect().unwrap();
    let m = u.io();
    // The queued SETUP transfer was ended; Run/Stop off; events masked.
    let ends = m.cmds_of(DEPCMD_ENDTRANSFER);
    assert_eq!(ends.iter().map(|c| c.0).collect::<Vec<_>>(), [0]);
    assert_eq!(m.regs[&DCTL] & DCTL_RUN_STOP, 0);
    assert_eq!(m.regs[&DEVTEN], 0);
    assert_ne!(m.regs[&GEVNTSIZ] & GEVNTSIZ_INTMASK, 0);
}

/// A device for the session test: remembers what it was asked to boot.
/// (The session test needs the `fastboot` feature, which provides
/// `fastboot::Transport` for `Udc`.)
#[cfg(feature = "fastboot")]
#[derive(Default)]
struct Dev {
    booted: Vec<u8>,
}
#[cfg(feature = "fastboot")]
impl fastboot::Device for Dev {
    fn getvar(&mut self, name: &str, out: &mut fastboot::Line) -> bool {
        use core::fmt::Write;
        name == "product" && write!(out, "yy3588").is_ok()
    }
    fn flash(&mut self, _: &str, _: usize, _: &mut [u8]) -> Result<(), &'static str> {
        Err("no")
    }
    fn boot(&mut self, image: &[u8]) -> Result<(), &'static str> {
        self.booted = image.to_vec();
        Ok(())
    }
}

#[cfg(feature = "fastboot")]
#[test]
fn a_fastboot_session_runs_over_the_controller() {
    let image: Vec<u8> = (0..1000u32).map(|i| (i ^ (i >> 3)) as u8).collect();
    let mut script = enumeration();
    script.extend([
        Host::BulkOut(b"getvar:product".to_vec()),
        Host::BulkIn,
        Host::BulkOut(b"getvar:max-download-size".to_vec()),
        Host::BulkIn,
        Host::BulkOut(b"download:000003e8".to_vec()),
        Host::BulkIn,
        Host::BulkOut(image.clone()),
        Host::BulkIn,
        Host::BulkOut(b"boot".to_vec()),
        Host::BulkIn,
    ]);
    let mut m = Model::default();
    m.script.extend(script);
    let mut u = udc(m);
    let mut d = Dev::default();
    let mut buf = vec![0u8; 0x10000];
    fastboot::serve(
        &mut u,
        &mut d,
        &fastboot::Config { unlocked: true },
        &mut buf,
    )
    .unwrap();
    assert_eq!(d.booted, image);
    let m = u.io();
    let replies: Vec<String> = m
        .bulk_in
        .iter()
        .map(|r| String::from_utf8_lossy(r).into_owned())
        .collect();
    assert_eq!(
        replies,
        [
            "OKAYyy3588",
            "OKAY0x00010000",
            "DATA000003e8",
            "OKAY",
            "OKAY"
        ]
    );
    assert_eq!(m.regs[&DCTL] & DCTL_RUN_STOP, 0);
    assert!(m.script.is_empty());
    // The 1000-byte download: one 512-byte TRB in place, the tail bounced.
    assert_eq!(&m.bulk_out_sizes[4..6], &[512, 512]);
    assert_eq!(trb::SIZE_MASK, 0x00ff_ffff);
}
