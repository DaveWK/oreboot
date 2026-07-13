// SPDX-License-Identifier: GPL-2.0-only
use dram_train::search::{feedback::*, BackendError, Unbound};
use std::cell::Cell;

fn route(index: u8, record: u8) -> Route {
    Route::new(255, index, Records::Nine, record).unwrap()
}
#[test]
fn full_byte_input_domain_obeys_numeric_loop_shapes_and_active_mask() {
    for index in 0..=255u16 {
        for logical in 0..=255u16 {
            for mode in [Records::Eight, Records::Nine, Records::SpecialNine] {
                let accepted = index < 8
                    && match mode {
                        Records::Eight => logical < 8,
                        Records::Nine => logical < 9,
                        Records::SpecialNine => logical == 9,
                    };
                let actual = Route::new(255, index as u8, mode, logical as u8);
                assert_eq!(actual.is_ok(), accepted);
                if let Ok(r) = actual {
                    let slot = if mode == Records::SpecialNine {
                        8
                    } else {
                        logical
                    };
                    // Independent additive field oracle; the two fields and
                    // nominal base do not overlap over the accepted domain.
                    let nominal = 0x100b7u32 + u32::from(index) * 4096 + u32::from(slot) * 256;
                    assert_eq!(0x100b7 | u32::from(r.selector_bits()), nominal);
                    assert_eq!(r.index(), index as u8);
                    assert_eq!(r.logical_record(), logical as u8);
                    assert_eq!(r.slot(), slot as u8);
                    assert_eq!(r.selector_bits() & !0x7f00, 0);
                }
            }
        }
    }
    for mask in 0..=255u8 {
        for index in 0..8 {
            for logical in 0..9 {
                assert_eq!(
                    Route::new(mask, index, Records::Nine, logical).is_ok(),
                    (u16::from(mask) / (1u16 << index)) % 2 == 1
                );
            }
        }
    }
}

#[test]
fn complete_collision_domain_rejects_special_nine_alias_and_duplicate_labels() {
    let routes: Vec<_> = (0..8)
        .flat_map(|i| (0..9).map(move |s| route(i, s)))
        .collect();
    assert_eq!(routes.len(), MAX_ROUTES);
    assert!(Plan::new(&routes).is_ok());
    for a in &routes {
        for b in &routes {
            assert_eq!(a.selector_bits() == b.selector_bits(), a == b);
        }
    }
    for index in 0..8 {
        let special = Route::new(255, index, Records::SpecialNine, 9).unwrap();
        for ordinary in &routes {
            let pair = [special, *ordinary];
            assert_eq!(
                Plan::new(&pair).is_err(),
                ordinary.index() == index && ordinary.slot() == 8
            );
        }
        assert_eq!(
            Plan::new(&[special, special]).err(),
            Some(RoutingError::Aliased)
        );
        assert_eq!(special.slot(), 8);
        assert_ne!(special.logical_record(), special.slot());
    }
    let mut complete = routes.clone();
    complete.extend((0..8).map(|index| Route::new(255, index, Records::SpecialNine, 9).unwrap()));
    for a in &complete {
        for b in &complete {
            let expected_alias = a.index() == b.index() && a.slot() == b.slot();
            assert_eq!(a.selector_bits() == b.selector_bits(), expected_alias);
            assert_eq!(Plan::new(&[*a, *b]).is_err(), expected_alias);
        }
    }
    assert_eq!(Plan::new(&[]).err(), Some(RoutingError::Empty));
    assert_eq!(
        Plan::new(&[route(0, 0); MAX_ROUTES + 1]).err(),
        Some(RoutingError::TooMany)
    );
}

struct Model {
    preflights: Cell<usize>,
    seen: Vec<Route>,
    raw: u16,
    reject: bool,
    fail_at: Option<usize>,
}
impl Model {
    fn new(raw: u16) -> Self {
        Self {
            preflights: Cell::new(0),
            seen: Vec::new(),
            raw,
            reject: false,
            fail_at: None,
        }
    }
}
impl Port for Model {
    fn preflight(&self, _: &Plan<'_>) -> Result<(), BackendError> {
        self.preflights.set(self.preflights.get() + 1);
        if self.reject {
            Err(BackendError::Unsupported)
        } else {
            Ok(())
        }
    }
    fn read_raw(&mut self, route: Route) -> Result<u16, BackendError> {
        self.seen.push(route);
        if self.fail_at == Some(self.seen.len()) {
            Err(BackendError::Fault)
        } else {
            Ok(self.raw)
        }
    }
}
#[test]
fn all_halfwords_remain_unmasked_and_unclassified_including_zero_high_bits() {
    let routes = [route(7, 8)];
    let plan = Plan::new(&routes).unwrap();
    let mut output = [None; MAX_ROUTES];
    for raw in 0..=u16::MAX {
        let mut port = Model::new(raw);
        observe(&mut port, &plan, &mut output).unwrap();
        let observed = output[0].unwrap();
        assert_eq!(observed.raw_word, raw);
        assert_eq!(observed.is_zero(), raw == 0);
        assert_eq!(observed.route, routes[0]);
        assert_eq!(port.seen, routes);
        assert_eq!(port.preflights.get(), 1);
        assert!(output[1..].iter().all(Option::is_none));
    }
}
#[test]
fn max_batch_is_ordered_bounded_and_each_fault_invalidates_successful_prefix() {
    let routes: Vec<_> = (0..8)
        .flat_map(|i| (0..9).map(move |s| route(i, s)))
        .collect();
    let plan = Plan::new(&routes).unwrap();
    let mut output = [None; MAX_ROUTES];
    let mut port = Model::new(0x8000);
    observe(&mut port, &plan, &mut output).unwrap();
    assert_eq!(port.seen, routes);
    assert_eq!(port.preflights.get(), 1);
    assert!(output.iter().all(|v| v.unwrap().raw_word == 0x8000));
    let previous = output;
    for fail in 1..=MAX_ROUTES {
        let mut port = Model::new(0);
        port.fail_at = Some(fail);
        output = previous;
        assert_eq!(
            observe(&mut port, &plan, &mut output),
            Err(Failure::Port(BackendError::Fault))
        );
        assert_eq!(port.seen.len(), fail);
        assert!(output.iter().all(Option::is_none));
    }
    let mut port = Model::new(7);
    port.reject = true;
    output = previous;
    assert_eq!(
        observe(&mut port, &plan, &mut output),
        Err(Failure::Port(BackendError::Unsupported))
    );
    assert!(port.seen.is_empty());
    assert!(output.iter().all(Option::is_none));
}
#[test]
fn unbound_port_remains_unsupported_before_any_feedback_read() {
    let routes = [route(0, 0)];
    let plan = Plan::new(&routes).unwrap();
    let mut out = [Some(Unclassified {
        route: routes[0],
        raw_word: 7,
    }); MAX_ROUTES];
    assert_eq!(
        observe(&mut Unbound, &plan, &mut out),
        Err(Failure::Port(BackendError::Unsupported))
    );
    assert!(out.iter().all(Option::is_none));
    assert_eq!(core::mem::size_of::<Route>(), 3);
    assert_eq!(core::mem::size_of_val(&out), 576);
}
