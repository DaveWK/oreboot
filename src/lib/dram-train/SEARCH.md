# CPU training search and control core

This is original, allocation-free Rust for bounded measurement search and
transactional application of selected settings. It is an intermediate component
for CPU-side training, **not an A733 PHY training driver**. It is not linked into
bt0, has no hardware register addresses, and does not run or copy PMU code.
`search::Unbound` returns `Unsupported` for every request before any access.

A reviewed backend must establish topology, stage order/dependencies, clock and
reset prerequisites, valid setting encodings and units, observation production,
mode-register command issue, safe result application and recovery. Current A733
MASTER CSR0x200a4 and pre-release CPU access remain unresolved. Do not infer
these from four configured DBYTEs, PMU access or older LPDDR4 definitions.

The search scans every requested discrete point and repeat; it intersects
fresh per-bit pass observations rather than averaging away failing bits. It
maximizes the area of an all-pass axis-aligned rectangle in **grid cells**,
then its smaller dimension and width; deterministic centers break ties.
Circular axes include seam-spanning windows without duplicating cells. The
objective is exact for the sampled grid. It is not an optimum physical
voltage/timing margin, an analog PHY model, or a measured maximum stable rate.
Boundary contact is reported because the physical eye may extend outside the
scan. A backend must choose and document meaningful physical domains.

`phase` adds original decision policies over backend-supplied domains. DFE and
DCA candidates are independent rows: adjacent coefficient/duty codes are never
assumed to form a continuous analog axis. Each candidate gets its own timing
eye; the widest stable all-bit eye wins, with explicit caller-supplied costs only
breaking equal-width ties. Constant-memory streaming selects each candidate
without a second512-byte grid. An independent cell-by-cell interval oracle checks
this objective. It is not a full multi-tap DFE adaptation or physical DCA loop.

Launch/poll counts and every scan/verify repeat are bounded. Tickets reject
stale completion; missing required-bit observations are errors. Selection is
applied and verified through separate probe requests. Every post-begin failure
attempts restoration; restoration failure is retained alongside the original
error. Backends themselves must bound entry, application, exit and recovery.
`session` validates exact explicitly listed targets, all nine stage labels in
each of four states, and same-target dependencies. It bounds the declared
launch/poll allowances, switches states through the backend, and requires
whole-session snapshot/restore. A later stage, transition or finalization
failure invalidates every output and attempts whole-plan recovery, retaining
both local and session recovery errors. The caller supplies and proves stage
order and physical topology; test order is not a proposed hardware sequence.
`session::run_phases` requires an explicit matching policy for every step and
checks availability/domain conformance before entry. A missing policy returns
`Unsupported`, with no automatic generic fallback.

| Feature | Original source behavior | A733 binding / acceptance |
|---|---|---|
| CA | Named joint timing/Vref all-bit window policy | CA training entry/exit, patterns, feedback and valid CSR fields unsupported |
| WCK2CK | Circular supplied-domain scan; original MR18 ratio/leveling bit helpers | Actual MRW/synchronization, PHY phase/sample semantics unsupported |
| Read gate | Named linear latency-domain policy, with caller-proven coarse/fine linearization | Gate detector/latency/coarse/fine semantics unsupported |
| Read centering | Named linear/circular timing-eye policy, per-lane requests and all-bit intersection | Read traffic, delay fields and actual measurement unsupported |
| Write centering | Named linear/circular timing-eye policy; separate verification request | Write traffic/readback and delay fields unsupported |
| Read/write Vref | Original 2D all-pass rectangle search; exact 7-bit DRAM Vref code/percentage and MR12/14/15 helpers | Exact part limits, receiver-Vref fields, command timing and apply sequence unsupported |
| DFE/DCA | Independent candidate rows; choose widest all-bit timing eye, explicit cost breaks ties | No multi-tap adaptation/physical duty algorithm, command/feedback/encoding or hardware binding |
| All four pstates | Explicit plan coverage/dependency/state-transition control; independently varied settings retained | No proven physical order, transitions, frequency units, full PMU traces or trained hardware |
| Result application | Apply + fresh verification; explicit session finalization; failed prefixes invalidated | No A733 register/result-storage or bt0 integration |
| Failure recovery | Timeout/budget/stale/missing/no-eye/verify/fault; per-stage and whole-plan recovery, independently failed recovery | Physical safe-state/reset/retraining remains unsupported |

Run `cargo test --offline -j1` and `cargo clippy --offline -j1 --all-targets -- -D warnings`.
The independent rectangle oracle enumerates every rectangle and constituent cell,
using a different algorithm from the production row-mask search. Test windows,
bit masks, latency and verification failures are explicit synthetic inputs.
They never supply topology or force completion in the PMU trace model.

`lpddr5` contains original value-only helpers corresponding to public primary
LiteDRAM commit `74522f715d7b10163ef6f6caa09887a90874be83`,
[`get_lpddr5_phy_init_sequence`](https://github.com/enjoy-digital/litedram/blob/74522f715d7b10163ef6f6caa09887a90874be83/litedram/init.py#L683)
(BSD-2-Clause FPGA PHY implementation). Vref conversion rejects off-grid
percentages and preserves the unrelated MR bit; MR18 changes only ratio and
leveling bits. Exhaustive controls cover every input byte and every u16
percentage input. No code was copied, no MRW is issued, and these protocol
fields do not establish A733 CSR meanings or DRAM-part operating limits.

`line` provides a separate original streaming trainer for up to256 explicit
settings, using constant interval state (at most64 bytes), without allocating
a grid. It intersects every required bit and repeat, joins circular suffix and
prefix windows once, and applies/freshly verifies the selected center with the
same bounded probe and rollback contract. It supports WCK, ordered read-gate,
and read/write centering requests through `LineBackend`; actual A733 requests
remain `Unsupported`. All-pass circles choose center0 because the supplied data
supports every phase equally. Linear boundary contact remains explicit.

The private static acquisition code contains128/212-iteration loops. This
motivates software capacity beyond64 but does not establish physical bin units,
period, polarity or the complete legal hardware domain. Host controls cover
128/212/256-point independent windows, four states, seam behavior, exhaustive
interval oracles, exact budgets and failure recovery.

`full_session::run` dispatches explicit long-line and grid/joint/candidate
policies within one complete transaction across all four states. Results retain
both variants, including span256 without narrowing. All policies, domains and
declared probe allowances are checked before entry and rechecked in the active
state. Mixed verification, transition, commit and domain failures invalidate
all outputs and attempt whole-session recovery. No automatic narrowing,
downsampling or generic fallback substitutes for the requested domain.

Production applicability is an explicit per-target `session::Applicability`
contract: required and excluded labels must partition all nine phases, match
the plan exactly, and cite evidence for every exclusion. The backend's read-only
preflight must actually validate those evidence identities against the reviewed
part, PHY, topology and state; a nonempty string alone is not proof. Missing
required phases remain errors. Comprehensive synthetic36-label and separately
reviewed synthetic34-label controls do not establish any physical exemption.
The original grid-session API keeps its comprehensive coverage requirement;
physical integration should use the reviewed mixed API.

`matrix` adds an original streaming joint timing/Vref policy for explicit domains
up to256 codes per axis. Vref remains linear; timing circularity is supplied
explicitly. Vertical run heights and a monotone histogram stack find the same
area/minimum-side/width/tie objective without storing65536 cells. The persistent
rectangle state is at most560 bytes; its transient histogram stack is1024 bytes.
Only CA/read-Vref/write-Vref requests use this joint policy. Candidate coefficient
rows remain independent and never become adjacent Vref codes by substitution.

`full_session::Policy::Matrix` and `Trained::Matrix` retain u16 widths/heights,
including span256, through complete mixed sessions. Independent small-domain
cell enumeration, full-size/circular/clipped/stack-capacity controls, separately
varied bit windows and recovery controls check these source semantics. These
software capacities are not actual legal A733/DRAM ranges; reviewed backend
domains, physical measurement semantics and part limits remain required.

## Explicit interval-record normalization

`interval_table` validates an immutable 132-byte software record containing 64
inclusive `(low, high)` rows and four opaque metadata/result bytes. The caller
must supply width and the exact reversed empty marker; there is no physical
255/0 or 127/0 default. Malformed reversed rows and out-of-domain endpoints fail
validation. This contract does not prove that captured endpoints imply every
interior cell passed, all-bit/repeat coverage, freshness, topology or units.

The original 64-row arithmetic geometry summary reports the rounded point
centroid and whether it belongs to the pass set. A centroid can fall in a gap;
it is not applied unchecked. Selection feeds the normalized cells to the
existing original all-pass rectangle objective, with both axes explicitly
linear. No captured output bytes or closed helper implementation are used.
Physical application still requires a reviewed backend, fresh verification and
whole-session recovery. No physical A733 backend is added.

Normalization and centroid each visit 64 pairs; selection performs at most
16,384 pushes using the existing constant-memory rectangle engine. Independent
point/rectangle oracles, centroid-gap, empty, malformed-marker, inclusive
256-width and metadata-preservation controls exercise this software contract.

## Raw acquisition command transaction

`command` implements the original synchronous parameter/settle/launch/poll/clear
transaction corresponding to the statically observed A733 acquisition interface.
The settling operation has no inferred physical duration. At most25,000 full
halfword status reads are allowed. A nonzero status remains `Unclassified`;
unknown error bits, a preexisting nonzero value and even a zero-to-nonzero
transition never become a fresh per-bit training measurement. This does not
close the physical command/feedback contract or connect an A733 trainer.

Unlike the observed native assertion/halt timeout path, the original policy
attempts one launch clear after an uncertain launch, read fault or timeout.
A failed clear invalidates output and quarantines the transaction against
automatic retry. This is a bounded software recovery policy, not proof of
physical status acknowledgement, analog reset or whole-stage rollback.

`Port` must validate safe exclusive idle ownership before mutation and bound
each operation. The only production port is `search::Unbound`, which rejects
before access. No MMIO mapping, selector topology, delay units, status polarity,
freshness generation or asynchronous command behavior is guessed. Synthetic
controls cover every nonzero status word, exact operation order and poll bounds,
unsupported/prelaunch faults, uncertain launch, timeout and failed-clear quarantine.

## Checked raw feedback routing

`feedback` is an original bounded selector/control piece derived from the exact
retained acquisition consumer. It accepts an explicit eight-bit active mask and
three numeric record-loop shapes, preserves logical record 9 as feedback slot 8,
and rejects duplicate selectors before access. Unlike setting routes, its packed
fields have no bank input. It supplies neither a CSR base nor CPU address. A
separately reviewed port remains mandatory; the actual A733 port is unsupported.

A batch preserves every raw halfword without masking or pass interpretation and
invalidates the whole output on preflight/read failure. At most 72 reads and one
preflight are issued; callbacks must themselves terminate. These observations
are not guaranteed simultaneous, fresh, physically valid, or training results.
No silicon count, pin map, polarity, units, or CPU access is inferred.


## Transactional DRAM Vref mode-register application

`vref_mr` is original bounded command-control source for up to twelve explicitly
listed CA/lower-DQ/upper-DQ Vref targets. It uses the existing LPDDR5 seven-bit
value helper and MR selector. Every target supplies explicit part limits; the
read-only port preflight must prove their exact part/FSP/topology/timing
applicability. Numeric ranges alone are not that proof. There is no default
FSP transition, training mode, clock, delay, register map or A733 port.

The engine snapshots every original byte before the first write and rejects
out-of-limit restore values. It preserves bit7, writes each selected target,
performs the port's bounded settling operation and requires fresh full-byte
readback. A failure invalidates the transaction and attempts restoration of
every possibly changed target, including a write that reported failure after
reaching DRAM. Restore errors remain separately per target, and any failed
restore quarantines automatic retry. A failed restore settle cannot authorize
a subsequent read; other targets still get independent restore attempts.
A port must enforce command ownership and safe recovery after faults itself.

At most twelve snapshot reads,36apply operations and36restore operations are
issued, plus one read-only preflight. Eleven source controls include all98,304
original-byte/Vref-code/kind combinations, exact command order, every normal
operation fault, readback mismatch, failed multi-target restoration, quarantine
and the84-operation worst case. Synthetic ports are software controls; they do
not prove physical command timing/freshness or make the A733 trainer usable.
The actual `search::Unbound` implementation rejects before access. This module is
not linked into bt0 and does not claim a complete Vref training scan or replace
the still-opaque PHY/PIE programs.
