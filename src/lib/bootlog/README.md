# bootlog: firmware RAM log and boot progress

The ring format below is the one the A7A firmware writes (format `A7A` in
`bootlog::ring`); other boards choose their own addresses and sizes through
`bootlog::Area`. The reader is `tools/fwlog` (formerly the A7A's
`a7a-fwlog`). Moved from ore-edk-boot-radxa-a7a `docs/fwlog.md` at `ed42550`
(inventory row A7A-INV-88); the addresses are the A7A's.


Everything oreboot bt0, oreboot main and EDK2 print on UART0 is also kept in RAM, so a
boot can be read back from Linux when no UART is attached (the 2×2.5 GbE HAT covers the
header). On the board: `tools/fwlog` (`fwlog`) prints this boot's firmware log, `fwlog -p` the
previous boot's.

## Where

| Area | Address | Size |
|---|---|---|
| this boot | `0x40140000` | 256 KiB |
| previous boot | `0x40180000` | 256 KiB |
| bt0's buffer before DRAM is trained | SRAM `0xF0000` | 16 KiB |

Both DRAM areas lie in the first 2 MiB of DRAM, which oreboot main keeps for itself. That
range is never in the memory map EDK2 or the OS get, so nothing else writes it, and Linux
can map it through `/dev/mem` even with `STRICT_DEVMEM`. EDK2 publishes the region as
`/chosen/oreboot,firmware-log = <base size>` (two 64-bit cells) in the OS devicetree.

## Format

Each area starts with a 64-byte header (little endian), followed by the ring data:

| Offset | Size | Field |
|---|---|---|
| 0 | 8 | magic `"A7AFWLOG"` |
| 8 | 4 | version, 1 |
| 12 | 4 | data size: area size − 64 |
| 16 | 8 | bytes written so far; the next byte goes to `written % size` |
| 24 | 4 | boot sequence number, +1 for each boot while the log survives |
| 28 | 36 | flags and reserved, 0 |

When `written` exceeds the data size, the ring has wrapped and the oldest
`written − size` bytes are gone.

## Who writes

1. **bt0** prints into the SRAM buffer until DRAM is trained. Right after training it
   copies the previous boot's area, if its header is still valid, to the previous-boot
   area. It then starts a new log (sequence number + 1) and moves the SRAM output into it.
   DRAM loses refresh during a reset, so whether the previous log survives a watchdog or
   warm reset is best effort; after a power cycle it is gone.
2. **oreboot main** appends to the log bt0 started (`fwlog::attach`). That includes
   anything secondary CPUs print later through PSCI, while the OS runs.
3. **EDK2** (`OrebootLogSerialPortLib`) appends everything it sends to the UART. Before
   its first write it asks oreboot main for the base with SiP SMC `0x82000002`. Under a
   vendor U-Boot chainload the vendor BL31 answers NOT_SUPPORTED, and EDK2 never writes:
   that range is then ordinary RAM, even if it still holds a valid-looking header from an
   earlier oreboot boot. EDK2 maps the region uncached, so readers see every byte.
   Runtime services print nothing after ExitBootServices.

## Who reads

`tools/a7a-fwlog/a7a-fwlog.c`, shipped in the board RPM, maps the region read-only and uses
byte loads (Linux maps it as device memory). `-f DUMP` reads a file dump of the region
instead, for tests.

Sources: oreboot `common/src/fwlog.rs`, the UART writer in `common/src/uart.rs`, EDK2
`Library/OrebootLogSerialPortLib` and `CubieA7ALib` (mapping), `FdtPlatformDxe` (the
`/chosen` property).
