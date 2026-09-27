# primer-ddr3-controller

Improve the open source DDR3 controller for the Tang Primer 20K so it can carry
real workloads. This is a standalone project: it starts from our fork of the
upstream controller and builds out from there. No other project's tree, docs or
conventions are in scope here.

- our fork (canonical, this repo): https://github.com/mobilinkd/ddr3-tang-primer-20k
- upstream: https://github.com/nand2mario/ddr3-tang-primer-20k (Apache-2.0,
  dormant: 5 commits, no tags, no releases)

## Minimum required throughput: 400 MB/s sustained

**400 MB/s sustained is the bar this project exists to clear.** It is the
working baseline for the controller, and any change that regresses it is a
regression, full stop.

Where the number comes from: 398.25 MB/s is what the OLD Gowin vendor DDR3 core
actually delivered on this board, and 398.74 MB/s is the LSTM weight burst an
audio-codec consumer needed (526,336 B in 1.32 ms). 400 MB/s is that bar rounded
up. It is a rate hardware has hit on this part, not a wish.

It is hard: 400 MB/s is 25% of DDR3-796's pin peak (~1593 MB/s) and about 50% of
a continuous BL8 stream at 796 MB/s.

### Where it stands today

Measured on hardware, Tang Primer 20K (`GW2A-LV18PG256C8/I7`, device version C):

| direction | rate | bytes/command | pclk/command |
|---|---|---|---|
| write | 28.21 MB/s | 2 | 6.0 floor (85% of the 2 B/cmd ceiling) |
| read | 159.30 MB/s | 16 (`dout128`) | 10.0 |

**Write is the binding direction**: roughly 14x short of 400 MB/s, against 2.5x
on read. Both rates are command-rate limited, not bandwidth limited. The
controller takes one 16-bit word per write command and returns one 16-byte burst
per read command, with no command queue, no pipelining and no backpressure
handshake. Throughput here is commands per second, so anything that lowers
per-command cost or puts multiple commands in flight is the shape of the fix.

Caveat carried forward: both rates are a command count multiplied by a
bytes-per-command figure, and the command count carries an unresolved 1.0132
scale factor. The on-chip command counter has to be readable over the UART
before either number is treated as settled.

## Repository layout

```
src/          controller RTL (ddr3_controller.v, ddr3_top.v, gowin_rpll/,
              print.v, uart_tx_V2.v), constraints (tang20k.cst), timing (ddr3.sdc)
simulation/   iverilog + gtkwave simulation of the DDR3 signalling
doc/          upstream's build screenshot
ddr3.gprj     upstream's Gowin project file
```

The controller is a soft core in fabric, not hard IP: it costs 1,508 LUT (7%),
2,830 REG (18%), 8 BSRAM, 0 DSP. DDR3-800 comes from the hard I/O-ring
primitives (OSER8_MEM/DQS, IDES8_MEM, IODELAY), not from the soft logic.

## Delta against upstream

One file so far, with the change notice in the file header per Apache-2.0
section 4(b):

| file | vs upstream `a6d866d` |
|---|---|
| `src/ddr3_controller.v` | **modified**: added the `accept` output |

`accept` is a one-cycle pulse at the IDLE -> READ/WRITE transition, i.e. exactly
one pulse per command the controller actually takes in. `wr`/`rd` are LEVEL
signals held high for the whole busy window, so a count derived from them
over-counts by ~3.35x. Every throughput number divides by `accept`, not by
busy-cycles.

## Two things about the current datapath, stated once

1. **Reads already return 16 bytes per command.** `dout128` is
   `{dq_in[0]..dq_in[7]}`, eight 16-bit lanes, fully populated and live.
   Verified by simulation on the unmodified controller: 400 commands, 400
   `data_ready` edges, 4045 distinct values, 128 of 128 bits ever driving 1,
   zero X. The read datapath is not the problem.
2. **Write is the problem.** BC4 with DM masking 3 of 4 beats means the useful
   write payload is genuinely one 16-bit word per command
   (`src/ddr3_controller.v:388-421`). Write busy negates at 24 pclk in the RTL
   comment (`:402`); the measured floor is 6.0 pclk. At 99.5625 MHz even a
   perfect one-command-per-cycle loop at 16 B/cmd is 1593 MB/s, so 400 MB/s is
   reachable only by getting more bytes per command AND more commands in flight.

## Build and bench

- The vendor toolchain runs in the podman image `localhost/fpga-tools:latest`.
  Never call the vendor binaries bare; go through a container wrapper so the
  toolchain stays reproducible.
- The board is on this host's bench: JTAG is `/dev/tang-jtag` (ttyUSB1), UART is
  `/dev/tang-uart` (ttyUSB2). The logic analyzer and the bench PSU are local
  too.
- One process at a time may touch the bench. Two concurrent flashes, or a flash
  racing a UART reader, wedge the FT2232 and cost hours. Take the exclusive lock
  before any bench operation.
- DIP switch 1 must be DOWN or JTAG hangs.
- LED indices in the `.cst` are MIRRORED versus the silkscreen; name both forms
  when reporting an LED.
- Gowin traps, learned the hard way: never ignore a synthesis warning (an
  invalid config usually does not fail the build, the tool substitutes a legal
  value and the chip runs at the wrong rate, so a green build is not evidence
  the design is right), and run long vendor builds in the background rather than
  on a foreground timeout.

## Provenance and licensing

Upstream is Apache-2.0 and its `LICENSE` is at the repository root. This fork is
not a drop-in replacement for the vendor core: it exposes a different,
lower-level user port (`rd`/`wr`/`refresh`/`busy`, plus `accept`), with no
ready/valid handshake, so any consumer written against the vendor app-port
backpressure contract needs an adapter.

---

<details>
<summary>Upstream README (nand2mario, 2022.9), preserved</summary>

# Simple low-latency DDR3 PHY controller for Tang Primer 20K

This is a DDR3 controller for GW2A / Tang Primer 20K. It was designed for
[NESTang](https://github.com/nand2mario/nestang) and should hopefully be useful
for other projects.

Unlike more portable designs, we use Gowin OSER8_MEM/DQS primitives for running
at higher speeds (DDR3-800). We are aiming mostly at low-latency use cases like
FPGA gaming. The achieved read latency is about 90ns. The interface is single
16-bit word based and uses no bursting. For more predictable behavior, the
controller also exposes a *refresh* input for executing auto-refreshes, avoiding
the longer latencies introduced by controller-initiated refreshes. Resource
usage is 1377 logic elements (6% on GW2A-18C).

DDR3 requires a fair amount of setting-ups to function properly. In particular,
it needs dynamic adjustments to clock timings to make reads/writes more stable.
Here are the implemented mechanisms: ZQ calibration, writing leveling, read
calibration and dynamic ODT.

The official documentation from Gowin is quite lacking for DDR-related
primitives like DQS, IDES8_MEM and OSER8_MEM. The dev process thus involved
quite some trial-and-errors and cross-checking with other vendors' docs. So the
code also serves as examples/documentation for these constructs.

Test screenshot: `doc/screenshot.png`

There's also an iverilog and gtkwave-based simulation. See
[simulation instructions](simulation/README.md).

Build instructions,
* Gowin IDE 1.8.0.7
* Project->Configuration->Synthesis: set Verilog language to SystemVerilog
* Project->Configuration->Dual Purpose Pin: Use SSPI as regular IO

nand2mario, 2022.9

</details>
