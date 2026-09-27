# Design note: reaching 400 MB/s sustained

**Author:** dave **Date:** 2026-09-27 **Branch:** `feat/400mbs`
**Status:** proposal, for review before implementation. No RTL in this commit.
**Bar:** 400 MB/s sustained, read and write, on Tang Primer 20K
`GW2A-LV18PG256C8/I7` device version C, `pclk = 99.5625 MHz`, `fclk = 398.25 MHz`.

---

## 0. The answer in one paragraph

Write and read are both **command-rate limited**, and both are limited by the
same three things: the controller puts **2 bytes** in a write command, takes
**exactly one command in flight**, and **auto-precharges the row on every
command**. The design point below changes all three: **16 bytes per command in
both directions** (BL8 instead of BC4+DM, with an 8-word write combiner in
front so the consumer's 2-byte port is unchanged), a **depth-8 command queue**
and a **depth-8 read-response FIFO** so several commands are in flight, and a
**row-open tracker** so the row is activated once per 2048 B instead of once
per command. At a 3 pclk/command issue cadence that is **531.00 MB/s
theoretical** in both directions against a 400 MB/s requirement — a 1.33x
design margin, and 1.5x more DRAM headroom than that (the DRAM itself will take
637.20 MB/s of writes and 796.50 MB/s of reads at BL8 with the row open).

---

## 1. Clock domains — a trap in the current numbers, corrected first

`fclk = 4 x pclk`, and the memory clock `ck` is a phase-shifted `fclk`. So:

| quantity | value |
|---|---|
| `pclk` | 99.5625 MHz (10.044 ns) |
| `fclk` = `ck` | 398.25 MHz (2.511 ns) |
| 1 pclk | **4 nCK** |
| pin bandwidth (x16, DDR, BL8) | 16 B/nCK x 398.25 MCK/s = **1593.00 MB/s = 16.00 B/pclk** |
| 400 MB/s | **4.0176 B/pclk = 25.11 % of pin bandwidth** |

**The brief's "busy negates at 24 pclk for write (`:402`, measured floor 6.0)"
is a unit error and must not be used.** The comment at `ddr3_controller.v:398`
("negedate at 24") is in **nCK**, not pclk, and the code negates `busy` at
`{WRITE, FIVEB'(20/4)}` = cycle 5 (`src/ddr3_controller.v:435`) — i.e. 20 nCK =
**5 pclk** of busy window, **6 pclk** accept-to-accept. Building the write
budget on 24 pclk would be wrong by 4x. Every number below is derived in
**pclk**, and nCK is given alongside because the DRAM limits are in nCK.

## 2. Where today's numbers come from, and the 1.0132

| direction | measured | B/cmd | pclk/cmd | derivation |
|---|---|---|---|---|
| read | 159.30 MB/s | 16 | **10.0000** | `(RCD+CAS+SERDES)/4+2 = 9` -> accept-to-accept 10. Exact to 4 figures. |
| write | 28.21 MB/s | 2 | **7.0587** | FSM cost is 6 pclk (`:435`); the extra ~1.06 pclk is the one-cycle handshake bubble in the top-level bulk loops, `ddr3_top.v:256` and `ddr3_top.v:283` (`work_counter` re-arm). |

`2 B / 7 pclk` = 28.446 MB/s, which is **1.0084x** the measured 28.21 MB/s. The
residuals sit in the same place as the unexplained **1.0132** on the command
count.

**Root cause, and it is not a mystery: there is no on-chip command count.**
`accept` is declared at `ddr3_controller.v:100` and driven at `:344`, but
`ddr3_top.v` **does not connect it** — the output is dangling at the top level
and reaches no pin. The only command counter in the design
(`cnt_read`/`cnt_write`, `ddr3_controller.v:147-149`) is 8 bits and **saturates
at 255**, and `debug` is wired to an unconnected wire in `ddr3_top.v:65,79`.
So the denominator of the two published rates was never measured; it was
inferred from a level signal, which is exactly the quantity the brief says
over-counts. **The 1.0132 is the error of that inference, and it cannot be
resolved by analysis — only by measuring `accept` and a pclk counter on-chip**
(step 2 of the dispatch, and section 6 below).

This note's design point is chosen with a margin wide enough that a 1.3 %
denominator error cannot decide pass/fail: 531.00 MB/s design point vs a
400 MB/s bar is 32.75 % of headroom.

## 3. (a) The arithmetic that reaches 400 MB/s

A rate is `(bytes/command) / (pclk/command) x 99.5625e6`. At 16 B/command:

| pclk/command | nCK/command | MB/s | vs 400 MB/s |
|---|---|---|---|
| 2.0 | 8 | 796.50 | DRAM-illegal for write (see below) |
| 2.5 | 10 | 637.20 | **write DRAM floor** (burst 4 nCK + tWR 6 nCK) |
| **3.0** | **12** | **531.00** | **chosen: 1.33x margin, legal in both directions** |
| 4.0 | 16 | **398.25** | **FAILS by 0.44 %** |
| 5.0 | 20 | 318.60 | fails |
| 6.0 | 24 | 265.50 | fails |

> **The 4 pclk/command row is the trap to avoid.** 16 B every 4 pclk is the
> obvious "obvious" choice, it satisfies every DRAM timing rule, and it measures
> 398.25 MB/s — 0.44 % short of the bar. Anything that lands the issue cadence
> on a round 4 pclk fails acceptance. The cadence is **3 pclk**, which is
> 12 nCK, above the 10 nCK write floor and with 1.5x of slack on the read path.

### DRAM floor per command, BL8 = 4 nCK of data

| policy | per-command nCK | pclk | MB/s | verdict |
|---|---|---|---|---|
| write, row open | 4 burst + 6 tWR = **10** | 2.50 | 637.20 | OK |
| read, row open | 4 burst + 4 tCCD = **8** | 2.00 | 796.50 | OK |
| write, auto-precharge every command | 4+6 tWR+6 tRP+1 ACT+6 tRCD = **23** | 5.75 | 277.04 | **SHORT** |
| read, auto-precharge every command | 4+6 tRP+1 ACT+6 tRCD = **17** | 4.25 | 374.82 | **SHORT** |

So **row-open is mandatory, not an optimisation**: keeping the current
auto-precharge behaviour caps write at 277 MB/s and read at 375 MB/s no matter
how good the command pipeline is. It is the precondition for every other lever.

### Refresh cost at the design point

tREFI = 7.8 us = 776.6 pclk; tRC = 48.75 ns = 4.85 pclk. Row size =
2^10 words = 2048 B = 128 commands, so one refresh every 7.8 us interrupts
about 2 rows of traffic. Adding the mandatory **precharge-all** that row-open
makes necessary (1 nCK) plus tRC plus the re-ACT/tRCD after refresh:
`(1 + 4.85 + 1.5) / 776.6` = **0.95 % throughput loss**. Negligible, and it is
paid, not avoided.

### The four candidate directions: verdicts

1. **More bytes per write command — ACCEPT, via write combining.**
   BC4 with `dm_out` masking 3 of 4 beats (`ddr3_controller.v:419,431-433`)
   really does deliver 2 useful bytes. Switching the write command to **BL8**
   (`A[12] = 1`, `dm_out = 0`) delivers 16. `MR0.M_BL = 2'b01` is already
   "BL4/8 selected by A12 on the fly" (`:196`), so **no mode register change
   and no init sequence change** is needed — this is a datapath change only.
   BL8 needs an 8-word-aligned DRAM column, so the controller gets an **8-word
   write combiner**: the app keeps pushing 16-bit words, and 8 *contiguous
   aligned* words become one BL8 command. **The consumer's port does not
   change.** Misaligned or non-contiguous sequences fall back to today's BC4+DM
   path, unchanged, so nothing loses correctness. This one change is 8x on
   write on its own.
2. **More than one command in flight — ACCEPT.** This is the other 8x (write)
   and 2.5x (read). A depth-8 command queue plus a depth-8 read-response FIFO
   and an in-order retire path. Justified in section 5 by Little's law.
3. **Lower per-command cost — REJECT as a standalone lever.** Even a perfect
   zero-cost command at today's 2 B/command is 199 MB/s, still 2x short; and
   the write busy window is already 5 pclk of 6, so there is 1 pclk to win out
   of 6. For read the 10 pclk is dominated by `SERDES = 16` pclk
   (`ddr3_controller.v:118`), which is a *latency* term, not a cost term: it
   is only paid once per command because nothing else is in flight to fill it.
   Pipelining (direction 2) removes it from the throughput path. Chasing
   per-command cost directly is a much smaller win at much higher risk.
4. **Keeping the row open — ACCEPT, and it is the precondition** (table
   above). The cost is real and is carried explicitly: with auto-precharge
   removed, the "no need for precharge-all b/c all our r/w are done with
   auto-precharge" assumption at `ddr3_controller.v:352` **becomes false**, and
   the controller must issue a precharge-all before every `refresh` (DDR3
   requires all banks precharged before REF) and must re-activate afterwards.
   This is a correctness change, not an optimisation.
5. **(my addition) Bank rotation — REJECT.** Rotating across the 8 banks to hide
   tRP/tRCD is the standard next trick, but at 25 % pin utilisation the row is
   open 128 consecutive commands and there is no tRP/tRCD to hide. It would add
   bank-conflict scheduling to both queues for no gain. Revisit only if the
   target rises above ~600 MB/s.

### Resource estimate

The controller is 1,508 LUT / 2,830 REG / 8 BSRAM today. The additions: an
8-entry x {26-bit address, 1-bit type} command queue (~40 REG), an 8 x 128-bit
write combiner (2 BSRAM or 128 REG), an 8 x 128-bit response FIFO (2 BSRAM),
a 26-bit open-row register, and 4 counters x 32 bit. BSRAM 8 -> 12, REG
~2,830 -> ~3,200, LUT roughly flat. On GW2A-18C (12,504 LUT / 65,536 REG) this
is comfortable and leaves the design well inside the part.

## 4. (b) The app-port contract change

Today the port is a level request with no handshake: `rd`/`wr` pulses, `busy`
says when the controller is free, one shared `dout128` register, one
`data_ready` pulse. That shape cannot express "I have 6 commands outstanding",
which is the whole point of direction 2.

**Added** (all new, all defaulted so the module still elaborates for an old
consumer):

| port | dir | meaning |
|---|---|---|
| `cmd_valid` | in | a command is offered this cycle |
| `cmd_addr[25:0]` | in | word address, as today |
| `cmd_is_write` | in | 1 = write, 0 = read |
| `cmd_data[15:0]` | in | write data, one 16-bit word, as today's `din` |
| `cmd_ready` | out | the command was taken **this cycle** (one per `accept`) |
| `rdata[127:0]` | out | one 16-byte read response |
| `rvalid` | out | response is on `rdata` this cycle |
| `rready` | in | consumer took the response |
| `rbeat[2:0]`, `rbeat_last` | out | which of the 8 words of `rdata` the 2-byte consumer is on |
| `cmd_count_wr`, `cmd_count_rd`, `refresh_count`, `pclk_count` | out | 32-bit measurement counters (section 6) |

**Unchanged:** `rd`, `wr`, `refresh`, `addr`, `din`, `dout`, `dout128`,
`data_ready`, `busy`, `accept`, all DDR3 pins, all debug/calibration ports.

**Compatibility rule:** if `cmd_valid` is tied low, the module runs in legacy
mode — a `rd`/`wr` pulse is internally converted into a one-entry command
offer, `dout128`/`data_ready` behave exactly as today. So a consumer written
against the old port compiles and runs unmodified, at the old speed, with no
adapter. The new port is what buys the 531 MB/s.

**Sequencing the consumer must know:**

- `cmd_ready` is the only backpressure on the command side. `cmd_valid` must
  not be withdrawn once asserted, and a consumer that holds `cmd_valid` high
  for many cycles sees `cmd_ready` pulse for exactly the cycles taken.
- A read response is 8 words. `rvalid`/`rready` is a per-**burst** handshake;
  `rbeat` walks 0..7 so a 2-byte consumer drains 8 cycles per burst without
  changing the command port.
- **Order is preserved in a single unified queue.** A read issued after a write
  to the same address returns that write's data. The cost is that a
  fine-grained read/write interleave stalls write coalescing: any pending read
  forces a flush of the partially-filled combiner through the BC4 path so
  ordering holds. A streaming workload (the target) never hits this; a
  random-access workload pays for correctness. This is a deliberate choice
  over split read/write queues, which would buy read/write overlap at the cost
  of needing write-forwarding and hazard logic.
- `busy` is retained and means "the command queue and response FIFO are
  saturated", not "one command is in flight". Existing code that polls `busy`
  still works, just with different meaning under load.

## 5. (c) Effect on read and write latency

**Read: latency goes up, deliberately and unavoidably.** Little's law, in-flight
bytes = rate x latency:

| | rate | latency | in flight | commands in flight |
|---|---|---|---|---|
| today | 159.30 MB/s | ~90 ns | 14.3 B | **0.9** |
| required | 400 MB/s | same 90 ns | 36 B | 2.25 — *not achievable* |
| required | 400 MB/s | 176 ns | 70.3 B | **4.39** |

At 400 MB/s the issue-to-data latency is not negotiable: CAS 6 nCK (1.5 pclk)
+ SERDES 16 pclk (161 ns) = **176 ns** minimum, and that latency must be
covered by outstanding commands, so 4.39 BL8 bursts are in flight. Hence:

- **FIFO depth 8** (2x the Little's-law minimum) to absorb jitter, refresh
  preemptions and row misses.
- First-word latency becomes **~176 ns minimum, ~390 ns worst case** (7 queued
  x 3 pclk) against ~90 ns today: a **2-4.4x increase for a 2.5x bandwidth
  gain**. This is the real trade and it is inherent — 400 MB/s at 16 B/command
  *is* 3.98 pclk/command, and a 176 ns SERDES round trip cannot hide inside
  that without ~4.4 outstanding commands. A consumer that needs low latency
  picks a shallower queue and a proportionally lower rate; `FIFO_DEPTH` is a
  parameter.
- Read-to-read latency for a single in-order stream is unchanged in *shape*:
  responses retire in order, so it is a uniform 176-390 ns, not a reordering
  hazard. No reordering buffer is needed and none is planned.

**Write: app-visible latency gains the coalescing delay.** On the wire a write
burst still starts ~3 pclk after its command (unchanged). But the app's first
word of a burst is now held until the 8th aligned word arrives (8 app cycles)
and then queues behind up to 7 other commands. Net: **+8 to +21 pclk (80-210 ns)**
app-visible, against a **14.1x** throughput gain. Point writes pay this and
streaming writes do not notice it.

**Memory-level latency is unchanged**: tRCD, tRP, tWR and CWL are all unchanged
— the pipeline changes when commands are *issued*, not what the DRAM sees per
command. The DRAM still sees BL8 bursts at the same phase relationship to CK.

## 6. (d) How this will be measured

**The primary measurement never uses the UART as a clock.** That is the fix for
the 1.0132, not a workaround for it.

1. **On-chip instrumentation.** Four 32-bit counters in the controller, all
   incremented on the same edge: `pclk_count` (free running), `cmd_count_wr`
   and `cmd_count_rd` (one per **`accept`** pulse, i.e. one per command
   actually taken in — never per `busy` cycle), and `refresh_count`.
2. **Markers over UART.** The bench top prints, in decimal, before and after
   each bulk phase:
   `BEGIN <phase> <bytes>` / `END <phase> <pclk_count> <cmd_count_wr>
   <cmd_count_rd> <refresh_count>`.
   `print.v` already has the multi-stage decimal print state machine
   (`ddr3_top.v:405-424`) — this extends that pattern, it does not invent a
   new one.
3. **The rate.** `rate = (delta_cmd_count * bytes_per_command) /
   delta_pclk_count * 99.5625e6`, with `bytes_per_command` stated next to it
   (16, and independently confirmed from `A[12]` in the capture). The UART
   carries *numbers*; the pclk counter carries *time*. A 115200-baud
   quantisation cannot enter the result.
4. **Decoder, committed and re-runnable.** `tools/decode_uart.py` parses a raw
   capture into `evidence/*.json` and prints the rate with both factors named
   and the exact invocation. It is committed under `tools/` (source, per rules
   §2), the captures and its output under `evidence/` (tracked, per the
   acceptance criteria).
5. **Independent cross-check: the wire.** A logic-analyzer capture of the DDR3
   bus (`ck`, `DQS`, `DQ`, `nRAS/nCAS/nWE`, `A`, `BA`) decoded by a committed
   sigrok protocol decoder that counts ACT and READ/WRITE column commands and
   counts `A[12]` to confirm BL8. This measures the denominator on the wire,
   with no RTL counter and no UART in the path, so it settles the 1.0132
   question independently of the RTL. If the analyzer's channel map is not
   available, this becomes a documented gap, not a silent omission.
6. **Two bulk sizes that must agree within 5 %:** 1 MiB and 8 MiB
   (65,536 and 524,288 commands at 16 B/command), both whole multiples of the
   2048 B row so no partial row skews the result. Read and write reported
   separately, each from both sizes, each stating `cmd_count` and
   `bytes_per_command`.
7. **Data is never constant** (rules §7): `din` carries a changing pattern and
   the bulk verify compares **all 128 bits** of every `dout128`, not the low
   byte the current test checks (`ddr3_top.v:313`).

## 7. What is missing before RTL can start (found while preparing this note)

These are environment facts, flagged now rather than discovered at hour three:

1. **`simulation/` cannot build as shipped.** `make run.controller` needs
   `ddr3.v`, `1024Mb_ddr3_parameters.vh`, `tb.v`, `subtest.vh` (Micron's
   model) and `prim_sim.v` (Gowin's primitive models). **None are in the repo**
   and the README asks you to download them. To keep the tree self-contained
   and reproducible I will write my own behavioural models: a DDR3 x16 model
   (ACT/PRE/REF/MRS, BL4/BL8, DM masking, tRCD/tRP/tRAS/tWR/tCCD/tREFI
   checking) and behavioural models for the Gowin primitives actually used
   (`OSER8`, `OSER8_MEM`, `IDES8_MEM`, `DQS`, `DLL`). That is the single
   largest chunk of work in this plan and it is on the critical path for the
   "sim passes" criterion. It is also the first thing that should prove the
   BL8 timing, which is the riskiest part of the design.
2. **There is no build script.** `ddr3.gprj` exists but there is no
   `build.tcl` to feed `gw gw_sh`. I will write one (it is source, and rules
   §2 requires it tracked).
3. **`tools/fpga-run.sh` has a wrong default path for this work.** It defaults
   `HOST_REPO_DIR` to `/media/openclaw/projects/primer-ddr3-controller`, which
   **exists and is a different clone** of this repo (on `main` at
   `34bf238 "remove the dispatch brief from source control"`, plus an empty
   `evidence/`). The dispatch says to work in
   `~/.hermes/profiles/dave/workspace/primer-ddr3-controller`. Invoking
   `tools/fpga-run.sh` as documented would therefore **build and flash a tree I
   am not measuring from**. I will pass `HOST_REPO_DIR` explicitly on every
   call and propose changing the default to the dispatch path. Flagging it
   because it is exactly the kind of thing that produces a plausible, wrong
   measurement.
4. The **brief's `24 pclk` write figure is a unit error** (section 1). Any plan
   built on it is wrong by 4x; this note does not use it.

## 8. Implementation order (for approval)

1. Behavioural DDR3 + Gowin primitive models, and a throughput testbench that
   counts `accept` pulses and asserts commands/cycle. **Proves the measurement
   path before any RTL change.**
2. On-chip counters + UART markers + `tools/decode_uart.py`, flashed and run on
   the **unmodified** controller. This is the 1.0132 resolution and it gives a
   clean re-measured baseline (expected ~33.2 MB/s write / 159.3 MB/s read
   from the RTL's own counters, replacing the inferred numbers).
3. `build.tcl` + first full vendor build, warnings read (rules §5), timing
   closed.
4. BL8 write datapath + 8-word combiner; sim; measure.
5. Command queue + response FIFO; sim; measure.
6. Row-open tracker + precharge-all on refresh; sim; measure.
7. Bench: both bulk sizes, both directions, wire capture, decoder, evidence
   committed.

Each of 4/5/6 is independently measurable, so if one of them does not reach
the bar the others still ship and the note stays honest about what moved.
