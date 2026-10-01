# MEAS4 is x because the snapshot slot port was 2 bits, not because a phase left without snapshotting

Status: root cause found by trace and fixed. Confirming run in flight at the
time of writing. **No rate is claimed here and none was re-run for one.** The
503.18 cadence figure is untouched.

## The bug, in one line

`task meas_snap; input [1:0] ph;` cannot hold the value 4. Every READ_BURST
call site passed `2'd4`, which is truncated by the language to `2'b00` -- so
both READ_BURST exits wrote **slot 0**, and **slot 4 had no reachable writer
in the entire design**.

`evidence/sim-exitprobe-run.log`, from the instrumented run:

```
TB-TRACE EXIT-A READ_BURST clean-completion pclk=3059898 wr=131075 rd=65539 issued=8192 recvd=8191 t=30733921854
TB-TRACE MEAS-SNAP slot=0 m_pclk=3059898 wr=131075 rd=65539 m_rf=1 t=30733921854
```

The clean-completion path ran. `meas_snap` executed. It executed with
**`slot=0`**.

## Two long-standing puzzles, one cause

The trace also explains the MEAS0 non-monotonicity, which had been carried as
a separate open item and attributed to a re-entered phase or a late write:

```
TB-TRACE MEAS-SNAP slot=0 m_pclk=1200101 wr=3        rd=3      m_rf=0   <- READ_DONE baseline
TB-TRACE MEAS-SNAP slot=1 m_pclk=1724398 wr=65539    rd=3      m_rf=1
TB-TRACE MEAS-SNAP slot=2 m_pclk=2248687 wr=131075   rd=3      m_rf=1
TB-TRACE MEAS-SNAP slot=3 m_pclk=3035120 wr=131075   rd=65539  m_rf=1
TB-TRACE MEAS-SNAP slot=0 m_pclk=3059898 wr=131075   rd=65539  m_rf=1   <- READ_BURST, aimed at 4
```

**Slot 0 has two writers.** The READ_DONE baseline wrote it at 1.20e6 pclk;
the READ_BURST completion -- silently aimed at slot 0 by the truncation --
overwrote it at 3.06e6 pclk, last write wins. That is exactly the reported
symptom: MEAS0 holding the END-of-run pclk, and MEAS0 > MEAS1..MEAS3 while
slots 1, 2, 3 are perfectly monotonic among themselves.

So the "late write" observation was correct and the "slot re-visit" theory was
wrong. Nothing re-entered anything. One bug, two symptoms, and fixing the
width fixes the monotonicity too -- so the separate MEAS0 investigation
recorded in `meas4_xxxxxxxx_root_cause.md` is closed by this commit, not
merely unexplained.

## The warning was in the build output the entire time

Icarus emitted, three times per compile:

```
../src/ddr3_top.v:159: warning: Numeric constant truncated to 2 bits.
../src/ddr3_top.v:555: warning: Numeric constant truncated to 2 bits.
../src/ddr3_top.v:597: warning: Numeric constant truncated to 2 bits.
```

Line 159 is the `2'd4:` **case label inside the task**; 555 and 597 are the
two `meas_snap(2'd4)` call sites. The case arm for slot 4 was unreachable by
construction -- a 2-bit value can never select it. Those three warnings are
the whole diagnosis, and they were in every compile log since the task was
written. A truncation warning on a *literal that selects a measurement slot*
is not a cosmetic warning; it is the bug, stated exactly, three times.

After the fix the same compile emits **zero** truncation warnings.

## Correction to the review: there is no third exit from READ_BURST

The review that prompted this work identified three FINISH exits in
`READ_BURST` and asked that the unsnapshotted one be deleted as dead code.
The first two thirds of that are right and the third is not, and the
distinction matters, so it is recorded precisely:

`grep -n 'state <= FINISH' src/ddr3_top.v` returns exactly three lines, and
`state` has exactly one driver (the FSM `always @(posedge clk)` block). By
enclosing phase:

| line (pre-fix) | phase | snapshots? |
|---|---|---|
| 539 | `READ_BURST` clean completion | yes, `meas_snap(2'd4)` |
| 577 | `READ_BURST` 128-bit mismatch | yes, `meas_snap(2'd4)` |
| 612 | **`VERIFY_BLOCK`** legacy byte mismatch | **no** |

**Line 612 is in `VERIFY_BLOCK`, not `READ_BURST`.** It is a different phase,
and it runs *before* `READ_BURST` in the sequence
`WIPE -> WRITE_BLOCK -> VERIFY_BLOCK -> READ_BURST -> FINISH`. So:

- There is **no third exit from `READ_BURST`**. Both of its exits snapshot.
  (They snapshot into the wrong slot, which is the bug above, but they do
  snapshot.)
- The arm at 612 is **not dead code**, and the reasoning that it was is
  wrong. `data_ready` is driven only from the legacy `{READ, cycle 7}` arm
  (`ddr3_controller.v:419`) -- correct -- but `fast_mode` is
  `(state == READ_BURST)`, and `VERIFY_BLOCK` is not `READ_BURST`, so
  `fast_mode` is 0 throughout `VERIFY_BLOCK` and the legacy FSM owns the
  datapath. `data_ready` is precisely the signal this phase is built around:
  it asserts `rd` and waits for `data_ready`. The arm is the live legacy
  verify check.
- Deleting it would have removed the only legacy read-data check in the
  design, on the strength of a phase attribution that was off by one case
  label.

It has been left in place and **commented with this reasoning** at the call
site, so the next reader does not have to re-derive it. It is genuinely the
one FINISH exit that takes no snapshot, and that is now argued to be correct
rather than an oversight: it is a *mid-phase* abort, not the end of a measured
phase. MEAS3 is taken at the end of `VERIFY_BLOCK` on the
`addr == START_ADDR+TOTAL_SIZE` arm, so there are no end-of-phase counters to
record here. Snapshotting into slot 3 from a mid-phase abort would write a
half-phase value into a slot the decoder differences against WIPE and
WRITE_BLOCK -- a worse lie than no number. The failure is reported through
`error_bit`, which the FINISH status block prints.

## Method note: how the exit was actually identified

Per the review, no conclusion was drawn from the MEAS text. A single
instrumented run (`-DTB_TRACE`, guards in the RTL so the synthesized design is
unchanged) put one `$display` in each of the three FINISH arms and one inside
`meas_snap` itself printing the resolved slot and the `m_*` source values.
That distinguishes the two candidate causes in one shot:

- `m_pclk` reads **x** at the call -> the source is broken.
- `m_pclk` reads a **number** and the printed line is still x -> the print
  path is broken.

It read a number, into the wrong slot. Item 2 of the review ("if they are x
there, the source is the problem") is answered: they were not x, and the
source was never the problem.

Instrumentation is `ifdef TB_TRACE`-guarded and was left in the tree, since
the next person to change a snapshot call site needs it. It is not in any
synthesis path.

## Second defect found while confirming: the print path truncated the stream

Fixing the slot width made the DUT-side dump complete, and the run then
exposed defects further down the same path. Recorded here because this is the
second time this project has mistaken a transport problem for a measurement
problem, and because **my first two attributions of it were both wrong** and
were caught by measurement rather than by reasoning.

`evidence/sim-meas4slot3-run.log` (slot width fixed, nothing else changed)
ended with `meas_go=0, print_meas=0, seq_head=54, seq_tail=54` and **zero**
`UART|` lines. Read naively that says "the DUT still did not print MEAS4". It
did: the trace in the same log shows all five slots written and monotonic. The
bytes were produced and then lost downstream.

### What I got wrong first, and how the probe caught it

I first blamed the `print.v` send block for "dropping the byte after a
completed one", then blamed the UART bit period. **Both were wrong**, and the
wire probe disproved them in one run: after applying both supposed fixes the
probe read `negedges_on_wire=26 tx_start_bytes=11` -- *byte-identical* to the
pre-fix run. A transport bit-period change cannot leave a byte counter
bit-identical, so neither fix had touched the active path. That is recorded
here rather than quietly deleted, because the wrong story was plausible and
would have shipped.

### Real defect 1: `seq_head` advanced per clock, not per byte (my own fix)

The original send block was:

```verilog
always@(posedge print_clk)begin
    uart_en<=1'b0;
    if(uart_en && uart_bz)   seq_head<=seq_head+8'd1;
    if(seq_head!=seq_tail && !uart_bz) uart_en<=1'b1;
end
```

`uart_bz` is the transmitter busy flag. This is lossy only in the sense of
costing an idle gap per byte. My "fix" -- hold `uart_en` high and re-assert it
inside the completion branch so bytes go back-to-back -- was **worse**: with
`uart_en` high for the whole ~8640-pclk transmission, the completion test
`uart_en && uart_bz` is true on *every* cycle of it, so `seq_head` advanced
once per **clock** instead of once per **byte**. Measured: `seq_head` reached
54 while the transmitter had started only 11 bytes.

`seq_head == seq_tail` at end of run is precisely what hid this -- a stream
truncated to 54 bytes looks perfectly drained.

Correct form: advance the head once per byte on the **falling edge** of
`uart_bz` (`tx_busy` is `(state != STATE_IDLE)`, so it falls when the
transmitter returns to IDLE), re-arming `uart_en` on that same edge. Verified:
the probe moved from `tx_start_bytes=11` to `54`, with `seq_head` tracking it
exactly -- the two numbers must stay equal, and `tb_top.v` prints both.

### Real defect 2: `int_print` drops requests, silently

With the FIFO correct, only **54 bytes were ever enqueued** for a stream that
needs 118 (FINISH status block) + 218 (dump) = 336. `int_print` accepts a
request only while `print_state == PRINT_IDLE_STATE`; the print FSM leaves
IDLE on the next cycle, so a label issued on back-to-back idle cycles is
thrown away. The macro has no return value and no error path, so a dropped
label leaves no trace -- this is the same silent-truncation failure as the
original MEAS4 print gap, one layer down.

Fixed with `print_stat_q` / `print_meas_q` request-pending flags, released
**by the consumer** (on `print_state` leaving IDLE, which proves the
`spin_state` toggle was taken) rather than by a timer. A first attempt that
cleared the flag one cycle after setting it lost every label outright; both
the attempt and its failure mode are documented in the RTL so it is not
reintroduced.

Measured effect of this fix: enqueued bytes 54 -> 88, and `print_stat` reached
10 instead of 0. The stream is still short of 336 at the 40 ms timeout, so
per-label throughput remains open work -- but it is now *advancing
monotonically and observably* instead of silently discarding labels, which is
what makes the next run diagnosable.

### Real defect 3: the UART bit period was wrong (independent, not the cause)

`FREQ` was `99_800_000`, a nominal figure, against a real pclk of 99.5625 MHz.
That gives a bit period of 8688.0 ns instead of 8680.6 ns. Separately,
`TX_CLK_MAX = (clk_freq / uart_freq) - 1` both truncates and has an off-by-one
(the counter bound is inclusive), yielding 863 pclk = 8667.9 ns, 12.6 ns short
per bit.

This is a genuine defect and is fixed -- correct value is 864 pclk
(8678.0 ns, -2.6 ns, well inside the 50% sampling window) -- but it is **not**
what caused the zero bytes, and it is not claimed to be. An elaboration
`$error` now rejects any value outside half a baud clock; the guard was
verified to fire on 863 and on 865 and to pass only on 864. `FREQ` has one
other consumer, `REFRESH_COUNT`, which is 773 under both the old and new
value, so the refresh cadence does not move.



Fixing the slot width made the DUT-side dump complete, and the run then
exposed a **separate** defect further down the same path. Recorded here
because it is the second time this project has mistaken a transport problem
for a measurement problem, and the two look identical from the log.

`evidence/sim-meas4slot3-run.log` (slot width fixed, nothing else changed)
ended:

```
TB-TIMEOUT DUT print dump did not complete -- harness failure, not a data point
PRINT-STATE print_state=0 seq_head=54 seq_tail=54 meas_go=0 print_meas=0 print_stat=0
TB-INCOMPLETE saw_end=0 saw_meas4=0
```

and `grep -c 'UART|'` on that log returns **0**. Read naively, that says "the
DUT still did not print MEAS4". It did. The trace in the same log shows all
five slots written and monotonic, and the dump driver ran to completion --
`meas_go=0`, `print_meas=0` is the counter having finished its walk. **Zero
bytes reached the wire.**

## Evidence index

Every log below is referenced here; a log nothing references does not belong
in git (AGENTS.md 2). Read the `tx_start_bytes` column across the last six
rows -- that single number is the whole print-path story, and the four
identical 11s are what disproved two of my own diagnoses.

| log | `UART|` lines | wire probe (negedges / bytes started) | seq head/tail | print_stat | what it establishes |
|---|---|---|---|---|---|
| `sim-exitprobe-run.log` | 0 | (probe not yet added) | 54/54 | 0 | **the assigned bug**: clean-completion EXIT-A, and `meas_snap` executing with `slot=0` |
| `sim-meas4slot3-run.log` | 0 | (probe not yet added) | 54/54 | 0 | 3-bit slot fix, full 64 Ki-word region: slots 0..4 monotonic, slot 4 present |
| `sim-meas4slot3-uart-run.log` | 0 | (probe not yet added) | 54/54 | 0 | same, plus the print.v reload change; still zero bytes |
| `sim-uartprobe-small.log` | 0 | **26 / 11** | 54/54 | 0 | probe added: `seq_head` 54 but only **11** bytes started -> head advancing per clock |
| `sim-uartfreq-fix-small.log` | 0 | **26 / 11** | 54/54 | 0 | after the FREQ/bit-period "fix": **byte-identical**, so the bit period was not the cause |
| `sim-uartbit-run.log` | 0 | **26 / 11** | 54/54 | 0 | after the TX_CLK_MAX off-by-one fix: still **byte-identical** |
| `sim-uartfix-run.log` | 0 | **26 / 11** | 54/54 | 0 | after reverting `meas_req`: still **byte-identical** |
| `sim-uartfix2-run.log` | 0 | **131 / 54** | 54/54 | 0 | print.v falling-edge fix: bytes started now **tracks** `seq_head` (54) |
| `sim-uartfix3-run.log` | 0 | **227 / 88** | 88/88 | **10** | request-pending flags: enqueued 54 -> 88, `print_stat` 0 -> 10 |

All nine end `TB-INCOMPLETE`. None is a measurement; none is quoted as a
rate. The first three predate the probe and carry `seq_head=54 seq_tail=54`,
which is precisely the "looks drained" condition that hid defect 1.

The four identical `26 / 11` rows are the load-bearing evidence in this
report. A bit-period change cannot leave a byte counter unchanged, so those
four runs falsified the bit-period theory and the first print.v theory
together, in one comparison, and pointed at the per-clock head advance.

## The lesson worth keeping

`meas_go=0, print_meas=0` means *the counter finished walking*. It does not
mean *the data was transmitted*. The tb's receiver, which tags the real bit
stream and sets `saw_meas4` only on a decoded `MEAS4`, is the only thing in
the design that distinguishes those two, and it is the reason the bench
stopped forging the dump. Had it still been forging lines, both of these
defects would have been invisible again -- the forged line would have printed
`MEAS4 <register value>` and looked perfect.

## Status of the numbers

- MEAS0..MEAS4 are now written to five distinct slots by five distinct phase
  boundaries, monotonic in `m_pclk` by construction. The decoder's
  `missing snapshot(s) 4 of 0..4` and the negative-WIPE-delta refusal are
  both consequences of the single truncation above.
- **No rate is claimed from this run.** It was a targeted confirming run of a
  slot-write bug, not a cadence measurement, and it was not run to move the
  503.18 figure, which is untouched.
- Unchanged and still open: the read capture remains unexercised at the design
  point, for the two independent reasons already on record (the bench drives
  `DDR3_DQS`; `IDES8_MEM` has no READ/HOLD input). Fixing a snapshot port
  width does not change that.
