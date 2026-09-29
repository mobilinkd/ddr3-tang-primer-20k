# MEAS4 printed `xxxxxxxx`: three defects, not a missing reset

Status: RTL + bench fixes committed (`3555fd9`). End-to-end confirming run
in flight at the time of writing; the numbers below marked UNVERIFIED are
still unverified. Nothing here is a rate. Nothing is pushed.

## The claim being replaced

"The `s1_*`/`s4_*` snapshot registers have no reset, so MEAS4 is x." That was
wrong on two counts, and it sent the search after a reset that was never the
problem. `m_*` and the snapshot registers are not the issue: `m_pclk` counts
correctly, and `meas_snap`'s nonblocking assignment into `sN_*` lands
correctly (verified by a standalone probe -- see "What the probe disproved").

`MEAS4 xxxxxxxx` had three independent causes. Any one of them alone is
enough to produce it.

## Defect 1: the print machine has no MEAS4 at all

`src/ddr3_top.v:739` (pre-fix) runs the case over `8'd0`..`8'd31` for
MEAS0..MEAS3, then jumps to `8'd35` for ENDMEAS. Labels 32, 33 and 34 have
no arm. `MEAS_LAST` was `8'd35`.

A MEAS line is EIGHT labels -- header, pclk, sp, wr, sp, rd, sp, rf -- so
line *n* occupies `[8n, 8n+7]`. MEAS0..MEAS3 therefore own 0..31, and a
fifth line needs 32..39. The terminator at 35 sits **inside the range MEAS4
requires**. The DUT could not print MEAS4 whatever `s4_*` held.

This is why the missing reset looked like the cause: the register could have
been perfectly correct and the line still would not appear. The gap is
invisible in a transcript, because the run still printed ENDMEAS and still
looked well formed.

Fix: `8'd32`..`8'd39` added for the `s4_*` fields, terminator moved to
`8'd40`, `MEAS_LAST = 8'd40 = 8 * MEAS_LINES`. An elaboration `$error` now
rejects `MEAS_LAST != 8*MEAS_LINES`, so the arithmetic cannot silently rot
again.

## Defect 2: the abort path never snapshotted

`ddr3_top.v:567-570` (pre-fix), the data-mismatch arm in READ_BURST:

```
error_bit <= 1'b1;
end_state <= state;
state <= FINISH;          // <-- no meas_snap(2'd4)
```

On a mismatch the phase left for FINISH without recording its end state, so
`s4_*` kept its uninitialised value. The x meant *"the phase aborted and never
recorded its end state"*, not *"the counter was never valid"* -- and the
transcript could not tell the two apart, because the tb printed the
registers rather than the DUT printing them (defect 3).

Fix: the abort arm calls `meas_snap(2'd4)` before leaving. The error is still
reported by `error_bit`; the counters are no longer lost on top of it.

## Defect 3: the bench was forging the output

This is the one that made the first two invisible.

`simulation/tb_top.v:324-348` (pre-fix) `$write`d all five MEAS lines
itself, prefixed `UART|`, and then called `$finish`:

```
$write("UART|MEAS0 %08x %08x %08x %08x\n", dut.s0_pclk, ...);
...
$write("UART|MEAS4 %08x %08x %08x %08x\n", dut.s4_pclk, ...);
```

So **every MEAS line in every log to date was the testbench reading the
registers and printing them.** The DUT's own print machine had never run in
any transcript. That is why `PRINT-STATE` always ended at `print_meas=0`,
and why a run whose MEAS4 read `xxxxxxxx` still reported `TB-COMPLETE`: the
completeness check was the tb asserting its own flag, `saw_meas3 = 1'b1`, on
the line it had just written itself.

It also means the earlier claim that the read capture is unexercised has a
*third* reason, independent of the two already on record (the bench drives
`DDR3_DQS` itself, and `IDES8_MEM` has no READ/HOLD input): the DUT's own
print path was never the thing under test.

Fix: the dump block no longer writes the dump. The DUT's UART is the only
source. The receiver decodes the real bit stream, sets `saw_end` on the DUT's
ENDMEAS, and ends the run there. The receiver now tags **MEAS4** (it tagged
MEAS3), so a run that prints four lines and stops is caught rather than
reported complete.

### The wait was off by four orders of magnitude

The old block waited `#(100 * 10044)` after FINISH. The timescale is
`1ps/1ps`, so that is **1.0044 us**, not 1 ms. Against a dump that needs
223 bytes x 10 bits x 8681 ns = **19.4 ms** of serial time, and which cannot
even start until the preceding `print_stat` sequence (~107 bytes, ~9.3 ms)
has drained to zero.

So the tb `$finish`ed the run about 1 us after the DUT entered FINISH, while
the DUT was still on its first MEAS line. That is the mechanical reason the
DUT's own output never appeared in any log, and it is why the print-machine
gap in defect 1 was never caught by looking at the output.

Fix: the wait is 40 ms, sized from the baud rate rather than guessed, and it
is an upper bound only -- a good run ends on the real ENDMEAS, not on the
timeout.

## What the probe disproved

Chasing the missing reset, I wrote `scratch/meas_snap_probe.v` to test
whether `meas_snap`'s nonblocking assignment into an unreset register lands
when the source counter is in a separate `always` block. The first run
printed `s0=0, m_pclk=1` and looked like a lost write.

That reading was wrong, and the probe disproved itself: `s0_pclk` was
sampling `m_pclk`'s **pre-edge** value, which is what a nonblocking
assignment from a task is supposed to do. A second version that lets 200 more
cycles pass distinguishes the two cases that actually matter -- a snapshot
freezes at the call, a live copy keeps counting -- and shows the snapshot
behaving correctly.

`meas_snap` is sound. The reset theory is dead, and the fix is not there.

## Still open (NOT addressed here)

1. ~~**MEAS0 is non-monotonic.**~~ **RESOLVED -- it was the same bug as
   MEAS4, and it was a 2-bit port, not a re-entered phase.** See
   `meas4_slot_width_root_cause.md`. The old item-1 text below is kept only
   to mark what was believed and why it was wrong; do not read it as current.

   > (superseded) `s0_pclk` = 3059898 = 30.73 ms, the pclk count at the
   > *end* of the run, but `meas_snap(2'd0)` is called once, in READ_DONE at
   > ~12.2 ms ... Undiagnosed.

   The trace settles it: `meas_snap` is called **twice with slot 0** --
   `m_pclk=1200101` at READ_DONE, then `m_pclk=3059898` at the READ_BURST
   completion. The READ_BURST call site passed `2'd4`, which truncated to
   `2'b00`, so the end-of-run value overwrote the baseline. The "late write"
   was real; the slot it was aimed at was the bug. One fix, both symptoms.

2. **The read capture remains unexercised at the design point.** Two
   independent reasons stand, both unchanged here: the bench drives
   `DDR3_DQS` (now a faithful `2'bzz` join, `5e81104`), and `IDES8_MEM`
   (`gowin_prim_models.v:398-404`) has no READ/HOLD input and samples `D`
   unconditionally, so `dqs_read` cannot affect what any bench captures.
   Fixing the print machine does not change this.

3. No rate is claimed. Per instruction, the 503.18 figure was not touched and
   nothing was re-run for a rate.
