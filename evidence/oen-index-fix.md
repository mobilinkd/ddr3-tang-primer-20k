# DQS output-enable index fix (`wen[{cnt[2:1],1'b1}]` -> `wen[cnt[2:1]]`)

## What was wrong

`simulation/gowin_prim_models.v`, module `OSER8_MEM`, the `Q1` assign.
It read:

```verilog
assign Q1 = wen[{cnt[2:1], 1'b1}];
```

`wen` is 4 bits (`wire [3:0] wen = {TX3, TX2, TX1, TX0}`). The
concatenation `{cnt[2:1], 1'b1}` is **three** bits, so the index takes the
values 1, 3, 5, 7. Indices 5 and 7 are out of range for a 4-bit vector,
so Verilog returns `x` for them. `cnt` is a 3-bit counter advancing on
**both** FCLK edges, so `cnt[2:1]` is 2 for `cnt`=4,5 and 3 for
`cnt`=6,7. **Q1 was therefore `x` for four of every eight half-cycles.**

Reproduced directly by `simulation/probe_oen.v` against the pre-fix model:

```
OEN-PROBE FAIL q1 is x at t=23884 (cnt=4)
OEN-PROBE FAIL q1 is x at t=25141 (cnt=5)
OEN-PROBE FAIL q1 is x at t=26398 (cnt=6)
OEN-PROBE FAIL q1 is x at t=27655 (cnt=7)
```

## Why it mattered

`Q1` is not cosmetic. It is the pin enable:

```verilog
assign DDR3_DQS[i2] = dqs_buf_oen[i2] ? 1'bz : dqs_buf[i2];   // ddr3_controller.v:1202
```

so an `x` enable puts `x` on the `DDR3_DQS` pin. The `DQS` primitive
derives `RBURST = ~DQSIN | (rburst_hold != 0)` from that pin, so `x`
propagated into `rburst`. In the controller,
`if (rburst[0]) rburst_seen[0] <= 1'b1;` (ddr3_controller.v:1067) never
fires on an `x` condition, so `rburst_seen` could never reach `2'b11`
and the `READ_CALIB` loop at ddr3_controller.v:609 could never exit.

The old comment justified the index by claiming the TX lanes are
"paired by the design (TX0=TX1, TX2=TX3)" and that "both halves of a TX
pair are equal by construction". **That is false for the patterns the
controller actually drives**: `dqs_oen <= 4'b1110` (ddr3_controller.v:449)
has TX0=0, TX1=1. No pair-member selection can be justified here.

## The fix

```verilog
assign Q1 = wen[cnt[2:1]];
```

Derived from the RTL's own convention, not chosen to silence a symptom.
`cnt` advances on both FCLK edges, so one pclk word is eight half-cycles;
half-cycle `cnt` carries word bit `cnt`, and quadrant `q == cnt[2:1]`
covers half-cycles 2q and 2q+1, i.e. word bits `[2q+1:2q]`. The
controller documents exactly that on the port it drives
(`out_enable_n for dqs_out[1:0], [3:2], [5:4], [7:6]`,
ddr3_controller.v:165), so quadrant `q` is enabled by TX lane `q`. The
index is then 0..3 — always in range, never `x`, and on the right lane.

## Evidence: read calibration now converges

Same bench, same build flags, only the model line changed
(`-DSIM -DIVERILOG -DDDR3_800 -g 2012`).

| | pre-fix | post-fix |
|---|---|---|
| `rclkpos=` sweeps (tb_row) | 50 | **3** |
| `All initialization DONE` (tb_row) | 0 | **1** |

Pre-fix log: `evidence/oen-baseline-row.log`
Post-fix log: `evidence/oen-fixed-row.log`

Post-fix, calibration lands on the first swept position and completes:

```
rclkpos=1, rclksel=6
rclkpos=1, rclksel=6
All initialization DONE...
TB-ROW-READ-PHASE t=2056644 rclkpos=1 wlevel=1 rcalib=1
```

`rcalib=1` is the DUT's own `read_calib_done`, reported by the bench from
the DUT port, not inferred. The two sweeps are the retry at
`RCALIB_COUNT=8` consecutive hits before commit.

Unit probe, post-fix: `OEN-PROBE PASS (no x, no quadrant mismatch)`. The
probe drives one-hot-per-quadrant stimulus (`QPAT=4'b1001`), so it fails
on a merely in-range but wrong-quadrant index, not only on out-of-range.

Clock self-check unaffected: `CLOCK-PROBE PASS` (pclk 99.5450 MHz,
fclk/pclk 4.002160).

## What this does NOT prove

**Read calibration is now exercised in sim, not validated.** The `DQS`
model in this file still asserts `RBURST` whenever the strobe is active
and still drives `DQSR90 = FCLK` unconditionally, ignoring `READ` and
`HOLD`. So the x no longer masks the loop, but the loop's *result* is
decided by a model shortcut. The x was real and is fixed; the converged
`rclkpos`/`rclksel` are not evidence that the real primitive calibrates.

**The 503.18 MB/s figure is untouched and still unvalidated on the read
data path.** Post-fix `tb_rb` reports the identical
`pclk_per_command=3.1659`, `cmds=8192`, `responses returned=8192 of
8192`. This change is not what produced that number, and it does not
raise the bar on it.

**`tb_rb` still sweeps 2744 times with no `DONE`, pre-fix and post-fix
alike — byte-identical logs.** That is not a regression from this fix; it
is a property of that bench. `tb_rb.v:117` reads

```verilog
assign DDR3_DQS = mem_dqs_o ? 2'b11 : 2'b00;
```

which **overrides the bidirectional `DDR3_DQS` net with the memory
model's output and discards the controller's DQS drive entirely.** The
`rburst` the calibration loop watches therefore never carries the
controller's contribution, so this bench cannot observe the fix at all.
`tb_rb` remains a cadence bench (queue, issue rate, response
bookkeeping) and must not be read as a calibration or data-path result.
`tb_row` is the bench that resolves the DQS pin honestly
(`tb_row.v:130`, `assign dqs = {2{m_dqs_o}}` — also model-driven, but it
drives the controller's DQS through the real `inout` net into the
primitive, which is where `rburst` is formed).

Per `tb_rb.v:164-175`, `tb_rb` already declines to wait on
`read_calib_done` precisely because the model cannot satisfy it. This
change makes that comment accurate rather than aspirational.

## Files

- `simulation/gowin_prim_models.v` — the fix, with the derivation, the
  measured before/after, and a correction of the false pairing claim.
- `simulation/probe_oen.v` — new unit probe; asserts the index directly.
  Registered in the `Makefile` as `oen.check`.
- `simulation/Makefile` — `oen.check` target.
- `evidence/oen-baseline-row.log`, `evidence/oen-fixed-row.log`,
  `evidence/oen-baseline-rb.log`, `evidence/oen-fixed-rb.log` — the runs
  quoted above.
