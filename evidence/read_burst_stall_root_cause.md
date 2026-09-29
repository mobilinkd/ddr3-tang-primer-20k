# READ_BURST stall: root cause, and what the 503.18 MB/s number is

Date: 2026-09-29. Branch `feat/rtl-400`. Fix commit `d41e26f`.
Probes: `simulation/probe_discard.v`, `simulation/probe_override.v`.
Logs: `evidence/probe-discard-run.log`, `evidence/probe-override-run.log`.

## 1. The stall: response counting, not the DQS path

The stall signature was `iss=8192, recvd=11`, `fsm=S_IDLE`, `q_count=0`,
`cmd_ready=1`, `dqs_read=0`, `dout128=z`, with `rburst_pulses` and `rq`
frozen across samples.

`dqs_read = 0` is **correct behaviour, not the fault**. In fast_mode
`dqs_read` pulses only from `rd_strb_tap` (`ddr3_controller.v:1081-1092`),
and `rd_strb` is shifted by `q_pop` (`:843`). With `q_pop = 0` the engine
correctly reports no read in flight. Nic's hypothesis 1 — that `dqs_read` is
never armed after calibration — is **disproved**: the tap is armed and
fires, 8194-8570 pulses per run.

The fault is in `ddr3_top.v`, and it is an accounting bug, not a datapath
bug. The controller's `rready` is tied to a constant `1'b1`
(`ddr3_top.v:216`), so the engine retires every response the moment it is
valid (`f_pop = rvalid & rready`). But `rb_recvd` was incremented only in
the `else if (rvalid)` arm of the READ_BURST `case`, and that arm is reached
only after the preceding `else if (rb_issued < RB_CMDS)` arm is false —
i.e. only once **all 8192 commands have been taken in**. Every response
arriving while commands were still being issued was retired by the engine
and never counted, and never data-checked either.

`recvd = 11` is therefore the pipeline **tail**, not a response count. With
`READ_LATENCY = 8` and `RESP_DEPTH = 8` (`ddr3_controller.v:683,:692`) a
tail of 8-16 is exactly what the arithmetic predicts.

`probe_discard.v` measures both counting rules on one run, at the wire,
with no reference to `rb_recvd`:

```
PROBE   commands offered/taken in = 8192
PROBE   q_pop    (issued to DRAM)  = 8192
PROBE   f_push   (captured)        = 8192
PROBE   f_pop    (retired)         = 8192
PROBE   rvalid WIRE  (all)         = 8192
PROBE   rvalid LATE  (ddr3_top's rule) = 11
PROBE   DISCARDED by the top       = 8181
PROBE   dqs_read pulses            = 8570
```

**All 8192 responses arrive, are captured, and are retired. The top
discarded 8181 of them.** The read data path was never dead.

Fix (`d41e26f`): count responses on every cycle, decoupled from the issue
pump, and complete on `rb_recvd + 1 == RB_CMDS`.

Post-fix, `evidence/sim-recvd-dbg.log` shows `recvd` tracking `iss`
through the whole phase (`iss=8165 recvd=8154`, delta stable at 10-11) and
the phase reaching FINISH. The stall is closed.

## 2. Still open: the rate is not measurable from tb_top yet

The post-fix run reaches FINISH, but `MEAS1`..`MEAS4` are all `xxxxxxxx`:

```
UART|MEAS0 0001e7c6 00000000 00000000 00000000
UART|MEAS1 xxxxxxxx xxxxxxxx xxxxxxxx 00xxxxxx
```

`MEAS0` is valid; the later slots are not. The `s1_*`..`s4_*` snapshot regs
(`ddr3_top.v:132-136`) have **no reset**, and `meas_snap` (`:139-150`) is
only called at each phase boundary. A slot whose phase never completes stays
X, and the decoder cannot produce a rate from X. So the 400 MB/s claim still
rests on the tb_rb cadence bench, not on the end-to-end top. **Not fixed
here** — this is the next piece of work.

## 3. The DQS override: cheap to remove, and it was never load-bearing

`DDR3_DQS` is an `inout` on the controller (`ddr3_controller.v:59`) which the
controller itself drives at `:1202`:

```verilog
assign DDR3_DQS[i2] = dqs_buf_oen[i2] ? 1'bz : dqs_buf[i2];
```

Both benches add a second, unconditional driver (`tb_rb.v:117`,
`tb_top.v:115`):

```verilog
assign DDR3_DQS = mem_dqs_o ? 2'b11 : 2'b00;
```

That is a **multiple-driver conflict**, not a clean override: while the
controller drives, the two resolve to X. The model already encodes
"released" as `dqs_o = 1` (`ddr3_x16_model.v:132`), so the faithful join is
one line:

```verilog
assign DDR3_DQS = mem_dqs_o ? 2'bzz : 2'b00;
```

`probe_override.v` is byte-identical to `probe_discard.v` except for that one
line, so the runs are directly comparable:

| run | DQS net driven by | `f_push` | `dqs_read` | `rburst` | `DDR3_DQS[0]` x-cycles |
|-----|-------------------|----------|------------|----------|--------------------------|
| `probe_discard` | model (override in place) | **8192** | 8570 | 0 | 0 |
| `probe_override` | controller, model only when released | **8192** | 8194 | 40 | 41 |

**Removing the override is cheap and loses nothing**: 8192/8192 responses
still captured. It also makes the controller's own DQS drive observable for
the first time — `rburst` goes from 0 to 40 pulses, which is the DQS
primitive actually seeing the controller's drive instead of only the
model's. The 41 x-cycles are the two drivers overlapping, which is itself
the finding: with the override the bench can never distinguish "the
controller drove DQS" from "the model drove DQS".

Recommendation: remove the override in both benches. It is one line each,
it is verified not to regress the capture, and it closes a real
model-fidelity gap. The x-cycles on overlap are a property of two drivers on
one net, not a new defect.

## 4. The remaining fidelity gap (unchanged, now quantified)

`gowin_prim_models.v` drives `DQSR90 = FCLK` unconditionally and the
`IDES8_MEM` model has no `READ`/`HOLD` input at all. The IDES8 is therefore
always clocked in simulation, so **the controller's `dqs_read` pulse cannot
affect what any bench captures** — the deserialiser samples on a clock it
gets regardless. This is a model-fidelity gap, not a design bug, and it is
the reason no bench can yet prove the read capture at the design point.
Settling it is bench work: the primitive needs to honour `READ` and `HOLD`.

## 5. What this means for the 503.18 MB/s number

See the characterisation agreed with Nic: publish it as a **cadence**
result with the boundary in the same sentence, or do not headline it. It
measures the queue, the issue cadence, the address decode, the row tracker
and the response bookkeeping. It does not exercise the controller's read
capture, for two independent reasons now on record: the DQS override (fixed
cheaply, §3) and the `IDES8_MEM` model ignoring `READ`/`HOLD` (§4).
