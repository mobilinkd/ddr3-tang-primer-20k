# Working rules for this repository

Non-negotiable. Each one cost hours on this bench. Read the whole file before
your first commit.

## 1. The bar

**400 MB/s sustained is the requirement.** It is in `README.md` with the
measurement table. Any change that regresses a measured rate is a regression,
whatever else it improves.

## 2. All source must be committed

Every source file (RTL, testbenches, build Tcl, constraint and timing files,
host scripts, decoder scripts) is tracked in git. Nothing that is source may
live only in `build/` or `scratch/`.

- Build output (`.fs`, netlists, logs, `.pnr.json`) is regenerable and stays out
  of git.
- **Never `git add -f` a generated artifact.** If a conclusion depends on a run
  log, the log belongs under `evidence/`, tracked, and the doc that depends on
  it names it. A log nothing references does not belong in git at all.
- Never commit a bitstream.

## 3. The measurement is the primary artifact

A throughput claim without its capture is not a result. Commit:

- the raw analyzer capture and the decoder output,
- the exact decode invocation and the tool version,
- the command count the rate divides by.

**A rate is a command count times a bytes-per-command figure.** State both.
Both of the current numbers carry an unresolved 1.0132 scale factor on the
command count -- resolve that before treating either as settled.

## 4. Never loosen a test to make it pass

If a test fails, the design or the test is wrong. Do not widen a tolerance, do
not delete an assertion, do not skip a case to get green. Fix the thing.

## 5. A green build is not evidence the design is right

Never ignore a synthesis warning. An invalid config usually does not fail the
build: the tool substitutes a legal value, the build "passes", and the chip runs
at an unintended rate. Read the warnings on every build. This is especially true
for PLL configuration.

## 6. Simulate before claiming a tool bug

When a tool appears to misbehave, reproduce the behaviour in simulation against
a known input first. Twice on this project "the tool is broken" was a design
bug or a harness bug. Bring a reproduction, not a theory.

## 7. Constant stimulus hides the datapath

A testbench that ties an input to a constant lets the synthesizer const-fold the
logic under test, so the thing being measured stops existing in the netlist. If
you are measuring a datapath, drive it with changing data. A read-throughput
measurement on this controller was 8x wrong for exactly this reason: the stub
tied all eight `IDES8_MEM` outputs to zero, so the 128-bit read bus was a
constant.

## 8. The bench is exclusive

One process at a time. Two concurrent flashes, or a flash racing a UART reader,
wedge the FT2232 and cost hours. **Always** go through `tools/bench.sh`:

```
tools/bench.sh <kind> [--wait SECS] [--hold SECS] -- <command...>
tools/bench.sh session --hold 600 -- bash -lc '...'   # for flash+capture
```

A lock only serializes callers that take it. A raw `cat /dev/tang-uart` or a
direct `podman run` bypasses it entirely and can still wedge the bench. Use
`session` for any test needing more than one bench op: between a flash and its
capture the lock would otherwise be unowned, and the UART's RX buffer keeps
stale bytes from the previous design. Always `drain` before `capture`.

- JTAG is `/dev/tang-jtag` (ttyUSB1), UART is `/dev/tang-uart` (ttyUSB2).
- **DIP switch 1 DOWN or JTAG hangs.**
- Give every shell command an explicit timeout. No interactive probe without a
  connect timeout and a read timeout.

## 9. Builds go through the container

The vendor toolchain lives in the podman image `localhost/fpga-tools:latest`.
Never call a vendor binary bare -- use `tools/fpga-run.sh`, which mounts this
repo at its own absolute path and refuses an unwrapped Gowin CLI:

```
tools/fpga-run.sh bash -lc 'gw gw_sh build.tcl'
```

Run long vendor builds in the BACKGROUND. A previous attempt died on a 600 s
foreground timeout and produced a false blocker.

## 10. Report honestly and early

Two failures on one step: log `BLOCKED: <step> <reason>`, commit WIP, and report
back rather than thrashing. A clean partial report beats a wedged agent, and a
wrong number reported confidently is worse than no number. If you did not
measure it, say you did not measure it.

## 11. Commit as you go

Commit WIP markers if rate-limited or long-running rather than losing work.
Commit partial results with clear WIP markers in the report. The final message
can be swallowed by a provider error; a git commit cannot.

## 12. LEDs are mirrored

LED indices in the `.cst` are MIRRORED versus the silkscreen. If you report an
LED, name both forms.
