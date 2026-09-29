#!/usr/bin/env python3
"""Decode a ddr3_top UART capture into measured throughput.

WHAT THIS IS FOR
----------------
A rate is a COMMAND COUNT times a BYTES-PER-COMMAND figure. Both halves
have to be stated, and the command count has to be measured rather than
inferred. The DUT counts, on chip, one pulse of `accept` per command it
actually took in, and one tick of pclk; it snapshots both at each bulk
phase boundary and prints the snapshots over the UART as

    MEAS<n> <pclk> <cmd_wr> <cmd_rd> <refresh>        (hex, 8 digits each)

so every phase is a difference of two on-chip numbers. Nothing is
printed during a measured phase, so the 115200-baud link cannot
perturb what it measures. The UART carries numbers; the pclk counter
carries time.

This script turns those lines into a rate. It runs unchanged against a
simulation transcript and against a raw capture from /dev/tang-uart,
because the bytes on the wire are identical.

WHAT IT REFUSES TO DO
---------------------
It will not report a rate from an incomplete run. A capture missing a
snapshot, or carrying non-monotonic counters, is a harness or DUT
failure and is reported as such rather than as a number.

Usage
-----
    decode_uart.py CAPTURE [-o OUT.json] [--pclk-mhz F] [--json]

    CAPTURE   a text file. Lines may be raw UART text, or simulator
              transcript lines of the form "UART|<text>". Anything
              outside a MEAS line is ignored, so the same file can be
              the whole sim log.
"""

import argparse
import json
import re
import sys

# pclk = fclk / 4 = 398.25 MHz / 4. This is the real hardware value.
# (src/ddr3_top.v uses FREQ=99_800_000 for its tick counters, which is a
# rounding; the rate arithmetic must use the actual clock.)
PCLK_HZ_DEFAULT = 99_562_500.0

# Which phase covers which direction, and how many useful bytes each
# command delivers. These are the design-note findings, and they are
# stated here rather than buried in the arithmetic:
#
#   WIPE         : bulk write, BC4 + DM, 2 useful bytes per command
#   WRITE_BLOCK  : bulk write, BC4 + DM, 2 useful bytes per command
#   VERIFY_BLOCK : bulk read,  BL8 legacy engine,  16 bytes per command
#   READ_BURST   : bulk read,  BL8 pipelined engine (the 400 MB/s phase),
#                  16 bytes per command -- READ-ONLY path, no writes
#
# bytes_per_command is overridable on the command line because it is the
# one number here that is an inference from the RTL rather than a
# measurement, and it is the number most likely to change when the
# datapath does.
PHASES = [
    # (snapshot index, name, direction, bytes per command)
    (1, "WIPE",        "write", 2),
    (2, "WRITE_BLOCK", "write", 2),
    (3, "VERIFY_BLOCK", "read", 16),
    (4, "READ_BURST",  "read", 16),
]

# The requirement, in MB/s, for the READ_BURST phase. The read datapath is
# the only one the 400 MB/s requirement applies to: the brief is explicitly
# read-only, and the write path is unchanged from the legacy controller.
READ_BAR_MB_S = 400.0

# The phase that must clear READ_BAR_MB_S.
READ_PHASE = "READ_BURST"

MEAS_RE = re.compile(
    r"MEAS(?P<idx>\d)\s+(?P<pclk>[0-9a-fA-F]+)\s+"
    r"(?P<wr>[0-9a-fA-F]+)\s+(?P<rd>[0-9a-fA-F]+)\s+(?P<rf>[0-9a-fA-F]+)"
)

BASELINE = 0


def extract(text):
    """Return {index: (pclk, wr, rd, refresh)} from MEAS lines."""
    found = {}
    for m in MEAS_RE.finditer(text):
        idx = int(m.group("idx"))
        found[idx] = (
            int(m.group("pclk"), 16),
            int(m.group("wr"), 16),
            int(m.group("rd"), 16),
            int(m.group("rf"), 16),
        )
    return found


def load(path):
    with open(path, "r", errors="replace") as fh:
        return fh.read()


def compute(snaps, pclk_hz, bpc_override=None):
    """Build the per-phase result list, or an error string."""
    missing = [i for i in range(5) if i not in snaps]
    if missing:
        return None, ("incomplete run: missing snapshot(s) %s of 0..4; "
                      "a harness or DUT failure, not a data point"
                      % ", ".join(str(m) for m in missing))

    base = snaps[BASELINE]
    results = []
    problems = []

    # Each phase is measured against the snapshot that PRECEDED it: the
    # first against the baseline, the rest against the previous phase's
    # end. Keeping the previous index explicit avoids the off-by-one a
    # positional lookup would introduce.
    prev_idx = BASELINE
    for idx, name, direction, bpc in PHASES:
        prev = snaps[prev_idx]
        cur = snaps[idx]
        pclk0, wr0, rd0, rf0 = prev
        pclk1, wr1, rd1, rf1 = cur

        d_pclk = pclk1 - pclk0
        d_wr = wr1 - wr0
        d_rd = rd1 - rd0
        d_rf = rf1 - rf0

        if d_pclk <= 0:
            problems.append("%s: pclk delta is %d, not positive" % (name, d_pclk))
            prev_idx = idx
            continue
        if d_wr < 0 or d_rd < 0 or d_rf < 0:
            problems.append("%s: a counter went backwards (wr=%d rd=%d rf=%d)"
                            % (name, d_wr, d_rd, d_rf))
            prev_idx = idx
            continue

        cmds = d_wr if direction == "write" else d_rd
        bpc = bpc_override if bpc_override is not None else bpc
        if cmds == 0:
            problems.append("%s: no %s commands were accepted" % (name, direction))
            prev_idx = idx
            continue

        rate = cmds * bpc / d_pclk * pclk_hz
        results.append({
            "phase": name,
            "direction": direction,
            "commands": cmds,
            "bytes_per_command": bpc,
            "bytes": cmds * bpc,
            "delta_pclk": d_pclk,
            "pclk_per_command": d_pclk / cmds,
            "delta_cmd_wr": d_wr,
            "delta_cmd_rd": d_rd,
            "delta_refresh_issued": d_rf,
            "mb_per_s": rate / 1e6,
        })
        prev_idx = idx

    if problems:
        return None, "; ".join(problems)
    return results, None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", help="UART capture or simulation transcript")
    ap.add_argument("-o", "--out", help="write the full result as JSON here")
    ap.add_argument("--pclk-mhz", type=float, default=PCLK_HZ_DEFAULT / 1e6,
                    help="pclk in MHz (default %(default)s)")
    ap.add_argument("--bpc", type=int, default=None,
                    help="override bytes-per-command for every phase")
    ap.add_argument("--json", action="store_true", help="print JSON to stdout")
    args = ap.parse_args()

    pclk_hz = args.pclk_mhz * 1e6
    snaps = extract(load(args.capture))

    if not snaps:
        print("decode_uart: no MEAS lines found in %s" % args.capture,
              file=sys.stderr)
        print("  This is a capture or harness failure, not a measurement.",
              file=sys.stderr)
        return 2

    results, err = compute(snaps, pclk_hz, args.bpc)

    payload = {
        "capture": args.capture,
        "pclk_hz": pclk_hz,
        "snapshots": {"MEAS%d" % k: {"pclk": v[0], "cmd_wr": v[1],
                                     "cmd_rd": v[2], "refresh_issued": v[3]}
                      for k, v in sorted(snaps.items())},
        "ok": err is None,
        "phases": results if results is not None else [],
    }
    if err is not None:
        payload["error"] = err

    # The pass/fail verdict on the READ bar, computed from the same numbers
    # that are printed. Stated here so the JSON carries the verdict and not
    # just the raw figures -- a reader must not have to re-derive the
    # comparison to know whether the requirement was met.
    read = None
    if results is not None:
        for r in results:
            if r["phase"] == READ_PHASE:
                read = r
    if read is not None:
        payload["read_bar_mb_s"] = READ_BAR_MB_S
        payload["read_result"] = {
            "phase": read["phase"],
            "mb_per_s": read["mb_per_s"],
            "pclk_per_command": read["pclk_per_command"],
            "commands": read["commands"],
            "bytes_per_command": read["bytes_per_command"],
            "pass": read["mb_per_s"] >= READ_BAR_MB_S,
        }

    if args.out:
        with open(args.out, "w") as fh:
            json.dump(payload, fh, indent=2, sort_keys=True)
            fh.write("\n")

    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print("decode_uart: %s" % args.capture)
        print("  pclk = %.4f MHz  (stated, not derived)" % (pclk_hz / 1e6))
        if err is not None:
            print("  RESULT: NOT A MEASUREMENT -- %s" % err)
        else:
            for r in results:
                print("  %-13s %-5s  %9d cmd x %2d B = %10d B"
                      "  / %8d pclk  = %8.2f MB/s   (%.4f pclk/cmd)"
                      % (r["phase"], r["direction"], r["commands"],
                         r["bytes_per_command"], r["bytes"], r["delta_pclk"],
                         r["mb_per_s"], r["pclk_per_command"]))
            print("  A rate is a command count times a bytes-per-command figure;")
            print("  both are printed above. The 400 MB/s bar is AGENTS.md rule 1.")
            if read is not None:
                v = payload["read_result"]
                print("  READ BAR: %s  %.2f MB/s  (%.4f pclk/cmd, %d cmd x %d B)"
                      % ("PASS" if v["pass"] else "FAIL", v["mb_per_s"],
                         v["pclk_per_command"], v["commands"],
                         v["bytes_per_command"]))
                print("             bar is >= %.1f MB/s on the READ path only."
                      % READ_BAR_MB_S)
    return 0 if err is None else 1


if __name__ == "__main__":
    sys.exit(main())
