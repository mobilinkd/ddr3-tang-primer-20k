#!/usr/bin/env bash
# bench.sh -- EXCLUSIVE access funnel for the Tang Primer 20k bench.
#
# WHY THIS EXISTS
#   The FT2232 wedges ("unable to open ftdi device: -6 ftdi_usb_reset failed",
#   or the device dropping off the USB bus entirely) when two processes touch
#   the bench at once. Concurrent `openFPGALoader` flashes, a flash racing a
#   UART reader, or two agents' sessions both driving /dev/tang-* will do it.
#   Observed 2026-09-15: two Kevin sessions ran in parallel, produced
#   contradictory "V-OLD control" captures from different bitstreams, and
#   wedged the FT2232 repeatedly. A capture is only meaningful if NOTHING ELSE
#   is touching the bench while it runs.
#
# HOW IT WORKS
#   Every bench operation takes an EXCLUSIVE flock on a shared lock file
#   before touching the hardware, and holds it for the whole operation. The
#   lock file lives in the user runtime dir, NOT in a repo, so every clone and
#   every peer profile on this host serializes against the same lock.
#   flock is released automatically when the holder dies -- there are no stale
#   locks to clean up.
#
# USAGE
#   tools/bench.sh <kind> [--wait SECS] [--hold SECS] -- <command...>
#
#   kind: human label for the log, e.g. flash | capture | probe
#
#   # SINGLE OPERATION
#   tools/bench.sh flash -- \
#     podman run --rm --network=host --name fpga-jtag-probe ... \
#     openFPGALoader -b tangprimer20k design.fs
#
#   tools/bench.sh capture --hold 300 -- \
#     cat /dev/tang-uart
#
#   # WHOLE TEST -- one atomic acquisition (flash + drain + capture)
#   # USE THIS for any test that needs more than one bench operation.
#   tools/bench.sh session --hold 600 -- bash -lc '
#       B=/media/openclaw/projects/primer-ddr3-controller/tools/bench.sh
#       "$B" flash   -- openFPGALoader -b tangprimer20k design.fs
#       "$B" drain   -- cat /dev/tang-uart >/dev/null
#       "$B" capture -- cat /dev/tang-uart > out.log
#   '
#
#   NOTE: do NOT wrap a nested op in its own `timeout`. The session's hold is
#   the single clock for the whole test -- pass the duration to `session --hold`
#   and leave the ops unbounded. A nested call DOES cap itself at the remaining
#   session time as a backstop, so a stray inner `timeout` cannot outlive the
#   session, but a second clock is still the wrong shape: the outer hold is the
#   only thing that knows how long the whole test should take.
#
# WHY SESSION MODE EXISTS
#   A lock taken per-invocation protects each operation but NOT the sequence.
#   Between a flash and its capture the lock is unowned, so another process can
#   drive the bench and the tty RX buffer keeps accumulating the previous
#   design's output. Observed 2026-09-16 (rung A): a "locked" capture returned
#   6884 B containing the V-OLD tester's text interleaved THROUGHOUT -- 23 full
#   V-OLD test cycles -- mixed with the V-NEW heartbeats under test.
#   A test is ONE unit of ownership. Use `session`.
#
#   `drain` is the companion fix: after a flash, the freshly-programmed design's
#   output is indistinguishable from stale bytes already sitting in the tty
#   buffer. Drain the port first, then capture, so the capture contains only the
#   design you just loaded. Always drain (and better: capture long enough to see
#   a known marker) before trusting a verdict.
#
# ENV OVERRIDES
#   FPGA_BENCH_LOCK        lock file path
#                          default: ${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/fpga-bench.lock
#   FPGA_BENCH_WAIT        seconds to WAIT for the lock before giving up (10)
#   FPGA_BENCH_HOLD        seconds a single operation may HOLD the lock (1800)
#   FPGA_BENCH_NO_PREFLIGHT  set to 1 to skip the device-in-use pre-flight
#
#   FPGA_BENCH_SESSION and FPGA_BENCH_RC_FILE are INTERNAL -- set by bench.sh
#   itself when it takes the lock, and inherited by that session's nested calls.
#   Setting FPGA_BENCH_SESSION by hand just makes a caller skip the lock; it is
#   not a threat model (same UID -- see SESSION-CHILD MODE).
#
# PRE-FLIGHT IS ENTRY-ONLY
#   Device pre-flight runs once when a caller ENTERS (takes the lock). It is the
#   only defence against a process that ignores bench.sh entirely (a raw
#   `cat /dev/tang-uart`). Inside a session the pre-flight is not re-run, so a
#   NON-bench.sh process that grabs a node MID-SESSION will not be detected. The
#   pre-flight cannot be fixed to cover this (it is inherently an entry check);
#   it is stated here so a contaminated capture is at least attributable.
#
# EXIT CODES
#   75  lock busy -- another bench operation is in progress (or the wait expired)
#   76  pre-flight: a device node is already held by a process outside the lock
#   77  operation exceeded FPGA_BENCH_HOLD (runaway guard)
#   otherwise, the exit code of the wrapped command.
#
#   For `session`: the exit code is the WORST result in the session -- a nested
#   op that failed (including one hold-killed, 77) fails the whole session even
#   if the session body itself exited 0. 77 means the hold fired; it does NOT
#   by itself mean the bench is free, though with the group-kill fix the whole
#   process tree is signalled and the lock is released promptly.
#
# ALWAYS go through this script for anything that touches /dev/tang-*,
# openFPGALoader, JTAG, or the UART. A lock only serializes callers that take
# it -- a raw podman/`cat /dev/tang-uart` invocation bypasses it entirely and
# can still wedge the bench.

set -uo pipefail

KIND="${1:-bench}"
shift || true

WAIT="${FPGA_BENCH_WAIT:-10}"
HOLD="${FPGA_BENCH_HOLD:-1800}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --wait) WAIT="$2"; shift 2 ;;
        --hold) HOLD="$2"; shift 2 ;;
        --)     shift; break ;;
        *)      break ;;
    esac
done

if [[ $# -eq 0 ]]; then
    echo "bench.sh: no command given. Usage: bench.sh <kind> [--wait N] [--hold N] -- <cmd...>" >&2
    exit 64
fi

LOCK_FILE="${FPGA_BENCH_LOCK:-${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/fpga-bench.lock}"
LOCK_DIR="$(dirname "$LOCK_FILE")"
if [[ ! -d "$LOCK_DIR" ]]; then
    mkdir -p "$LOCK_DIR" 2>/dev/null || {
        echo "bench.sh: cannot create lock dir $LOCK_DIR" >&2
        exit 65
    }
fi

# ---------------------------------------------------------------------------
# SESSION MODE -- ONE atomic acquisition for a whole test.
#
# WHY: the per-invocation lock is NOT sufficient for a test that needs more
# than one bench operation. A flash and a capture taken as two separate
# acquisitions leave an unowned window in between where another process can
# drive the same FPGA resource, and where the tty RX buffer keeps accumulating
# the previous design's output. Observed 2026-09-16 (rung A): a capture held
# 6884 B with the V-OLD tester's text interleaved THROUGHOUT (23 full test
# cycles) alongside the V-NEW heartbeats, even though the lock was "used".
#
# RULE: a test is ONE session. Acquire the lock once, keep it for flash +
# capture + decode, and never release it mid-test.
#
#   tools/bench.sh session --hold 600 -- bash -c '
#       bench.sh flash   -- openFPGALoader ... design.fs
#       bench.sh drain   -- cat /dev/tang-uart >/dev/null
#       bench.sh capture -- cat /dev/tang-uart > out.log
#   '
#
# ---------------------------------------------------------------------------
# SESSION-CHILD MODE -- a nested call inside a LIVE session.
#
# The marker is TRUSTED. bench.sh sets it itself when it takes the lock, and the
# only processes that can inherit it are that session's own descendants (the
# export happens in bench.sh's process, not the caller's shell, so a parent or
# a later long-lived shell cannot pick it up).
#
# WHY THERE IS NO FORGERY DEFENCE HERE (do not add one back):
#   Every agent on this bench runs as the SAME UID. A same-user process can
#   bypass this lock by not calling bench.sh at all (a raw `openFPGALoader` or
#   `cat /dev/tang-uart` never touches this script), so a hand-set marker is not
#   a threat this script can or should address. The lock is a COOPERATIVE
#   serialization discipline between agents, NOT a security boundary. An earlier
#   round bolted on an fd-9 inode check, a live-holder probe, an ancestry walk
#   and a /proc/self/fdinfo lock-line check to defeat hand-set markers; each was
#   defeated in turn by a same-user process copying the marker or rewriting the
#   writable lock header, and the whole arms race defended nothing that same-UID
#   access does not already hand over. RULED OUT OF SCOPE (Rob, 2026-09-16);
#   see the "THREAT MODEL" section in docs/
#   BENCH_ACCESS_AND_ROUND4_CONTROL_ROOTCAUSE.md.
#
# What still matters and IS kept: a nested op must not OUTLIVE the session (it
# would strand the flock through the inherited fd 9), and a nested failure must
# not be invisible at the session boundary.
# ---------------------------------------------------------------------------
if [[ "${FPGA_BENCH_SESSION:-}" == "$LOCK_FILE" ]]; then
    echo "bench.sh: [session] $KIND (lock already held by this session)" >&2
    # Bound the nested op by the SESSION DEADLINE, not by a fresh full hold.
    #
    # A nested op must not OUTLIVE the session: the inherited fd 9 keeps the
    # flock HELD, so a nested op that runs past the session's hold limit leaves
    # the bench locked with no live session (measured: session --hold 4 kept the
    # lock 20 s while a plain nested op ran to completion, and ~25 s when the
    # nested op wrapped itself in `timeout 60 sleep 25`).
    #
    # Two mechanisms, both needed:
    #   1. no SECOND full-duration timeout here -- the session's own timeout runs
    #      the whole body in its process group and signals the group at the hold
    #      limit; and
    #   2. a nested call caps itself at the time REMAINING in the session
    #      (FPGA_BENCH_HOLD_DEADLINE), so even an inner `timeout` written by the
    #      caller expires no later than the session deadline and its own process
    #      group is reaped then.
    if [[ -n "${FPGA_BENCH_HOLD_DEADLINE:-}" ]]; then
        _remaining=$(( FPGA_BENCH_HOLD_DEADLINE - $(date +%s) ))
        if [[ $_remaining -lt 1 ]]; then
            echo "bench.sh: [session] $KIND SKIPPED -- session hold already expired" >&2
            rc=77
        else
            timeout --signal=TERM --kill-after=5 "$_remaining" "$@"
            rc=$?
            if [[ $rc -eq 124 || $rc -eq 137 ]]; then
                echo "bench.sh: [session] $KIND hit the SESSION HOLD CAP (${_remaining}s remaining) and was killed" >&2
                rc=77
            fi
        fi
    else
        "$@"
        rc=$?
    fi
    # Record the rc so the session can report the WORST nested result --
    # otherwise a hold-killed capture looks like success at the session
    # boundary.
    if [[ -n "${FPGA_BENCH_RC_FILE:-}" ]]; then
        printf '%s %s\n' "$rc" "$KIND" >> "$FPGA_BENCH_RC_FILE" 2>/dev/null || true
    fi
    exit $rc
fi

# ---------------------------------------------------------------------------
# Pre-flight: refuse if something OUTSIDE the lock already holds a bench node.
# The lock cannot stop a caller that ignores it; this at least refuses to ADD
# to the collision instead of silently corrupting a capture.
# ---------------------------------------------------------------------------
if [[ "${FPGA_BENCH_NO_PREFLIGHT:-0}" != "1" ]]; then
    for dev in /dev/tang-uart /dev/tang-jtag /dev/ttyUSB1 /dev/ttyUSB2; do
        [[ -e "$dev" ]] || continue
        holders="$(fuser "$dev" 2>/dev/null | tr -s ' ')"
        if [[ -n "${holders// /}" ]]; then
            echo "bench.sh: PRE-FLIGHT REFUSED -- $dev is already open by PID(s):$holders" >&2
            echo "  Another process is on the bench outside the lock. Wait for it, or" >&2
            echo "  confirm it is dead before retrying. (FPGA_BENCH_NO_PREFLIGHT=1 to skip)" >&2
            exit 76
        fi
    done
    if pgrep -x openFPGALoader >/dev/null 2>&1; then
        echo "bench.sh: PRE-FLIGHT REFUSED -- openFPGALoader already running (PID $(pgrep -x openFPGALoader | tr '\n' ' '))" >&2
        echo "  (FPGA_BENCH_NO_PREFLIGHT=1 to skip)" >&2
        exit 76
    fi
fi

# ---------------------------------------------------------------------------
# Take the exclusive lock for the WHOLE operation.
#
# Open READ-WRITE WITHOUT TRUNCATION (`<>`), not `>`. A plain `>` truncates the
# lock file the moment a caller opens it -- so a WAITER would wipe the holder's
# header record before failing to lock and reading it back, and the "LOCK BUSY"
# message would report `<unknown>` for a perfectly healthy holder. Verified:
# `exec 9>lock` on a file holding a header leaves it 0 bytes.
# ---------------------------------------------------------------------------
exec 9<>"$LOCK_FILE" || { echo "bench.sh: cannot open lock file $LOCK_FILE" >&2; exit 65; }

if ! flock -w "$WAIT" 9; then
    holder="$(cat "$LOCK_FILE" 2>/dev/null || true)"
    echo "bench.sh: LOCK BUSY -- another bench operation is in progress." >&2
    echo "  lock:   $LOCK_FILE" >&2
    echo "  holder: ${holder:-<unknown>}" >&2
    echo "  Waited ${WAIT}s. Try again, or raise FPGA_BENCH_WAIT." >&2
    exit 75
fi

# Mark this process tree as the lock owner. Any nested bench.sh call inherits
# this marker and runs WITHOUT re-locking (it is the same serialized owner) --
# the marker is trusted, see SESSION-CHILD MODE above for why no forgery check
# belongs here.
export FPGA_BENCH_SESSION="$LOCK_FILE"

# The session DEADLINE, so a nested call can cap itself at the time REMAINING
# rather than starting a fresh full hold. Without it an inner `timeout` in a
# session body outlives the session and strands a holder (measured 2026-09-16:
# session --hold 4 kept the lock ~25 s with a nested `timeout 60 sleep 25`).
FPGA_BENCH_HOLD_DEADLINE=$(( $(date +%s) + HOLD ))
export FPGA_BENCH_HOLD_DEADLINE

# Nested calls append their rc here so the session can report the WORST result.
# Without it a nested op killed for exceeding a hold is invisible at the session
# boundary (measured: a session whose capture was hold-killed exited 0).
SESSION_RC_FILE="$(mktemp "${TMPDIR:-/tmp}/fpga-bench-rc.XXXXXXXX")"
export FPGA_BENCH_RC_FILE="$SESSION_RC_FILE"

# ---------------------------------------------------------------------------
# Hold guard: a bench node that is grabbed and never released wedges everyone.
#
# WHY NOT JUST `timeout`: `timeout` signals the PROCESS GROUP of the body. But
# `timeout` calls setpgid and NOT setsid, so a nested `timeout` inside a session
# body gets its own NEW process group while keeping the session's SID. A group
# kill therefore cannot reach the level below it -- measured: a session with
# --hold 4 and a nested `timeout 60 sleep 25` kept the LOCK held for ~25 s.
#
# THE FIX is in SESSION-CHILD MODE, not here: a nested call re-arms a
# timeout capped at the REMAINING session time, so it expires no later than the
# session deadline and signals its OWN group. The nested op is therefore reaped
# at the deadline rather than at its own inner timeout, and the lock is released
# on time. Verified: session --hold 4 returns at 4 s (rc 77) with the lock freed
# and no surviving orphan. A setsid + pkill-by-SID guard was tried and reverted:
# the watchdog subshell inherited fd 9, so its `sleep` alone kept the lock held
# and broke serialization (10 selftest failures).
# ---------------------------------------------------------------------------
# Lock-file header: the human-readable record of who holds it and what for.
# Command is quoted per-arg so the record is not mangled by embedded spaces.
_quoted=""
for _a in "$@"; do _quoted+=" $(printf '%q' "$_a")"; done
echo "$$ $(date -Iseconds) kind=$KIND hold<=${HOLD}s cmd=${_quoted# }" > "$LOCK_FILE"

# The guard: bound the body by the hold and signal its process GROUP.
#
# A nested `timeout` inside the body gets its OWN process group, which a group
# signal cannot reach -- that is why the deadline mechanism in SESSION-CHILD
# MODE exists: a nested call caps itself at the REMAINING session time
# instead of starting a fresh full hold, so a nested op cannot outlive the
# session by wrapping itself in a long inner timeout. Verified: session --hold 4
# with `capture -- bash -c "timeout 60 sleep 25"` returns at 4 s with the lock
# released and no surviving orphan (was 25 s before the deadline cap).
timeout --signal=TERM --kill-after=10 "$HOLD" "$@"
rc=$?
if [[ $rc -eq 124 || $rc -eq 137 ]]; then
    echo "bench.sh: OPERATION EXCEEDED HOLD LIMIT (${HOLD}s) and was killed: $*" >&2
    rc=77
fi
worst=0
if [[ -s "$SESSION_RC_FILE" ]]; then
    while read -r _rc _kind; do
        [[ "$_rc" =~ ^[0-9]+$ ]] || continue
        if [[ "$_rc" -ne 0 && "$worst" -eq 0 ]]; then
            worst="$_rc"
            echo "bench.sh: [session] nested op '${_kind}' returned ${_rc}" >&2
        fi
    done < "$SESSION_RC_FILE"
fi
rm -f "$SESSION_RC_FILE" 2>/dev/null || true

# A nested failure outranks a clean session body -- a hold-killed capture must
# not report success.
if [[ "$worst" -ne 0 ]]; then
    exit "$worst"
fi

exit $rc