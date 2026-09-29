#!/usr/bin/env bash
# build.sh -- unattended headless build for primer-ddr3-controller.
#
# Synthesises, places, routes, and packs the design to a Gowin .fs bitstream
# via the licensed vendor toolchain (Gowin IDE 1.9.11+ bundled in the
# `localhost/fpga-tools` podman image).
#
# Usage:
#     tools/build.sh                # build, log to build/build.log
#
# Prereqs (verified by AGENTS.md / tools/fpga-run.sh):
#     - podman installed, image localhost/fpga-tools present
#     - this repo mounted at the path you pass via HOST_REPO_DIR
#     - NO other podman container is currently bind-mounted against this
#       repo path (same-checkout concurrency would clobber build/)
#     - bench is OFF (this script never touches /dev/tang-* or USB)
#
# What it does:
#     1. Asserts HOST_REPO_DIR is THIS working tree (the wrapper's default
#        points at a different clone -- see tools/fpga-run.sh line 30).
#     2. Asserts no other container is already bind-mounted to this repo
#        (the wrapper refuses to start one while another is alive in this
#        checkout, but a *different* checkout's build can still run the
#        same image concurrently -- and that is the documented concurrency
#        model).
#     3. Launches the container and runs `gw gw_sh tools/build_ddr3.tcl`.
#        gw_sh in Gowin 1.9.11.x exits naturally when the Tcl script
#        completes (the in-container bash exits, stdin closes). The
#        `-exit` flag documented in some Gowin flow recipes makes
#        gw_sh exit BEFORE running any script -- verified empirically
#        on this image. Do not add `-exit`.
#     4. Tail of build log prints the .fs path, byte size, warning counts,
#        and every warning that suggests a substituted configuration
#        (EX0205/EX0210/PA1019/TA1123 family -- the ones that turn a green
#        build into a wrong-frequency chip).
#
# Output:
#     - build/ddr3/proj/ddr3/impl/pnr/ddr3.fs      (the bitstream)
#     - build/ddr3/proj/ddr3/impl/pnr/ddr3.rpt.txt (PnR report)
#     - build/ddr3/proj/ddr3/impl/gwsynthesis/ddr3_syn.rpt.html
#     - build/build.log                            (captured stdout+stderr)
#
# Never commit the .fs. The .gitignore already excludes build/* and *.fs.

set -euo pipefail

# ----------------------------------------------------------------------------
# 0. Resolve paths. We must hand tools/fpga-run.sh an absolute path that
#    resolves to THIS repo on the host; the wrapper's default is /media/
#    openclaw/projects/primer-ddr3-controller which is the DISPATCHER's
#    clone, not ours.
# ----------------------------------------------------------------------------
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." >/dev/null 2>&1 && pwd)"
TOOLS_DIR="$REPO_DIR/tools"

# PWD check: refuse to run from outside the repo. Catches the case where
# someone sets HOST_REPO_DIR to a stale or sibling clone.
if [[ ! -f "$REPO_DIR/ddr3.gprj" ]]; then
    echo "build.sh: REPO_DIR=$REPO_DIR does not contain ddr3.gprj -- aborting." >&2
    exit 1
fi

LOG_DIR="$REPO_DIR/build"
mkdir -p "$LOG_DIR"
BUILD_LOG="$LOG_DIR/build.log"

# ----------------------------------------------------------------------------
# 1. Container-serialization gate. Since the wrapper dropped --name (bcc1836),
#    detect a live build in THIS checkout by scanning podman for any
#    running container whose mount list includes this repo path. Two
#    concurrent builds against the same bind-mounted source will stomp
#    each other's build/ tree -- refuse early. Other checkouts running
#    the same image in parallel are fine; the image is the concurrency-safe
#    unit, the bind mount is the hazard.
# ----------------------------------------------------------------------------
if podman ps --format '{{.Names}} {{.Mounts}}' 2>/dev/null \
        | grep -F "$REPO_DIR" \
        | grep -q .; then
    echo "build.sh: BLOCKED: another build is already running against" >&2
    echo "  $REPO_DIR" >&2
    echo "  Two concurrent builds with the same bind mount will stomp each" >&2
    echo "  other's build/ tree. Wait for it to finish, then re-run." >&2
    podman ps --filter status=running \
              --format 'table {{.Names}}\t{{.Status}}\t{{.Created}}\t{{.Command}}' >&2 \
        | head -20 >&2
    exit 2
fi

# ----------------------------------------------------------------------------
# 2. Launch the container. -exit on gw_sh forces stdout flush at exit.
#    Backgrounding this script is the recommended path (AGENTS.md #9,
#    `A previous attempt died on a 600 s foreground timeout and produced
#    a false blocker.`) -- this script does NOT background itself so its
#    caller (a CI runner, an agent) can decide.
# ----------------------------------------------------------------------------
echo "build.sh: starting fpga-tools container (ephemeral name), repo=$REPO_DIR" >&2
echo "build.sh: full log -> $BUILD_LOG" >&2

cd "$REPO_DIR"
# EXPORT the host repo path so the inner `bash -lc` body in podman sees it.
# tools/fpga-run.sh reads HOST_REPO_DIR from its own env, but never exports
# it for the podman child -- so we have to. Likewise REPO is read by the
# Tcl driver.
export HOST_REPO_DIR="$REPO_DIR"
export REPO="$REPO_DIR"

"$TOOLS_DIR/fpga-run.sh" \
    bash -lc 'cd "$HOST_REPO_DIR" && REPO="$REPO" gw gw_sh tools/build_ddr3.tcl' \
    2>&1 | tee "$BUILD_LOG"

# ----------------------------------------------------------------------------
# 3. Summarise. Never trust a green build -- read the warnings. A bitstream
#    that builds with a substituted PLL config will run at the wrong rate.
# ----------------------------------------------------------------------------
FS_PATH="$REPO_DIR/build/ddr3/proj/ddr3/impl/pnr/ddr3.fs"
RPT_TXT="$REPO_DIR/build/ddr3/proj/ddr3/impl/pnr/ddr3.rpt.txt"

echo
echo "================ build.sh summary ================"

if [[ -s "$FS_PATH" ]]; then
    echo "BITSTREAM: $FS_PATH"
    echo "SIZE:      $(stat -c%s "$FS_PATH") bytes"
else
    echo "BITSTREAM: MISSING (build did not produce ddr3.fs)" >&2
    exit 3
fi

# Resource usage, pulled from the text PnR report.
if [[ -s "$RPT_TXT" ]]; then
    echo "----- Resource Usage (from ddr3.rpt.txt) -----"
    # Section headings include 'I/O' which has a literal '/', so use grep -n
    # to find both section boundaries and sed to slice between their line
    # numbers -- simpler than an awk range pattern.
    # Disable pipefail locally: a grep that returns no match exits 1, which
    # under 'set -o pipefail' would abort the script mid-summary. The
    # `|| echo` fallback further down is not enough if the pipeline is in a
    # command substitution (the subshell inherits pipefail). Use simple
    # grep-and-handle-no-match explicitly.
    set +o pipefail
    start=$(grep -n '^3\. Resource Usage Summary$' "$RPT_TXT" | head -1 | cut -d: -f1)
    end=$(grep -n '^4\. I/O Bank Usage Summary$' "$RPT_TXT" | head -1 | cut -d: -f1)
    set -o pipefail
    if [[ -n "$start" && -n "$end" ]]; then
        sed -n "${start},${end}p" "$RPT_TXT" | grep -v '^4\. I/O Bank Usage Summary$'
    fi
fi

# Timing -- pull from the timing-paths text file and the HTML summary.
TIMING_PATHS="$REPO_DIR/build/ddr3/proj/ddr3/impl/pnr/ddr3.timing_paths"
TR_HTML="$REPO_DIR/build/ddr3/proj/ddr3/impl/pnr/ddr3_tr_content.html"
echo "----- Timing summary -----"
if [[ -s "$TR_HTML" ]]; then
    # Extract "Numbers of Setup/Hold Violated Endpoints" and the corresponding
    # integer value from the HTML.
    python3 -c '
import re, sys
text = open("'"$TR_HTML"'").read()
for label in ("Setup", "Hold"):
    m = re.search(r"Numbers of " + label + r" Violated Endpoints</td>\s*<td>([0-9]+)</td>", text)
    if m:
        print(f"  {label} violated endpoints: {m.group(1)}")
    else:
        print(f"  {label} violated endpoints: (label not found)")
'
fi
if [[ -s "$TIMING_PATHS" ]]; then
    # Worst-case SETUP and HOLD slack (first numeric line after each header).
    # awk exits 1 if the pattern never matches; guard with `|| true` so the
    # substitution under 'set -o pipefail' doesn't abort the script.
    setup=$(awk '/^SETUP$/{getline; print; exit}' "$TIMING_PATHS" || true)
    hold=$(awk '/^HOLD$/{getline; print; exit}' "$TIMING_PATHS" || true)
    echo "  Worst SETUP slack (ns): ${setup:-?}"
    echo "  Worst HOLD  slack (ns): ${hold:-?}"
fi

# All synthesis/PnR warnings -- the brief says read EVERY warning. Highlight
# the substituted-configuration family in CAPS for easy eyeballing.
echo "----- WARNINGS (every line starting with WARN) -----"
if [[ -s "$BUILD_LOG" ]]; then
    grep -E '^WARN|^[[:space:]]*WARN' "$BUILD_LOG" || echo "(none)"
    echo
    # Anchor on the WARN  (EXxxxx)  format so the grep doesn't match the
    # LITERAL codes that appear in our own header text. The header lists
    # EX0205/EX0210/PA1019/TA1123 as human-readable labels; a naive grep
    # would match the header line itself, printing it twice (once for the
    # real echo, once from grep), and never reaching the '(none)' fallback.
    echo "----- Substituted-config warnings (gating EX0205/EX0210/PA1019/TA1123) -----"
    grep -E 'WARN[[:space:]]+\((EX0205|EX0210|PA1019|TA1123)\)' "$BUILD_LOG" \
        || echo "(none -- PLL/clock configs accepted as-is)"
fi

echo "================ /build.sh summary ================"