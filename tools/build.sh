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
#     - NO other container named `fpga-tools` running on this host
#     - bench is OFF (this script never touches /dev/tang-* or USB)
#
# What it does:
#     1. Asserts HOST_REPO_DIR is THIS working tree (the wrapper's default
#        points at a different clone -- see tools/fpga-run.sh line 30).
#     2. Asserts no fpga-tools container is already running (the wrapper
#        will rm -f fpga-tools and that would kill a peer's work).
#     3. Launches the container and runs `gw gw_sh -exit build_ddr3.tcl`.
#        The `-exit` flag is essential: without it gw_sh buffers stdout
#        and only flushes on process exit, so a killed mid-run build
#        looks like "no output".
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
# 1. Container-serialization gate. ONE fpga-tools name shared across peers;
#    a live run means someone else is mid-build (or mid-sim with fpga-sim
#    on this host under a different name -- not our problem here). Refuse.
# ----------------------------------------------------------------------------
if podman ps --filter name=fpga-tools --format '{{.Names}}' 2>/dev/null \
        | grep -q .; then
    echo "build.sh: BLOCKED: an 'fpga-tools' container is already running." >&2
    echo "  Another peer is using the shared toolchain container." >&2
    echo "  Wait for it to finish, then re-run." >&2
    podman ps --filter name=fpga-tools --format 'table {{.Names}}\t{{.Status}}\t{{.Created}}' >&2
    exit 2
fi

# ----------------------------------------------------------------------------
# 2. Launch the container. -exit on gw_sh forces stdout flush at exit.
#    Backgrounding this script is the recommended path (AGENTS.md #9,
#    `A previous attempt died on a 600 s foreground timeout and produced
#    a false blocker.`) -- this script does NOT background itself so its
#    caller (a CI runner, an agent) can decide.
# ----------------------------------------------------------------------------
echo "build.sh: starting fpga-tools container, repo=$REPO_DIR" >&2
echo "build.sh: full log -> $BUILD_LOG" >&2

cd "$REPO_DIR"
# EXPORT the host repo path so the inner `bash -lc` body in podman sees it.
# tools/fpga-run.sh reads HOST_REPO_DIR from its own env, but never exports
# it for the podman child -- so we have to. Likewise REPO is read by the
# Tcl driver.
export HOST_REPO_DIR="$REPO_DIR"
export REPO="$REPO_DIR"

"$TOOLS_DIR/fpga-run.sh" \
    bash -lc 'cd "$HOST_REPO_DIR" && REPO="$HOST_REPO_DIR" gw gw_sh -exit tools/build_ddr3.tcl' \
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
    awk '/^3\. Resource Usage Summary/,/^4\. I/O Bank Usage Summary/' "$RPT_TXT" \
        | grep -v '^4\. I/O Bank Usage Summary'
fi

# Timing -- look for the "<TNS>" line and the "slack" lines in the HTML/PNR log.
SYN_LOG="$REPO_DIR/build/ddr3/proj/ddr3/impl/pnr/ddr3.log"
if [[ -s "$SYN_LOG" ]]; then
    echo "----- Timing summary -----"
    grep -E -i 'slack|tns|wns|setup|hold' "$SYN_LOG" | head -20 || true
fi

# All synthesis/PnR warnings -- the brief says read EVERY warning. Highlight
# the substituted-configuration family in CAPS for easy eyeballing.
echo "----- WARNINGS (every line starting with WARN) -----"
if [[ -s "$BUILD_LOG" ]]; then
    grep -E '^WARN|^\s*WARN' "$BUILD_LOG" || echo "(none)"
    echo
    echo "----- Substituted-config warnings (EX0205/EX0210/PA1019/TA1123) -----"
    grep -E 'EX0205|EX0210|PA1019|TA1123' "$BUILD_LOG" || echo "(none -- PLL/clock configs accepted as-is)"
fi

echo "================ /build.sh summary ================"