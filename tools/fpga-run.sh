#!/usr/bin/env bash
# Run the FPGA toolchain container for primer-ddr3-controller.
#
# Owned by this project. It shares nothing with any other tree on this host:
# the repo is mounted at its own absolute path and the in-container path IS
# that path, so absolute paths in build scripts and $readmemh in testbenches
# always resolve.
#
# - Mounts the license directory read-only at /license ONLY IF it exists. The
#   image ships the Gowin Education edition, which needs no license: no MAC
#   check, no node-lock, nothing to mount.
# - --network=host so vendor tools can reach host resources.
# - USB device access for openFPGALoader JTAG programming.
#
# Overridable via environment:
#   HOST_REPO_DIR      host path of this repo
#                      default: /media/openclaw/projects/primer-ddr3-controller
#   HOST_LICENSE_DIR   host dir holding gowin_*.lic (OPTIONAL)
#                      default: /media/openclaw/projects/gowin
#   FPGA_IMAGE         container image
#                      default: localhost/fpga-tools
#                      use localhost/fpga-sim for Verilator/C++ sims
#
# Usage: fpga-run.sh [command...]          # default image (fpga-tools)
#        FPGA_IMAGE=fpga-sim fpga-run.sh [command...]
#        (no command = interactive shell)

set -euo pipefail

HOST_REPO_DIR="${HOST_REPO_DIR:-/media/openclaw/projects/primer-ddr3-controller}"
HOST_LICENSE_DIR="${HOST_LICENSE_DIR:-/media/openclaw/projects/gowin}"
IMAGE="${FPGA_IMAGE:-localhost/fpga-tools}"

if [[ ! -d "$HOST_REPO_DIR" ]]; then
    echo "fpga-run.sh: repo dir not found: $HOST_REPO_DIR" >&2
    exit 1
fi

MOUNT_LICENSE=0
if [[ -d "$HOST_LICENSE_DIR" ]]; then
    MOUNT_LICENSE=1
else
    echo "fpga-run.sh: no license dir at $HOST_LICENSE_DIR -- running" >&2
    echo "  license-free (Gowin Education edition). This is expected." >&2
fi

# NEVER use a fixed container name here. Every checkout must get its OWN name.
#
# MEASURED FAILURE THIS PREVENTS: with a fixed name, the `podman rm -f` below
# force-kills any OTHER agent's running build. With a bind-mounted repo the
# killed build's partial artifacts STAY on disk, and the killed caller's run
# exits with no error at all -- so the next agent finds a half-written
# bitstream and a truncated log and can read it as a finished build. The image
# itself is fine with concurrency (two containers from the same image run at
# once); the fixed NAME is the whole problem, because all agents on this host
# share one uid and can therefore see and kill each other's containers.
#
# Derive the name from the repo path so clones cannot collide. Override with
# FPGA_CONTAINER_NAME if you need to.
_REPO_KEY="$(printf '%s' "$HOST_REPO_DIR" | cksum | cut -d' ' -f1)"
CONTAINER_NAME="${FPGA_CONTAINER_NAME:-fpga-tools-$(basename "$HOST_REPO_DIR")-${_REPO_KEY}}"

# A name that is already RUNNING is another live build in the same checkout.
# Refuse rather than force-killing it: a killed build leaves a partial artifact
# that reads as output. Set FPGA_FORCE=1 only to clear a wedged one.
if podman container exists "$CONTAINER_NAME" 2>/dev/null; then
    if [[ "$(podman inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" == "true" ]]; then
        if [[ "${FPGA_FORCE:-0}" != "1" ]]; then
            echo "fpga-run.sh: REFUSING to start: $CONTAINER_NAME is already running." >&2
            echo "  Another build is live in this checkout. Killing it would leave" >&2
            echo "  partial artifacts that look like output. Wait for it, or set" >&2
            echo "  FPGA_FORCE=1 if you know it is wedged." >&2
            exit 3
        fi
        echo "fpga-run.sh: FPGA_FORCE=1, killing the running $CONTAINER_NAME" >&2
    fi
    podman rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
fi

ARGS=(
    --rm
    --network=host
    --name "${CONTAINER_NAME}"
    # Inherit host supplementary groups (plugdev) so libusb can open the
    # FT2232 JTAG node (rootless podman drops them without this)
    --group-add keep-groups
    # JTAG/UART USB access for openFPGALoader
    --device /dev/bus/usb
    # Repo mounted at its own host path, so absolute paths are identical
    # inside and outside. :Z relabels for SELinux (Fedora host).
    -v "${HOST_REPO_DIR}:${HOST_REPO_DIR}:Z"
)

if [[ "$MOUNT_LICENSE" == "1" ]]; then
    ARGS+=( -v "${HOST_LICENSE_DIR}:/license:ro,Z" )
fi

if [[ $# -gt 0 ]]; then
    # The Gowin CLI tools MUST be invoked through the `gw` wrapper
    # (/usr/local/bin/gw), which sets LD_LIBRARY_PATH and
    # QT_QPA_PLATFORM=offscreen. Calling `gw_sh` directly dies on libsmime3.so,
    # and because the build script is piped to a log it can look like it merely
    # "produced nothing" -- a 2-line dead log masquerading as a build result.
    _argv="$*"
    if [[ "$_argv" == *gw_sh* || "$_argv" == *gowin_ide* ]]; then
        if [[ "$_argv" != *"gw gw_sh"* && "$_argv" != *"gw gowin_ide"* ]]; then
            echo "fpga-run.sh: REFUSING to run a Gowin CLI without the wrapper." >&2
            echo "  argv: $_argv" >&2
            echo "  Use: gw gw_sh <script>   (not gw_sh <script>)" >&2
            exit 2
        fi
    fi
    exec podman run "${ARGS[@]}" "--workdir" "${HOST_REPO_DIR}" "$IMAGE" "$@"
else
    exec podman run "${ARGS[@]}" -it "--workdir" "${HOST_REPO_DIR}" "$IMAGE"
fi
