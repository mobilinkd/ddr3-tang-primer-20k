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

# NO --name here, deliberately. Containers run with --rm are ephemeral and do
# not need a name. A fixed name made every checkout on this host collide, and
# worse, the `podman rm -f` that accompanied it force-killed whatever OTHER
# agent was running under that name -- with a bind-mounted repo the partial
# artifact then stayed on disk looking like a finished build, while the killed
# caller exited 137 with empty stdout/stderr. All agents on this host share one
# uid, so every clone could kill every other clone's build. Dropping the name
# removes the collision and the kill handle in one step; parallel runs of the
# same image are fine without it.

ARGS=(
    --rm
    --network=host
    # Inherit host supplementary groups (plugdev) so libusb can open the
    # FT2232 JTAG node (rootless podman drops them without this)
    --group-add keep-groups
    # JTAG/UART USB access for openFPGALoader
    --device /dev/bus/usb
    # Repo mounted at its own host path, so absolute paths are identical
    # inside and outside. :Z relabels for SELinux (Fedora host).
    -v "${HOST_REPO_DIR}:${HOST_REPO_DIR}:Z"
)

# Rootless podman on this Fedora host drops most env vars by default. Forward
# the well-known project vars if they are set in the caller's env, so
# Tcl scripts inside the container can read them.
for _v in HOST_REPO_DIR REPO HOST_LICENSE_DIR FPGA_IMAGE; do
    if [[ -n "${!_v:-}" ]]; then
        ARGS+=( --env "$_v" )
    fi
done

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
