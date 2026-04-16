#!/bin/bash
# Rebuild package ROS2 tertentu lalu restart seano.service
# Default package: seano_startup

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
SERVICE_NAME="seano.service"

PKGS=("seano_startup")
NO_SUDO=false

usage() {
    cat <<'EOF'
Usage:
  ./scripts/rebuild_restart_seano.sh [OPTIONS]

Options:
  --pkg <name>        Tambah package untuk dibuild (bisa diulang)
  --no-sudo           Jangan pakai sudo saat restart/status service
  -h, --help          Tampilkan bantuan

Contoh:
  ./scripts/rebuild_restart_seano.sh
  ./scripts/rebuild_restart_seano.sh --pkg seano_startup --pkg seano_command
EOF
}

log() {
    echo "[INFO] $*"
}

warn() {
    echo "[WARN] $*"
}

err() {
    echo "[ERROR] $*" >&2
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pkg)
            [[ $# -lt 2 ]] && { err "Missing value for --pkg"; exit 1; }
            PKGS+=("$2")
            shift 2
            ;;
        --no-sudo)
            NO_SUDO=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            err "Unknown argument: $1"
            usage
            exit 1
            ;;
    esac
done

if [[ ! -f "$ROS_SETUP" ]]; then
    err "ROS setup tidak ditemukan: $ROS_SETUP"
    exit 1
fi

if [[ ! -d "$WS_DIR" ]]; then
    err "Workspace tidak ditemukan: $WS_DIR"
    exit 1
fi

# Deduplicate package list sambil jaga urutan
DEDUP_PKGS=()
for p in "${PKGS[@]}"; do
    skip=false
    for e in "${DEDUP_PKGS[@]}"; do
        if [[ "$p" == "$e" ]]; then
            skip=true
            break
        fi
    done
    if [[ "$skip" == false ]]; then
        DEDUP_PKGS+=("$p")
    fi
done

log "Workspace: $WS_DIR"
log "Build package: ${DEDUP_PKGS[*]}"

cd "$WS_DIR"
# shellcheck source=/dev/null
set +u
source "$ROS_SETUP"
set -u

colcon build --packages-select "${DEDUP_PKGS[@]}"

if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    set +u
    source "$WS_DIR/install/setup.bash"
    set -u
else
    warn "install/setup.bash belum ada setelah build"
fi

if [[ "$NO_SUDO" == true ]]; then
    log "Restart service tanpa sudo: $SERVICE_NAME"
    systemctl restart "$SERVICE_NAME"
    systemctl --no-pager --full status "$SERVICE_NAME" | head -n 40
else
    log "Restart service dengan sudo: $SERVICE_NAME"
    sudo systemctl restart "$SERVICE_NAME"
    sudo systemctl --no-pager --full status "$SERVICE_NAME" | head -n 40
fi

log "Selesai. Untuk monitor log realtime jalankan:"
if [[ "$NO_SUDO" == true ]]; then
    echo "  journalctl -u $SERVICE_NAME -f"
else
    echo "  sudo journalctl -u $SERVICE_NAME -f"
fi
