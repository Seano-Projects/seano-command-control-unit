#!/bin/bash
# =============================================================================
# SEANO USV — System Startup Script
# Menjalankan seluruh stack ROS2 via system.launch.py
#
# Usage:
#   ./start_seano.sh              → launch normal (field_test mode)
#   ./start_seano.sh --build      → build workspace dulu, lalu launch
#   ./start_seano.sh --no-vision  → launch tanpa vision stack & RTMP
#   ./start_seano.sh --actuation  → aktifkan vision actuation (RC override)
#   ./start_seano.sh --help       → tampilkan bantuan
# =============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"
LAUNCH_PKG="seano_startup"
LAUNCH_FILE="system.launch.py"
LOG_DIR="$WS_DIR/log/run"
SERIAL_DEVICE="${SEANO_SERIAL_DEVICE:-/dev/ttyACM0}"

# ── default launch arguments ─────────────────────────────────────────────────
ARG_VISION_STACK="false"
ARG_VISION_ACTUATION="false"
ARG_RTMP="false"
DO_BUILD=false
EXTRA_ARGS=()   # argumen key:=value tambahan untuk ros2 launch

# ── colour helpers ────────────────────────────────────────────────────────────
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
CYN='\033[0;36m'
BLD='\033[1m'
RST='\033[0m'

info()  { echo -e "${CYN}[INFO]${RST}  $*"; }
ok()    { echo -e "${GRN}[OK]${RST}    $*"; }
warn()  { echo -e "${YLW}[WARN]${RST}  $*"; }
error() { echo -e "${RED}[ERROR]${RST} $*"; }

sanitize_ros_env() {
    # Drop inherited overlay paths from unrelated workspaces to avoid
    # resolving packages from stale install prefixes.
    unset AMENT_PREFIX_PATH
    unset COLCON_PREFIX_PATH
    unset CMAKE_PREFIX_PATH
    unset PYTHONPATH
}

stop_serial_conflicts() {
    local pattern="mavproxy.py.*--master=${SERIAL_DEVICE}"
    local pids

    pids="$(pgrep -f "$pattern" || true)"
    if [[ -z "$pids" ]]; then
        return 0
    fi

    warn "Terdeteksi MAVProxy memakai ${SERIAL_DEVICE}; menghentikan agar MAVROS stabil."
    pkill -f "$pattern" || true
    sleep 1

    pids="$(pgrep -f "$pattern" || true)"
    if [[ -n "$pids" ]]; then
        warn "MAVProxy masih aktif, kirim SIGKILL untuk cegah konflik serial."
        pkill -9 -f "$pattern" || true
        sleep 1
    fi

    if pgrep -f "$pattern" >/dev/null 2>&1; then
        error "Gagal menghentikan MAVProxy di ${SERIAL_DEVICE}."
        error "Stop manual dulu, lalu jalankan ulang service SEANO."
        exit 1
    fi

    ok "Konflik serial berhasil dibersihkan."
}

ensure_gcs_arg() {
    local arg
    local ssh_ip
    local has_gcs_arg=false

    for arg in "${EXTRA_ARGS[@]}"; do
        if [[ "$arg" == gcs_url:=* ]]; then
            has_gcs_arg=true
            break
        fi
    done

    if [[ "$has_gcs_arg" == true ]]; then
        return 0
    fi

    # Priority: explicit env var set by operator.
    if [[ -n "${SEANO_GCS_URL:-}" ]]; then
        EXTRA_ARGS+=("gcs_url:=${SEANO_GCS_URL}")
        info "Menggunakan GCS URL dari env: ${SEANO_GCS_URL}"
        return 0
    fi

    # Fallback: pick SSH client IP so Mission Planner on current remote host can receive MAVLink.
    ssh_ip="$(awk '{print $1}' <<< "${SSH_CLIENT:-}")"
    if [[ -n "$ssh_ip" ]]; then
        EXTRA_ARGS+=("gcs_url:=udp://@${ssh_ip}:14550")
        warn "gcs_url tidak di-set; auto gunakan SSH client: udp://@${ssh_ip}:14550"
    else
        warn "gcs_url tidak di-set dan SSH client tidak terdeteksi; pakai default launch file."
    fi
}

# ── parse arguments ───────────────────────────────────────────────────────────
for arg in "$@"; do
    case "$arg" in
        --build)
            DO_BUILD=true
            ;;
        --no-vision)
            ARG_VISION_STACK="false"
            ARG_RTMP="false"
            warn "Vision stack & RTMP stream dinonaktifkan."
            ;;
        --actuation)
            ARG_VISION_ACTUATION="true"
            warn "Vision actuation (RC override) DIAKTIFKAN — pastikan area aman!"
            ;;
        --help|-h)
            echo ""
            echo -e "${BLD}SEANO USV Startup Script${RST}"
            echo ""
            echo "Usage:"
            echo "  $0 [OPTIONS] [key:=value ...]"
            echo ""
            echo "Options:"
            echo "  --build       Build seluruh workspace sebelum launch"
            echo "  --no-vision   Nonaktifkan vision stack & RTMP stream"
            echo "  --actuation   Aktifkan vision actuation / RC override"
            echo "  --help        Tampilkan help ini"
            echo ""
            echo "ROS2 launch args (key:=value) bisa langsung ditambahkan, contoh:"
            echo "  $0 enable_vision_stack:=false"
            echo "  $0 vision_det_max_fps:=3.0"
            echo "  $0 --build enable_rtmp_stream:=false"
            echo ""
            exit 0
            ;;
        *:=*)
            # Teruskan argumen key:=value langsung ke ros2 launch
            EXTRA_ARGS+=("$arg")
            ;;
        *)
            error "Argument tidak dikenal: $arg  (gunakan --help)"
            exit 1
            ;;    esac
done

# ── header ────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BLD}╔══════════════════════════════════════════╗${RST}"
echo -e "${BLD}║        SEANO USV — System Startup        ║${RST}"
echo -e "${BLD}╚══════════════════════════════════════════╝${RST}"
echo ""

# ── check ROS2 ────────────────────────────────────────────────────────────────
if [[ ! -f "$ROS_SETUP" ]]; then
    error "ROS2 Humble tidak ditemukan di $ROS_SETUP"
    error "Pastikan ROS2 Humble sudah terinstall."
    exit 1
fi

info "Source ROS2 Humble..."
info "Sanitizing inherited ROS overlay environment..."
sanitize_ros_env
# shellcheck source=/dev/null
source "$ROS_SETUP"

# ── optional build ────────────────────────────────────────────────────────────
if [[ "$DO_BUILD" == "true" ]]; then
    info "Build workspace: $WS_DIR"
    cd "$WS_DIR"
    colcon build --symlink-install 2>&1 | \
        awk '{ if (/error:|ERROR/) print "\033[0;31m" $0 "\033[0m"; \
               else if (/warning:|WARN/) print "\033[1;33m" $0 "\033[0m"; \
               else print $0 }'
    BUILD_STATUS="${PIPESTATUS[0]}"
    if [[ "$BUILD_STATUS" -ne 0 ]]; then
        error "Build gagal! Periksa error di atas."
        exit 1
    fi
    ok "Build selesai."
fi

# ── check install setup ───────────────────────────────────────────────────────
if [[ ! -f "$WS_SETUP" ]]; then
    error "install/setup.bash tidak ada. Jalankan dengan --build terlebih dahulu."
    exit 1
fi

info "Source workspace install..."
# shellcheck source=/dev/null
source "$WS_SETUP"

# ── preflight: cegah konflik serial autopilot ───────────────────────────────
stop_serial_conflicts
ensure_gcs_arg

# ── check package tersedia ────────────────────────────────────────────────────
if ! ros2 pkg list 2>/dev/null | grep -q "^${LAUNCH_PKG}$"; then
    error "Package '${LAUNCH_PKG}' tidak ditemukan."
    error "Coba jalankan dengan --build atau periksa apakah build sudah dilakukan."
    exit 1
fi

# ── buat log dir ──────────────────────────────────────────────────────────────
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date '+%Y-%m-%d_%H-%M-%S')
LOG_FILE="$LOG_DIR/seano_${TIMESTAMP}.log"

# ── ringkasan konfigurasi ─────────────────────────────────────────────────────
echo ""
echo -e "  ${BLD}Workspace    :${RST} $WS_DIR"
echo -e "  ${BLD}Launch       :${RST} $LAUNCH_PKG / $LAUNCH_FILE"
echo -e "  ${BLD}Vision stack :${RST} $ARG_VISION_STACK"
echo -e "  ${BLD}Actuation    :${RST} $ARG_VISION_ACTUATION"
echo -e "  ${BLD}RTMP stream  :${RST} $ARG_RTMP"
echo -e "  ${BLD}Log file     :${RST} $LOG_FILE"
echo ""

# ── launch ────────────────────────────────────────────────────────────────────
info "Memulai SEANO system... (Ctrl+C untuk berhenti)"
echo ""

cd "$WS_DIR"

ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" \
    enable_vision_stack:="$ARG_VISION_STACK" \
    enable_vision_actuation:="$ARG_VISION_ACTUATION" \
    enable_rtmp_stream:="$ARG_RTMP" \
    "${EXTRA_ARGS[@]}" \
    2>&1 | tee "$LOG_FILE"

echo ""
ok "SEANO system berhenti. Log tersimpan di: $LOG_FILE"
