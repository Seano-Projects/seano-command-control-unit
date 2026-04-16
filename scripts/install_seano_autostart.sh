#!/bin/bash

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "Script ini harus dijalankan dengan sudo."
    echo "Contoh: sudo ./scripts/install_seano_autostart.sh"
    exit 1
fi

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_NAME="seano.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
ENV_PATH="/etc/default/seano"

RUN_USER="${SUDO_USER:-${USER}}"
RUN_HOME="$(getent passwd "${RUN_USER}" | cut -d: -f6)"

if [[ -z "${RUN_HOME}" ]]; then
    echo "Gagal menentukan home directory untuk user ${RUN_USER}."
    exit 1
fi

if [[ ! -f "${WS_DIR}/start_seano.sh" ]]; then
    echo "start_seano.sh tidak ditemukan di ${WS_DIR}."
    exit 1
fi

cat > "${ENV_PATH}" <<EOF
SEANO_START_ARGS=""
EOF

cat > "${SERVICE_PATH}" <<EOF
[Unit]
Description=SEANO ROS2 Startup
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=${RUN_USER}
WorkingDirectory=${WS_DIR}
Environment=HOME=${RUN_HOME}
EnvironmentFile=-${ENV_PATH}
ExecStart=/bin/bash -lc 'exec "${WS_DIR}/start_seano.sh" \$SEANO_START_ARGS'
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=20
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

chmod 644 "${SERVICE_PATH}" "${ENV_PATH}"
chmod +x "${WS_DIR}/start_seano.sh"

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"

echo "Service terpasang: ${SERVICE_PATH}"
echo "Environment file: ${ENV_PATH}"
echo "Autostart aktif. Untuk langsung menjalankan sekarang gunakan:"
echo "  sudo systemctl start ${SERVICE_NAME}"
echo ""
echo "Perintah cek status:"
echo "  systemctl status ${SERVICE_NAME}"
echo "  journalctl -u ${SERVICE_NAME} -f"
echo ""
echo "Jika ingin tambah argumen startup, edit ${ENV_PATH}."
echo "Contoh: SEANO_START_ARGS=\"--no-vision\""