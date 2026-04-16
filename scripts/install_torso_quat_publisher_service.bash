#!/bin/bash
# Installs torso_quat_publisher as a systemd user service.
#
# Loads the pre-computed mounting offset (offset.npy from calibrate.py) and
# continuously publishes corrected torso quaternions to the "TorsoQuat"
# CycloneDDS topic.
#
# Requires:
#   - offset.npy produced by calibrate.py (must exist before running this script)
#   - imu_publisher.service installed and running

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PYTHON="$(which python3)"
SERVICE_NAME="torso_quat_publisher"
SERVICE_FILE="$HOME/.config/systemd/user/${SERVICE_NAME}.service"

# ── Pre-flight checks ─────────────────────────────────────────────────────────

if [[ ! -f "${SCRIPT_DIR}/offset.npy" ]]; then
    echo "ERROR: offset.npy not found in ${SCRIPT_DIR}" >&2
    echo "       Run calibrate.py first to generate it." >&2
    exit 1
fi

IMU_SERVICE_FILE="$HOME/.config/systemd/user/imu_publisher.service"
if [[ ! -f "${IMU_SERVICE_FILE}" ]]; then
    echo "WARNING: imu_publisher.service is not installed." >&2
    echo "         Run scripts/install_imu_publisher_service.bash first." >&2
fi

# ── Write service file ────────────────────────────────────────────────────────

mkdir -p "$HOME/.config/systemd/user"

cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Torso Quaternion Publisher (witimu -> calibrated TorsoQuat DDS topic)
After=network.target imu_publisher.service
Wants=imu_publisher.service
StartLimitIntervalSec=0

[Service]
Type=simple
WorkingDirectory=${SCRIPT_DIR}
Environment="CYCLONEDDS_URI=file://${SCRIPT_DIR}/cyclonedds.xml"
ExecStart=${PYTHON} ${SCRIPT_DIR}/torso_quat_publisher.py
Restart=on-failure
RestartSec=2s
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=default.target
EOF

echo "Service file written to: $SERVICE_FILE"

# ── Enable and (re)start ──────────────────────────────────────────────────────

systemctl --user daemon-reload
systemctl --user enable "$SERVICE_NAME"

if systemctl --user is-active --quiet "$SERVICE_NAME"; then
    systemctl --user restart "$SERVICE_NAME"
else
    systemctl --user start "$SERVICE_NAME" || true
fi

echo ""
echo "Service status:"
systemctl --user status "$SERVICE_NAME" --no-pager
echo ""
echo "To view logs:    journalctl --user -u $SERVICE_NAME -f"
echo "To stop:         systemctl --user stop $SERVICE_NAME"
echo "To disable:      systemctl --user disable $SERVICE_NAME"