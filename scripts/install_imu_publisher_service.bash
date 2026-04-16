#!/bin/bash
# Installs imu_publisher as a systemd user service that autostarts on login
# and restarts every 2 seconds on failure.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PYTHON="$(which python)"
SERVICE_NAME="imu_publisher"
SERVICE_FILE="$HOME/.config/systemd/user/${SERVICE_NAME}.service"

mkdir -p "$HOME/.config/systemd/user"

cat > "$SERVICE_FILE" << EOF
[Unit]
Description=IMU Publisher (BWT901CL → CycloneDDS)
After=network.target
StartLimitIntervalSec=0

[Service]
Type=simple
WorkingDirectory=${SCRIPT_DIR}
Environment="CYCLONEDDS_URI=file://${SCRIPT_DIR}/cyclonedds.xml"
ExecStart=${PYTHON} ${SCRIPT_DIR}/imu_publisher.py
Restart=on-failure
RestartSec=2s
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=default.target
EOF

echo "Service file written to: $SERVICE_FILE"

systemctl --user daemon-reload
systemctl --user enable "$SERVICE_NAME"
systemctl --user start  "$SERVICE_NAME" || true

echo ""
echo "Service status:"
systemctl --user status "$SERVICE_NAME" --no-pager
echo ""
echo "To view logs:    journalctl --user -u $SERVICE_NAME -f"
echo "To stop:         systemctl --user stop $SERVICE_NAME"
echo "To disable:      systemctl --user disable $SERVICE_NAME"
