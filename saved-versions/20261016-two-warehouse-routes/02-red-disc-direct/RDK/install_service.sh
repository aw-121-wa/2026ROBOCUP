#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_USER="${SUDO_USER:-${USER}}"
TEMPLATE="$PROJECT_DIR/systemd/rdk-disc.service.in"
TMP_SERVICE="$(mktemp)"
trap 'rm -f "$TMP_SERVICE"' EXIT

sed \
  -e "s|__USER__|$SERVICE_USER|g" \
  -e "s|__WORKDIR__|$PROJECT_DIR|g" \
  "$TEMPLATE" > "$TMP_SERVICE"

sudo install -m 0644 "$TMP_SERVICE" /etc/systemd/system/rdk-disc.service
sudo systemctl daemon-reload
sudo systemctl enable --now rdk-disc.service

echo
sudo systemctl --no-pager --full status rdk-disc.service || true

echo
echo "Installed and enabled rdk-disc.service"
echo "Logs:    journalctl -u rdk-disc.service -f"
echo "Status:  systemctl status rdk-disc.service"
echo "Stop:    sudo systemctl stop rdk-disc.service"
echo "Disable: ./uninstall_service.sh"
