#!/usr/bin/env bash
set -euo pipefail
sudo systemctl disable --now rdk-disc.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/rdk-disc.service
sudo systemctl daemon-reload
sudo systemctl reset-failed rdk-disc.service 2>/dev/null || true
echo "rdk-disc.service removed"
