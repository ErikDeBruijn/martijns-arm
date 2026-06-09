#!/usr/bin/env bash
set -euo pipefail

# Eenmalig: XQuartz starten en TCP-connecties toestaan zodat Docker (Linux VM)
# zijn GUI naar de Mac kan sturen.

if ! pgrep -x XQuartz >/dev/null; then
  echo "XQuartz starten..."
  open -a XQuartz
  sleep 3
fi

# Allow connections from network clients (in XQuartz: Settings > Security)
defaults write org.xquartz.X11 nolisten_tcp -bool false
defaults write org.xquartz.X11 enable_iglx -bool true

# Sta lokaal Docker host toe te connecten naar X
xhost +localhost >/dev/null 2>&1 || /opt/X11/bin/xhost +localhost

echo "XQuartz draait. DISPLAY=host.docker.internal:0 zou moeten werken in container."
echo "Test in container:  apt list --installed | grep x11-apps  &&  xclock"
