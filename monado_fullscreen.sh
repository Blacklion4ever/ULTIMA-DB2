#!/usr/bin/env bash
set -euo pipefail

# Choix de l’écran X11 (index) et plein écran
export XRT_COMPOSITOR_XCB_DISPLAY=1
export XRT_COMPOSITOR_XCB_FULLSCREEN=1
export XRT_COMPOSITOR_FORCE_XCB=1

# Nettoyage du socket Monado (anciens chemins possibles)
rm -f /tmp/monado_comp_ipc 2>/dev/null || true
[ -n "${XDG_RUNTIME_DIR:-}" ] && rm -f "$XDG_RUNTIME_DIR/monado_comp_ipc" 2>/dev/null || true

# Lancer le service
exec monado-service

