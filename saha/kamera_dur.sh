#!/usr/bin/env bash
# Robottaki Orbbec kamerasını kapatır.
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
ssh -o BatchMode=yes "$ROBOT" 'pkill -INT -f "dabai[.]launch"'; sleep 3
ssh -o BatchMode=yes "$ROBOT" 'pgrep -fa "orbbec_camer[a]" || echo "kamera kapali"'
