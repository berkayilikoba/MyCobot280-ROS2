#!/usr/bin/env bash
# Kayıtlı dikdörtgen tur: robotta kaydı aç -> dikdortgen.py (odom ile) -> kaydı kapat.
# Önkoşul: nav_baslat.sh çalışıyor (çarpışma monitörü + AMCL referans), robot başlangıç işaretinde ve ilk kenara bakıyor,
# kamera isteniyorsa kamera_baslat.sh. Ek argümanlar dikdortgen.py'ye gider, ör.:
#   dikdortgen_tur.sh --tur 3 --hiz 0.15            dikdortgen_tur.sh --kenarlar 2,1,2,1 --sag
set -u
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
KAM=""; ssh -o BatchMode=yes "$ROBOT" 'pgrep -f "orbbec_camer[a]" >/dev/null' || KAM="--kamerasiz"
"$SAHA/kayit_baslat.sh" dikdortgen $KAM || exit 1
OUT=$VERI/dikdortgen_$(date +%Y%m%d_%H%M%S).csv
ros_humble; sleep 2
python3 "$SAHA/dikdortgen.py" --cerceve odom --log "$OUT" "$@"
sleep 2; "$SAHA/kayit_bitir.sh"
echo "köşe hataları: $OUT"
