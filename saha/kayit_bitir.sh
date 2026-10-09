#!/usr/bin/env bash
# Robottaki kaydı kapatır, konu sayılarını/hızlarını gösterir.  Kullanım: kayit_bitir.sh [--indir]
#   --indir : kaydı bu PC'ye ($LIMO_VERI, varsayılan ~/limo_veri) kopyalar (kamerali kayıt GB'larca; Wi-Fi'da dakikalar sürer)
set -u
source "$(dirname "$(readlink -f "$0")")/ortam.sh"
DIZIN=$(cat "$VERI/aktif_kayit" 2>/dev/null) || { echo "aktif kayit yok"; exit 1; }
ssh -o BatchMode=yes "$ROBOT" 'pkill -INT -f "ros2 bag recor[d]"'; sleep 6
robot_cmd "for b in $DIZIN/sensor $DIZIN/kamera; do [ -d \$b ] || continue; echo == \$b; ros2 bag info \$b 2>/dev/null | python3 -c '
import sys,re
s=sys.stdin.read(); d=float(re.search(r\"Duration:\s+([\d.]+)\",s).group(1)); print(\"sure %.1f s, boyut\"%d, re.search(r\"Bag size:\s+(.*)\",s).group(1))
for t,c in re.findall(r\"Topic: (\S+) .*?Count: (\d+)\",s): print(\"  %-32s %6s mesaj  %6.1f Hz\"%(t,c,int(c)/d))'; done"
if [ "${1:-}" = "--indir" ]; then
  mkdir -p "$VERI/robot_kayit"; echo "kopyalaniyor -> $VERI/robot_kayit/"
  scp -rq "$ROBOT:$DIZIN" "$VERI/robot_kayit/" && echo "tamam: $VERI/robot_kayit/$(basename "$DIZIN")"
fi
