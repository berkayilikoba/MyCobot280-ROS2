#!/usr/bin/env bash
# Aynı konfigürasyonu N kez, en fazla P paralel koşar (her koşu ayrı ROS_DOMAIN_ID).
# Kullanım:
#   bench_many.sh <cikti_kok> <mode> <N> <ilk_domain_id> [nav2_params.yaml] [P=3] [timeout_s=120]
# Çıktı: <cikti_kok>/r1 .. rN altında result.json; sonunda özet tablo.
# ROS alanları: koşu i -> ilk_domain_id + i, altyapı yeniden denemesi -> + i + 10.
# Hepsi 101 altında kalmalı: Linux'ta 102+ alanlar DDS portlarını geçici port
# aralığına (32768+) taşır ve rastgele keşif hatalarına yol açabilir.
set -u
ROOT=${1:?}; MODE=${2:?}; N=${3:?}; DOM0=${4:?}; PARAMS=${5:-}; P=${6:-3}; TO=${7:-120}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$ROOT"
seq 1 "$N" | xargs -P "$P" -I{} bash -c '
  i={}; sleep $(( (i-1) % '"$P"' * 12 ))
  "'"$HERE"'/bench_run.sh" "'"$ROOT"'/r$i" "'"$MODE"'" $(( '"$DOM0"' + i )) "'"$PARAMS"'" "'"$TO"'" >/dev/null 2>&1
  # Altyapı hatası (bringup düşmesi / /odom gelmemesi = rc 3/4) ölçüm değildir: bir kez yeniden dene.
  st=$(cat "'"$ROOT"'/r$i/status" 2>/dev/null)
  if [ "$st" = "BRINGUP_FAILED" ] || [ "$st" = "BENCH_RC=3" ] || [ -z "$st" ]; then
    mv "'"$ROOT"'/r$i" "'"$ROOT"'/r$i.altyapi_hatasi" 2>/dev/null
    sleep 5
    "'"$HERE"'/bench_run.sh" "'"$ROOT"'/r$i" "'"$MODE"'" $(( '"$DOM0"' + i + 10 )) "'"$PARAMS"'" "'"$TO"'" >/dev/null 2>&1
  fi
  echo "r$i bitti: $(cat "'"$ROOT"'/r$i/status" 2>/dev/null)"
'
python3 "$HERE/bench_summary.py" "$ROOT"
