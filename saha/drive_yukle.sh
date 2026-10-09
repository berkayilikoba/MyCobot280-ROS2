#!/usr/bin/env bash
# Yerel klasörü Google Drive klasörüne yükler (rclone; yalnız kopyalar, Drive'da hiçbir şey silmez) ve boyutları doğrular.
# Kurulum (bir kez): sudo apt install rclone && rclone config create gdrive drive scope drive   (tarayıcıda giriş)
# Kullanım: drive_yukle.sh <yerel_klasör> <drive_klasör_id> [hedef_alt_klasör_adı=yerel klasör adı]
#   drive_klasör_id: Drive adresindeki .../folders/<ID> kısmı. Ör. hocanın "Robot Folder"ı: 1FPr_aRQvwX3CJ5O-f0bj-8IDx6XJpgzd
set -u
YEREL=${1:?yerel klasör}; ID=${2:?drive klasör id}; AD=${3:-$(basename "$YEREL")}
command -v rclone >/dev/null || { echo "rclone yok: sudo apt install rclone"; exit 1; }
rclone listremotes | grep -q '^gdrive:' || { echo "gdrive tanımlı değil: rclone config create gdrive drive scope drive"; exit 1; }
# SQLite geçici dosyaları (bag açılınca oluşur) yüklenmez
FILTRE=(--exclude '*.db3-shm' --exclude '*.db3-wal' --exclude '__pycache__/**')
rclone copy "$YEREL" "gdrive:$AD" --drive-root-folder-id "$ID" "${FILTRE[@]}" --drive-chunk-size 64M --transfers 4 \
  --stats 30s --stats-one-line -v 2>&1 | grep -E "Transferred:|ERROR" | tail -5
echo "== doğrulama (boyut)"
rclone check "$YEREL" "gdrive:$AD" --drive-root-folder-id "$ID" "${FILTRE[@]}" --size-only 2>&1 | grep -E "NOTICE|ERROR" | tail -4
