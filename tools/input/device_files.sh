#!/bin/bash
set -euo pipefail
ACTION="${1:?Usage: device_files.sh export|import|recordings DEVICE LOCAL_PATH}"
DEVICE="${2:?Supply the Swan device serial}"
LOCAL_PATH="${3:?Supply a local path}"
REMOTE_PATH="/sdcard/Android/data/dev.apexmr.prototype/files/input-validation"
case "$ACTION" in
  export) pico-cli files pull "$REMOTE_PATH/input-config.json" "$LOCAL_PATH" --replace --device "$DEVICE" ;;
  import) pico-cli files push "$LOCAL_PATH" "$REMOTE_PATH/input-config.json" --device "$DEVICE" ;;
  recordings) pico-cli files pull "$REMOTE_PATH" "$LOCAL_PATH" --device "$DEVICE" ;;
  *) exit 2 ;;
esac
