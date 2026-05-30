#!/usr/bin/env bash
# Push the robot model + Isaac Lab task package to the GB10.
# Run from repo root on the laptop:  bash training/remote/sync.sh
set -euo pipefail

HOST="${ROBOTDOG_GB10:-asus@gx10-f3fb}"
DEST="${ROBOTDOG_REMOTE_DIR:-~/robotdog}"

echo ">>> syncing model/ + training/ to $HOST:$DEST"
ssh "$HOST" "mkdir -p $DEST"
rsync -az --delete \
  --exclude '__pycache__' --exclude '*.pyc' \
  model/ "$HOST:$DEST/model/"
rsync -az --delete \
  --exclude '__pycache__' --exclude '*.pyc' \
  training/ "$HOST:$DEST/training/"
echo ">>> done. Register the task by adding $DEST to PYTHONPATH (see train.sh)."
