#!/usr/bin/env bash
set -e

BAGS_DIR="./bags/videos"
SCRIPT="./zed2i/capture_video.py"

mkdir -p "$BAGS_DIR"

n=1
while [ -d "$BAGS_DIR/test$n" ]; do
  ((n++))
done

SAVE_DIR="$BAGS_DIR/test$n"
mkdir -p "$SAVE_DIR"

echo "🟢 Starting new video capture session in: $SAVE_DIR"

python3 "$SCRIPT" "$SAVE_DIR"
