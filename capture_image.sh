#!/usr/bin/env bash
set -e

BAGS_DIR="./bags/images"
SCRIPT="./zed2i/capture_image.py"

mkdir -p "$BAGS_DIR"

n=1
while [ -d "$BAGS_DIR/test$n" ]; do
  ((n++))
done

SAVE_DIR="$BAGS_DIR/test$n"
mkdir -p "$SAVE_DIR"

echo "🟢 Starting new image capture session in: $SAVE_DIR"

python3 "$SCRIPT" "$SAVE_DIR"
