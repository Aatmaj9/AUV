#!/bin/bash
set -e

SKETCH_PATH="./arduino/sauvc"
PORT="/dev/portenta"

echo "Going to bootloader mode for code upload..."

stty -F "$PORT" 1200 || true
sleep 3

echo "Uploading..."

arduino-cli upload \
  --fqbn arduino:mbed_portenta:envie_m7 \
  -p "$PORT" \
  "$SKETCH_PATH"

echo "🚀 Upload successful."