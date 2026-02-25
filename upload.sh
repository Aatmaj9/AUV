#!/bin/bash
set -e

# Path to the sketch
SKETCH_PATH="./arduino/sauvc"

# Serial port (change if needed)
PORT="/dev/arduino"

# Upload to ESP32 via USB
arduino-cli upload \
  -p "$PORT" \
  --fqbn esp32:esp32:esp32 \
  "$SKETCH_PATH"

echo "🚀 ESP32 Upload successful."