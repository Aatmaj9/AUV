#!/bin/bash
set -e

# Path to the sketch
SKETCH_PATH="./arduino/sauvc"

# Compile for ESP32 Dev Module
arduino-cli compile \
  --fqbn esp32:esp32:esp32 \
  "$SKETCH_PATH"

echo "✅ ESP32 Compilation successful."