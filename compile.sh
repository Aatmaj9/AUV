#!/bin/bash
set -e

# Path to the sketch
SKETCH_PATH="./arduino/sauvc"

# Compile for Portenta H7 (M7 core)
arduino-cli compile \
  --fqbn arduino:mbed_portenta:envie_m7 \
  "$SKETCH_PATH"

echo "✅ Portenta H7 (M7 core) code compilation successful."