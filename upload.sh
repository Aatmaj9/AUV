#!/bin/bash
set -e

# Path to the sketch
SKETCH_PATH="./arduino/sauvc"

# Serial port (change if needed)
PORT="/dev/portenta"

# Upload to Portenta H7 (M7 core)
arduino-cli upload \
  -p "$PORT" \
  --fqbn arduino:mbed_portenta:envie_m7 \
  "$SKETCH_PATH"

echo "🚀 Portenta H7 (M7 core) upload successful."