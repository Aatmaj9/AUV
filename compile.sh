# Exit immediately on error
set -e

# Path to the sketch
SKETCH_PATH="./arduino/sookshma_masv01"

if [ "$(uname -m)" = "x86_64" ]; then
    arduino-cli compile --fqbn arduino:sam:arduino_due_x_dbg "$SKETCH_PATH"
else
    arduino-cli compile --fqbn per1234:sam:arduino_due_x_dbg "$SKETCH_PATH"
fi