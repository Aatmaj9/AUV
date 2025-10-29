# Exit immediately on error
set -e

# Path to the sketch
SKETCH_PATH="./arduino/sookshma_masv01"

if [ "$(uname -m)" = "x86_64" ]; then
    arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:sam:arduino_due_x_dbg "$SKETCH_PATH"
else
    arduino-cli upload -p /dev/ttyACM0 --fqbn per1234:sam:arduino_due_x_dbg "$SKETCH_PATH"
fi