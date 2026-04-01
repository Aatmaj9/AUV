echo "🔄 Activating sensors..."
sudo chmod 666 /dev/ping2 /dev/ping360 /dev/sbg /dev/arduino /dev/frontcam /dev/bottomcam
./udev.sh

docker exec -d auv bash -ic 'source ~/.bashrc; dvl & ping360 & ping2 & sbg & frontcam & bottomcam & modem &'

sleep 5

echo "🟢 All sensors activated!"

