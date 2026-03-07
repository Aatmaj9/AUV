echo "🔄 Activating sensors..."
sudo chmod 666 /dev/ping2 /dev/ping360 /dev/sbg /dev/arduino /dev/frontcam /dev/bottomcam
./udev.sh

docker exec -d auv_dev bash -ic 'source ~/.bashrc; dvl & ping360 & ping2 & sbg & frontcam & bottomcam &'

echo "🟢 All sensors activated!"

