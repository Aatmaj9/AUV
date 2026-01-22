sudo cp 99-usb-serial.rules /etc/udev/rules.d/99-usb-serial.rules
sudo udevadm control --reload-rules
sudo udevadm trigger