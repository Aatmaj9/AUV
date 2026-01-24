# Jetson is flashed with Jetpack 6.2 

# auto-connect to wifi and set hostname
```
nmcli device wifi list
nmcli device wifi connect mavlab password mavlab24
nmcli connection modify mavlab connection.autoconnect yes
sudo hostnamectl set-hostname masv01
```
----------------------------------------------Git installation-------------------------------------
```
sudo apt-get update
sudo apt-get install git -y
```
---------------------------------------------Docker installation ----------------------------------
# add user to docker group
```
sudo usermod -aG docker $USER
```

----------------------------------------------- Wifi Adapter driver for Jetson -----------------------------------------
# USB Wi-Fi adapter Setup for Jetson
```
sudo apt install dkms git build-essential
git clone https://github.com/morrownr/88x2bu-20210702.git
cd 88x2bu-20210702
sudo ./install-driver.sh
sudo reboot
```

# Now check if both the interfaces appear`
```
iwdev
```
# You should see both interface now - If the Wifi adapter doesnt have a IP address run this:
```
sudo nmcli dev wifi connect "mavlab" password "mavlab24" ifname wlan1
```

# To view signal strength and link quality - Lower the signal strength in dBm the better it is
```
iwconfig wlP1p1s0
iwconfig wlx8c902d14c25e
```
----------------------------------------------- Setting static ip to ethernet switch -----------------------------------------

# Set static ip for the ethernet switch on enP8p1s0 (stays even after reboot)
```
sudo nmcli con add type ethernet con-name eth_switch ifname enP8p1s0 ip4 192.168.194.10/24
sudo nmcli con mod eth_switch connection.autoconnect yes
sudo nmcli con up eth_switch

```
