# Jetson is flashed with Jetpack 6.2 

# auto-connect to wifi and set hostname
```
nmcli device wifi list
nmcli device wifi connect mavlab password mavlab24
nmcli connection modify mavlab connection.autoconnect yes
sudo hostnamectl set-hostname timi
```
------------------------------------------SSH Installation-----------------------------------

```
sudo apt update
sudo apt install -y openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
```

----------------------------------------------Git installation-------------------------------------
```
sudo apt-get update
sudo apt-get install git -y
```
---------------------------------------------Docker installation ----------------------------------
# add user to docker group
```
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y
sudo systemctl enable docker
sudo systemctl start docker
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
iw dev
```
# You should see both interface now - If the Wifi adapter doesnt have a IP address run this:
```
sudo nmcli dev wifi connect "mavlab" password "mavlab24" ifname wlx8c902d14c273
```

# To view signal strength and link quality - Lower the signal strength in dBm the better it is
```
iwconfig wlP1p1s0
iwconfig wlx8c902d14c273
```
----------------------------------------------- Setting static ip to ethernet switch -----------------------------------------

# Set static ip for the ethernet switch on enP8p1s0 (stays even after reboot)
```
sudo nmcli con add type ethernet con-name eth_switch ifname enP8p1s0 ip4 192.168.194.10/24
sudo nmcli con mod eth_switch connection.autoconnect yes
sudo nmcli con up eth_switch

```
----------------------------------------------- For running SonarView App image and permission for ports -----------------------------------------
```
sudo apt update
sudo apt install libfuse2
sudo usermod -aG dialout $USER
```

-------------------------------------------- For DVL--------------------------------------

```
ssh -L 8080:192.168.194.95:80 timi@192.168.1.162
```

Tuning for large messages for Cyclone DDS
All DDS implementations are not designed to handle large messages (such as images or point clouds). Therefore, it is necessary to tune them and the network parameters to prevent data loss and system overloading.

For making it permanent -

sudo nano /etc/sysctl.d/60-auv-ros2-buffers.conf

net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
net.core.rmem_max=2147483647

sudo sysctl -p /etc/sysctl.d/60-auv-ros2-buffers.conf

For temporary - 

sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
sudo sysctl -w net.core.rmem_max=2147483647