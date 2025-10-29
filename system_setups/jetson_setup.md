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

# The default Velodyne ip is set to 192.168.1.201

# The following nmcli command tells your Jetson’s Ethernet port to always use a fixed IP address instead of dynamically asking a router for one via DHCP.
```
nmcli device status
sudo nmcli con add type ethernet ifname eno1 con-name velodyne ipv4.addresses 192.168.3.100/24 ipv4.method manual autoconnect yes
```

In nmcli connection show you should see velodyne listed.

```
sudo nmcli con up velodyne
```
