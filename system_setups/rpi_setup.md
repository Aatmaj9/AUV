# We have flashed RPi with Ubuntu 24.04.3 LTS - Server

--------------------------------------------- Git installation--------------------------------
```
sudo apt-get update
sudo apt-get install git -y
```

---------------------------------------------- Docker Installation ----------------------------------------

```
sudo apt update
sudo apt install ca-certificates curl gnupg lsb-release -y
curl -fsSL https://get.docker.com | sudo bash
sudo usermod -aG docker $USER
newgrp docker
```

----------------------------------------------- Wifi Adapter driver for RPi -----------------------------------------
```
sudo apt install dkms git build-essential
git clone https://github.com/morrownr/88x2bu-20210702.git
cd 88x2bu-20210702

sudo apt update
sudo apt install "linux-headers-$(uname -r)"
sudo apt install linux-headers-raspi

sudo apt install dkms git build-essential

cd 88x2bu-20210702
sudo ./install-driver.sh
sudo reboot
```

```
ip link show
```

# you will see something like this - wlxdc6279486c49
```
sudo ip link set wlxdc6279486c49 up
```
# Now it will also be visible in ifconfig
```
ifconfig
```
# Now you need to make the wifi adapter connect to the mavlab network
```
sudo nano /etc/netplan/50-cloud-init.yaml
```

# Edit it to make it look like this
```
network:
  version: 2
  wifis:
    wlan0:
      optional: true
      dhcp4: true
      access-points:
        "mavlab":
          auth:
            key-management: "psk"
            password: "24696f206c0110710c56de6212e4557c2f1826529da8f0bf8992f695fde981e8"
    wlxdc6279486c49:
      dhcp4: true
      optional: true
      access-points:
        "mavlab":
          password: "mavlab24"
```
sudo netplan apply