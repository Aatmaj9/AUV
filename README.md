# SOOKSHMA JETSON


## About the files

The build.sh script is to be used only for primary building the image on the system. The build.sh script will build multiplatform image - for amd64 and arm64. On all the RPIs use pull.sh  to pull the respective arm64 image.

Use the start.sh to start the container and the enter.sh to enter the container in another terminal.

libgen_arm.sh used only on the main system to generate the custom message compatible micro_ros_arduino library for ARM platforms. The library has been already built and provided as zip file which has already been installed in the dockerfile.

libgen_amd.sh used only on the main system to generate the custom message compatible micro_ros_arduino library for AMD platforms. The library has been already built and provided as zip file which has already been installed in the dockerfile.

The compile and upload scripts from inside the container to arduino from arm and amd systems respectively.

Do pull.sh to pull the latest image.

## Requirements for building the multi-platform image on the system

```
docker buildx create --name multiarch --driver docker-container --use 
docker run --privileged --rm tonistiigi/binfmt --install all 
docker buildx inspect --bootstrap

```
## Requirements for libgen on the main system

Arduino CLI

```
cd ~
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

echo 'export PATH=$PATH:$HOME/bin' >> ~/.bashrc
source ~/.bashrc

arduino-cli core update-index
arduino-cli cache clean

```
Microros Arduino Library

```
mkdir -p ~/Arduino/libraries
cd ~/Arduino/libraries
git clone -b humble https://github.com/micro-ROS/micro_ros_arduino.git

```
Now run the libgen.sh script to generate the custom library.

### First lets do library generation for AMD

Install on the system -
```
arduino-cli core update-index
arduino-cli core install arduino:sam@1.6.12

```

Then run the libgen_amd.sh script.

Inside ~/Arduino/libraries/micro_ros_arduino run this -

```
strings src/cortex-m3/libmicroros.a | grep "GCC: ("
```
Check the compiler - for amd it should should show version 4.8.3 

Zip the new newly generated micro_ros_arduino library as micro_ros_srduino_amd.zip inside this repo. Delete the library and clone it again for generating the library for arm.

### Now lets do library generation for ARM

Install on the system -

```
arduino-cli core update-index --additional-urls https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json
export ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS=https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json
arduino-cli core install per1234:sam

```
Then run the libgen_arm.sh script.

```
strings src/cortex-m3/libmicroros.a | grep "GCC: ("
```
Check the compiler - for arm it should should show version 7-2017-q4-major 

Zip the new newly generated micro_ros_arduino library as micro_ros_srduino_arm.zip inside this repo.


