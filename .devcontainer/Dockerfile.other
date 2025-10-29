# ==================== Base: JetPack 6.x / L4T 36.4 (Ubuntu 22.04) ====================
ARG L4T_TAG=r36.4.0
FROM nvcr.io/nvidia/l4t-jetpack:${L4T_TAG}

# ==================== Args & Env ====================
ARG ZED_SDK_MAJOR=5
ARG ZED_SDK_MINOR=0
ARG L4T_MAJOR=36
ARG L4T_MINOR=4

ENV LOGNAME=root \
    DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    ROS_WS=/opt/ros2_ws \
    NVIDIA_DRIVER_CAPABILITIES=all \
    NVIDIA_VISIBLE_DEVICES=all

# ==================== Core deps & ZED SDK ====================
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    sudo apt-utils apt-transport-https lsb-release udev usbutils wget less \
    cmake curl gnupg2 build-essential python3 python3-pip python3-dev \
    python3-setuptools libusb-1.0-0-dev zstd ca-certificates && \
    python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel protobuf && \
    \
    # Fake /etc/nv_tegra_release for compatibility with ZED SDK installer
    bash -lc "echo '# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}.0' > /etc/nv_tegra_release" && \
    wget -q --no-check-certificate -O /tmp/ZED_SDK_Linux_JP.run \
      https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons && \
    chmod +x /tmp/ZED_SDK_Linux_JP.run && \
    /tmp/ZED_SDK_Linux_JP.run silent skip_tools && \
    rm -rf /usr/local/zed/resources/* /tmp/ZED_SDK_Linux_JP.run && \
    \
    # Fix EGL/V4L inside container
    ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so && \
    rm -rf /var/lib/apt/lists/*

# ==================== ROS 2 Humble (repo + base) ====================
RUN apt-get update -y && apt-get install -y --no-install-recommends software-properties-common && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    bash -lc 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list' && \
    apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      python3-rosdep \
      python3-vcstool \
      python3-colcon-common-extensions \
      build-essential && \
    rosdep init || true && rosdep update && \
    rm -rf /var/lib/apt/lists/*

# ==================== Additional APT packages (dev, GUI libs, ROS extras, PCL, OpenCV, transports, etc.) ====================
RUN apt-get update && apt-get install -y --no-install-recommends \
      software-properties-common && apt-add-repository -y universe && apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-pip python3-serial python3-venv \
      sudo xauth htop udev zip unzip curl \
      libboost-all-dev libasio-dev \
      gcc gfortran git \
      libsm6 libxext6 libxrender1 libfontconfig1 libgl1 \
      libpcl-dev libpcap-dev libopencv-dev libyaml-cpp-dev libeigen3-dev gdebi-core \
      ros-humble-diagnostic-updater \
      ros-humble-nmea-msgs \
      ros-humble-rtcm-msgs \
      ros-humble-rviz2 \
      ros-humble-rosbridge-server \
      ros-humble-rosx-introspection \
      ros-humble-foxglove-bridge \
      ros-humble-image-transport-plugins \
      ros-humble-compressed-image-transport \
      ros-humble-compressed-depth-image-transport \
      ros-humble-theora-image-transport \
      ros-humble-point-cloud-transport-plugins \
      ros-humble-draco-point-cloud-transport \
      ros-humble-zlib-point-cloud-transport \
      ros-humble-zstd-point-cloud-transport \
      ros-humble-xacro \
      ros-humble-ffmpeg-image-transport \
      ros-humble-ffmpeg-encoder-decoder \
      ros-humble-zed-msgs \
      ros-humble-geographic-msgs \
      ros-humble-image-transport \
      ros-humble-point-cloud-transport \
      ros-humble-pcl-conversions \
      ros-humble-robot-localization \
      ros-humble-backward-ros \
      python3-opencv \
      gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi bossa-cli \
      python3-numpy python3-scipy && \
    ln -sf /usr/bin/python3 /usr/bin/python && \
    chmod -R a+rX /usr/local/lib/python3.10/dist-packages /usr/lib/python3/dist-packages && \
    rm -rf /var/lib/apt/lists/*

# ==================== Create non-root user ====================
ARG USERNAME=mavlab
ENV USERNAME=${USERNAME}
RUN groupadd -f zed && \
    useradd -m "${USERNAME}" && \
    echo "${USERNAME}:${USERNAME}" | chpasswd && \
    usermod --shell /bin/bash "${USERNAME}" && \
    usermod -aG sudo,dialout,video,plugdev,zed "${USERNAME}" && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# ==================== Switch to non-root ====================
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# ==================== Python venv + requirements ====================
RUN python3 -m venv --system-site-packages /home/${USERNAME}/.venv
ENV VIRTUAL_ENV=/home/${USERNAME}/.venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
COPY --chown=${USERNAME}:${USERNAME} requirements.txt /home/${USERNAME}/requirements.txt
RUN python -m pip install --upgrade pip setuptools wheel && \
    python -m pip install --no-cache-dir -r /home/${USERNAME}/requirements.txt && \
    echo 'source $VIRTUAL_ENV/bin/activate' >> /home/${USERNAME}/.bashrc

# ==================== Arduino CLI + micro-ROS setup ====================
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
ENV PATH="/home/${USERNAME}/bin:${PATH}"
ENV ARDUINO_DATA="/home/${USERNAME}/.arduino15"
RUN arduino-cli config init && \
    arduino-cli config set directories.data "${ARDUINO_DATA}" && \
    arduino-cli core update-index && \
    arduino-cli cache clean

RUN arduino-cli core update-index && \
    arduino-cli cache clean && \
    arduino-cli core update-index --additional-urls https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json && \
    export ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS=https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json && \
    arduino-cli core install per1234:sam && \
    SAM_ARM_PATH="/home/${USERNAME}/.arduino15/packages/per1234/hardware/sam/1.6.11-arm64" && \
    curl -fsSL https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/main/extras/patching_boards/platform_arduinocore_sam.txt -o ${SAM_ARM_PATH}/platform.txt && \
    sed -i 's/-4.8.3-2014q1././g' ${SAM_ARM_PATH}/platform.txt && \
    arduino-cli lib install Servo && \
    git clone --depth=1 https://gitlab.com/timwilkinson/FlySkyIBus.git /home/${USERNAME}/Arduino/libraries/FlySkyIBus

COPY --chown=${USERNAME}:${USERNAME} micro_ros_arduino_arm.zip /tmp/micro_ros_arduino.zip
RUN printf "board_manager:\n  additional_urls: []\n\nlibrary:\n  enable_unsafe_install: true\n" > /home/${USERNAME}/.arduino15/arduino-cli.yaml && \
    arduino-cli --config-file /home/${USERNAME}/.arduino15/arduino-cli.yaml lib install --zip-path /tmp/micro_ros_arduino.zip && \
    rm /tmp/micro_ros_arduino.zip

# ==================== Node.js via NVM + localtunnel ====================
ENV NVM_DIR="/home/${USERNAME}/.nvm"
ENV NODE_VERSION="20.17.0"
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash && \
    . "$NVM_DIR/nvm.sh" && \
    nvm install $NODE_VERSION && \
    nvm alias default $NODE_VERSION && \
    nvm use default && \
    npm install -g npm@latest localtunnel
ENV PATH="${NVM_DIR}/versions/node/v${NODE_VERSION}/bin:${PATH}"

# ==================== ROS 2 bashrc + aliases ====================
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc && \
    echo 'if [ -f /workspaces/mavlab/ros2_ws/install/setup.bash ]; then source /workspaces/mavlab/ros2_ws/install/setup.bash; fi' >> /home/${USERNAME}/.bashrc && \
    echo "alias sros2='source /opt/ros/humble/setup.bash && if [ -f /workspaces/mavlab/ros2_ws/install/setup.bash ]; then source /workspaces/mavlab/ros2_ws/install/setup.bash; fi'" >> /home/${USERNAME}/.bashrc && \
    echo "alias sbg='sros2 && ros2 launch sbg_driver sbg_device_launch.py'" >> /home/${USERNAME}/.bashrc && \
    echo "alias uwb='sros2 && ros2 run uwb_driver uwb'" >> /home/${USERNAME}/.bashrc && \
    echo "alias gnc='sros2 && ros2 run gnc gnc'" >> /home/${USERNAME}/.bashrc && \
    echo "alias mavsim='sros2 && ros2 run mav_simulator simulate'" >> /home/${USERNAME}/.bashrc && \
    echo "alias ardusimple='sros2 && ros2 launch ublox_gps ublox_gps_node-launch.py'" >> /home/${USERNAME}/.bashrc

# ==================== Ports & final workdir ====================
EXPOSE 9000 9001 9002 9003
WORKDIR /home/${USERNAME}
