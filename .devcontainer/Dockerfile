# Base: ZED SDK 5.0 for JetPack 6.2.1 (L4T r36.4)
FROM stereolabs/zed:5.0-tools-devel-l4t-r36.4

# -------------------- ROS 2 Humble + Base Deps --------------------
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      python3-rosdep \
      python3-vcstool \
      python3-colcon-common-extensions \
      build-essential \
 && rosdep init || true \
 && rosdep update \
 && rm -rf /var/lib/apt/lists/*

# -------------------- Additional Apt Packages --------------------
RUN apt-get update && apt-get install -y software-properties-common \
 && apt-add-repository -y universe && apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-pip python3-serial python3-venv \
      sudo xauth htop udev zip unzip curl \
      libboost-all-dev libasio-dev \
      build-essential gcc gfortran git \
      libsm6 libxext6 libxrender1 libfontconfig1 libgl1 libpcl-dev libpcap-dev libopencv-dev libyaml-cpp-dev libeigen3-dev gdebi-core \
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
      ros-humble-velodyne \
      ros-humble-velodyne-driver \
      ros-humble-velodyne-pointcloud \
      ros-humble-tf2-ros \
      ros-humble-tf2-geometry-msgs \
      ros-humble-message-filters \
      ros-humble-camera-info-manager \
      ros-humble-pointcloud-to-laserscan \
      ros-humble-laser-filters \
      python3-opencv \
      gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi bossa-cli \
      python3-numpy python3-scipy \
 && ln -sf /usr/bin/python3 /usr/bin/python \
 && chmod -R a+rX /usr/local/lib/python3.10/dist-packages /usr/lib/python3/dist-packages \
 && rm -rf /var/lib/apt/lists/*

# -------------------- Create non-root user --------------------
ARG USERNAME=mavlab
ENV USERNAME=${USERNAME}
RUN useradd -m "${USERNAME}" \
 && echo "${USERNAME}:${USERNAME}" | chpasswd \
 && usermod --shell /bin/bash "${USERNAME}" \
 && usermod -aG sudo,dialout,video,plugdev,zed "${USERNAME}" \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# -------------------- Python venv + pip packages --------------------
USER ${USERNAME}
WORKDIR /home/${USERNAME}
RUN python3 -m venv --system-site-packages /home/${USERNAME}/.venv
ENV VIRTUAL_ENV=/home/${USERNAME}/.venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
COPY --chown=${USERNAME}:${USERNAME} requirements.txt /home/${USERNAME}/requirements.txt
RUN python3 -m pip install --upgrade pip setuptools wheel \
 && python3 -m pip install --no-cache-dir -r /home/${USERNAME}/requirements.txt
RUN echo 'source $VIRTUAL_ENV/bin/activate' >> ~/.bashrc

# -------------------- Arduino CLI + micro-ROS setup --------------------
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
ENV PATH="/home/${USERNAME}/bin:${PATH}"
ENV ARDUINO_DATA="/home/${USERNAME}/.arduino15"
RUN arduino-cli config init \
 && arduino-cli config set directories.data "${ARDUINO_DATA}" \
 && arduino-cli core update-index \
 && arduino-cli cache clean

RUN arduino-cli core update-index \
 && arduino-cli cache clean \
 && arduino-cli core update-index --additional-urls https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json \
 && export ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS=https://per1234.github.io/ArduinoCore-sam/package_per1234_samarm64_index.json \
 && arduino-cli core install per1234:sam \
 && SAM_ARM_PATH="/home/${USERNAME}/.arduino15/packages/per1234/hardware/sam/1.6.11-arm64" \
 && curl -fsSL https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/main/extras/patching_boards/platform_arduinocore_sam.txt -o ${SAM_ARM_PATH}/platform.txt \
 && sed -i 's/-4.8.3-2014q1././g' ${SAM_ARM_PATH}/platform.txt \
 && arduino-cli lib install Servo \
 && git clone --depth=1 https://gitlab.com/timwilkinson/FlySkyIBus.git /home/${USERNAME}/Arduino/libraries/FlySkyIBus

COPY --chown=${USERNAME}:${USERNAME} micro_ros_arduino_arm.zip /tmp/micro_ros_arduino.zip
RUN printf "board_manager:\n  additional_urls: []\n\nlibrary:\n  enable_unsafe_install: true\n" > /home/${USERNAME}/.arduino15/arduino-cli.yaml \
 && arduino-cli --config-file /home/${USERNAME}/.arduino15/arduino-cli.yaml lib install --zip-path /tmp/micro_ros_arduino.zip \
 && rm /tmp/micro_ros_arduino.zip

# -------------------- Node.js via NVM + localtunnel --------------------
ENV NVM_DIR="/home/${USERNAME}/.nvm"
ENV NODE_VERSION="20.17.0"
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash \
 && . "$NVM_DIR/nvm.sh" \
 && nvm install $NODE_VERSION \
 && nvm alias default $NODE_VERSION \
 && nvm use default \
 && npm install -g npm@latest localtunnel
ENV PATH="${NVM_DIR}/versions/node/v${NODE_VERSION}/bin:${PATH}"

# -------------------- ROS 2 bashrc + aliases --------------------
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc \
 && echo 'if [ -f /workspaces/mavlab/ros2_ws/install/setup.bash ]; then source /workspaces/mavlab/ros2_ws/install/setup.bash; fi' >> /home/${USERNAME}/.bashrc \
 && echo "alias sros2='source /opt/ros/humble/setup.bash && if [ -f /workspaces/mavlab/ros2_ws/install/setup.bash ]; then source /workspaces/mavlab/ros2_ws/install/setup.bash; fi'" >> /home/${USERNAME}/.bashrc \
 && echo "alias colb='pushd ~/ros2_ws >/dev/null && sudo rm -rf build log install && colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers \$(nproc) && sros2 && popd >/dev/null'" >> /home/${USERNAME}/.bashrc \
 && echo "alias topics='ros2 topic list'" >> /home/${USERNAME}/.bashrc \
 && echo "alias nodes='ros2 node list'" >> /home/${USERNAME}/.bashrc

RUN echo "alias sbg='sros2 && ros2 launch sbg_driver sbg_device_launch.py'" >> /home/${USERNAME}/.bashrc \
 && echo "alias gnc='sros2 && ros2 run gnc gnc'" >> /home/${USERNAME}/.bashrc \
 && echo "alias uwb='sros2 && ros2 run uwb_driver uwb'" >> /home/${USERNAME}/.bashrc \
 && echo "alias mavsim='sros2 && ros2 run mav_simulator simulate'" >> /home/${USERNAME}/.bashrc \
 && echo "alias ardusimple='sros2 && ros2 launch ublox_gps ublox_gps_node-launch.py'" >> /home/${USERNAME}/.bashrc \
 && echo "alias zed='sros2 && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i'" >> /home/${USERNAME}/.bashrc \
 && echo "alias velodyne='sros2 && ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py'" >> /home/${USERNAME}/.bashrc

EXPOSE 9000 9001 9002 9003
