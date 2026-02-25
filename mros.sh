docker run -d --privileged --rm \
  --name microros_agent \
  --net=host \
  --volume="/dev":"/dev" \
  --device=/dev/arduino \
  microros/micro-ros-agent:humble \
  serial --dev /dev/arduino -b 115200