docker run -d --privileged --rm \
  --name microros_agent \
  --net=host \
  --volume="/dev":"/dev" \
  --device=/dev/ttyACM0 \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyACM0 -b 115200