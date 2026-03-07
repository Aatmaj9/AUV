docker run -it --privileged --rm \
  --name microros_agent \
  --net=host \
  --volume="/dev":"/dev" \
  --device=/dev/portenta \
  microros/micro-ros-agent:humble \
  serial --dev /dev/portenta -b 115200
