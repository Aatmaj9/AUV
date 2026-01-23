#!/usr/bin/env bash

docker exec auv_dev bash -c '
pgrep -f "ros2 run ping360_sonar" | xargs -r kill -9
pgrep -f "ros2 run ping_sonar_ros" | xargs -r kill -9
pgrep -f "ros2 launch sbg_driver" | xargs -r kill -9
pgrep -f "ros2 launch zed_wrapper" | xargs -r kill -9
pgrep -f "ros2 launch dvl_a50" | xargs -r kill -9
'

echo "Sensors stopped"
