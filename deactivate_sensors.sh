#!/usr/bin/env bash

docker exec auv_dev bash -c '
pkill -9 -f "ros2 run ping360_sonar"
pkill -9 -f "ros2 run ping_sonar_ros"
pkill -9 -f "ros2 launch sbg_driver"
pkill -9 -f "ros2 launch zed_wrapper"
pkill -9 -f "ros2 launch dvl_a50"
'

echo "Sensors stopped"
