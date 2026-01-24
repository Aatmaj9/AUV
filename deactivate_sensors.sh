#!/usr/bin/env bash

docker exec auv_dev bash -c '
pkill -9 -f sbg_device
pkill -9 -f ping360
pkill -9 -f ping_sonar_ros
pkill -9 -f dvl
pkill -9 -f zed
pkill -9 -f zed_wrapper
pkill -9 -f zed_node
'

echo "🔴 All sensors deactivated"
