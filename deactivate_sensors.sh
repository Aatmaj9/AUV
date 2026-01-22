#!/usr/bin/env bash
docker exec auv_dev bash -c "
pkill -f ping2
pkill -f ping360
pkill -f dvl
pkill -f sbg
pkill -f zed
"
echo "Sensors stopped"
