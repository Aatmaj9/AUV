#!/usr/bin/env bash
set -m  
trap "kill 0" SIGTERM SIGINT

dvl &
ping360 &
ping2 &
sbg &
zed &

wait