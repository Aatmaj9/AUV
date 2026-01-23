source udev.sh
docker exec -d auv_dev bash -ic 'shopt -s expand_aliases; source ~/.bashrc; dvl & ping360 & sleep 4; ping2 & sleep 4; sbg & sleep 2; zed &'

echo "Sensors started"
docker exec -it auv_dev bash
