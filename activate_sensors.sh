source udev.sh
docker exec -d auv_dev bash -ic 'source ~/.bashrc; dvl & ( ping360 & sleep 6 ) ; ( ping2 & sleep 4 ) ; sbg & zed &'

echo "Sensors started"
docker exec -it auv_dev bash
