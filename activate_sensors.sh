source udev.sh
#Please note the order of sensors - ping360 then ping2 then sbg then zed
docker exec -d auv_dev bash -ic 'source ~/.bashrc; dvl & ping360 & ping2 & sbg & zed &'
docker exec -it auv_dev bash
echo "Sensors started"