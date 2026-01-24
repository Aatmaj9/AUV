# Start DVL + Ping360 first
docker exec -d auv_dev bash -ic 'source ~/.bashrc; dvl & ping360 &'

sleep 8

# Start Ping2
docker exec -d auv_dev bash -ic 'source ~/.bashrc; ping2 &'

# Start rest
docker exec -d auv_dev bash -ic 'source ~/.bashrc; sbg & zed &'

echo "Sensors started"
