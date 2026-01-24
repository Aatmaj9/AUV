# Start DVL + Ping360 first
docker exec -d auv_dev bash -ic 'source ~/.bashrc; dvl & ping360 &'

sleep 4

# Start Ping2
docker exec -d auv_dev bash -ic 'source ~/.bashrc; ping2 &'

sleep 4

# Start rest
docker exec -d auv_dev bash -ic 'source ~/.bashrc; sbg & zed &'

echo "🟢 All sensors activated!"

