docker exec -d auv_dev bash -ic 'source ~/.bashrc; dvl & ping360 &'

sleep 4

docker exec -d auv_dev bash -ic 'source ~/.bashrc; ping2 &'

sleep 4

docker exec -d auv_dev bash -ic 'source ~/.bashrc; sbg & zed &'

sleep 8

echo "🟢 All sensors activated!"

