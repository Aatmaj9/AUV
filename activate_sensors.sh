echo "🔄 Activating sensors..."

docker exec -d auv_dev bash -ic 'source ~/.bashrc; dvl & ping360 &'

sleep 2

docker exec -d auv_dev bash -ic 'source ~/.bashrc; ping2 & zed &'

sleep 4

docker exec -d auv_dev bash -ic 'source ~/.bashrc; sbg &'

sleep 6

echo "🟢 All sensors activated!"

