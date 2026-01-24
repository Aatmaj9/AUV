echo "🔄 Deactivating sensors..."

nohup docker exec auv_dev pkill -u mavlab -9 -f "sbg|ping|dvl|zed|ros2" >/dev/null 2>&1 &
disown

sleep 1

docker exec auv_dev bash -ic 'ros2 daemon stop' > /dev/null 2>&1

echo "🛑 All sensors deactivated!"