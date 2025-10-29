# docker kill microros_agent
docker exec -d sookshma_dev bash -lc '
  pkill -f "(ros2 launch velodyne|velodyne_driver_node|velodyne_transform_node|velodyne_laserscan_node)" || true
  pkill -f "(ros2 launch zed|zed_camera.launch.py|zed_wrapper)" || true
'
