source udev.sh
# source mros.sh
docker exec -d sookshma_dev bash -ic 'source ~/.bashrc; zed & velodyne &'
docker exec -it sookshma_dev bash
