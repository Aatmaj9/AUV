#!/bin/bash
docker compose -f .devcontainer/docker-compose.yml up -d
docker exec -it auv_dev bash
