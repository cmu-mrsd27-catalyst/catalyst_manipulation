#!/bin/bash
CONTAINER=$(docker ps -qf "ancestor=ros2_jazzy" | head -1)

if [ -z "$CONTAINER" ]; then
  echo "No running ros2_jazzy container found. Run ./run_docker.sh first."
  exit 1
fi

docker exec -it "$CONTAINER" bash -c "source /root/ros2_ws/install/setup.bash 2>/dev/null; exec bash"
