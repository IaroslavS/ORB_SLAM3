#!/bin/bash
xhost +
xhost +local:docker
docker exec -it --user "docker_orb_slam3" orb_slam3 \
    /bin/bash -c "cd /home/docker_orb_slam3/orb_slam3; /bin/bash"