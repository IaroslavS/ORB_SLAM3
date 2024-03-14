#!/bin/bash
xhost +local:docker

dataset_path=$1

cd "$(dirname "$0")"
cd ..
workspace_dir=$PWD
# Assuming that workspace_dir is a folder of the repository

desktop_start() {
    docker run -it -d --rm \
        --gpus all \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --privileged \
        --name orb_slam3 \
        --net "host" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $workspace_dir/:/home/docker_orb_slam3/orb_slam3:rw \
        -v ${dataset_path}:/home/datasets:rw \
        --cap-add=SYS_PTRACE \
        --security-opt seccomp=unconfined \
        ${ARCH}orb_slam3:latest
}

main () {
    if [ "$(docker ps -aq -f status=exited -f name=orb_slam3)" ]; then
        docker rm orb_slam3;
    fi

    ARCH="$(uname -m)"

    xhost +local:
    desktop_start;
}

main;
