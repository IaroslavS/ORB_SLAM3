#!/bin/bash

ARCH="$(uname -m)"

main () {
    pushd "$(dirname "$0")";
    if [ "$ARCH" = "x86_64" ]; then 
        file="./Dockerfile.x86_64"
    elif [ "$ARCH" = "aarch64" ]; then
        file="./Dockerfile.aarch64"
    else
        echo "There is no Dockerfile for ${ARCH} arch"
    fi
    echo "Building image for ${ARCH} arch. from Dockerfile: ${file}"
    docker build . --progress=plain \
        -f ${file} \
        --network host \
        --build-arg NUM_THREADS=`expr $(nproc) - 1` \
        -t ${ARCH}orb_slam3:latest;
    popd;
}

main "$@"; exit;