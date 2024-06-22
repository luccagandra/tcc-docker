#!/bin/bash

echo "TCC - Ubuntu 22.04 Docker Image"

echo "Building ROS 2 Jammy"

docker build \
    -f Dockerfile \
    --build-arg SSH_KEY="$(cat $HOME/.ssh/id_rsa_pessoal)" \
    -t tcc_docker .