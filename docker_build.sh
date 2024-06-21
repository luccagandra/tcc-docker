#!/bin/bash

echo "TCC - Ubuntu 20.04 Docker Image"

echo "Building ROS Noetic"

docker build \
    -f Dockerfile \
    --build-arg SSH_KEY="$(cat $HOME/.ssh/id_rsa_pessoal)" \
    -t tcc_docker .