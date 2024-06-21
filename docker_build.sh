#!/bin/bash

echo "TCC - Ubuntu 20.04 Docker Image"

echo "Building ROS Noetic"

docker build \
    -f Dockerfile \
    -t tcc_docker .