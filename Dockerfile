FROM osrf/ros:humble-desktop-full-jammy

ARG SSH_KEY
ENV SSH_KEY=$SSH_KEY

# Make ssh dir
RUN mkdir $HOME/.ssh/
 
# Copy over private key, and set permissions
RUN  apt-get -yq update && \
     apt-get -yqq install ssh

RUN echo "$SSH_KEY" > $HOME/.ssh/id_rsa
RUN chmod 600 $HOME/.ssh/id_rsa
 
# Create known_hosts
RUN touch $HOME/.ssh/known_hosts

# Add bitbuckets key
RUN ssh-keyscan bitbucket.org >> $HOME/.ssh/known_hosts
RUN ssh-keyscan github.com >> $HOME/.ssh/known_hosts
RUN ssh-keyscan gitlab.com >> $HOME/.ssh/known_hosts

ARG CATKIN_WORKSPACE=simulator
ARG DEBIAN_FRONTEND=noninteractive
ARG HOME=/root
ARG ROS_DISTRO=noetic

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Install apt packages
RUN apt-get install -y \
    nano 

RUN apt-get update

# Create new workspace for aerial manipulator
RUN mkdir -p $HOME/simulator/src
WORKDIR $HOME/simulator/src
# tcc_docker
RUN git clone git@github.com:luccagandra/tcc-docker.git

WORKDIR $HOME/simulator/