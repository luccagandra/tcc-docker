FROM lmark1/uav_ros_simulation:focal-bin-0.0.1 as uav_base

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
RUN git clone https://github.com/luccagandra/tcc-docker.git
# ardupilot_models
RUN git clone https://github.com/larics/ardupilot_gazebo.git
# mav_comm
RUN git clone https://github.com/larics/mav_comm.git -b larics_master
# rotors simulator
RUN git clone https://github.com/larics/rotors_simulator.git -b larics_noetic_master
WORKDIR $HOME/simulator/
RUN bash -c "source /opt/ros/noetic/setup.bash; source ~/.bashrc;  catkin build" 

# Add localhost for it 
RUN echo "export ROS_MASTER_URI=http://127.0.0.1:11311" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=127.0.0.1" >> ~/.bashrc
RUN echo "source /root/simulator/devel/setup.bash" >> ~/.bashrc 

CMD ["bash"]