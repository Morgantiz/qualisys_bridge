FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive 

ENV ROS_WS_DIR="/qualisys_ws"
ENV ROS_DISTRO="${ROS_DISTRO}"

WORKDIR ${ROS_WS_DIR}

SHELL ["/bin/bash", "-c"]

RUN echo 'source /opt/ros/humble/setup.bash h' >> ~/.bashrc \
  && echo 'if [ -e "/qualisys_ws/install/setup.bash" ]; then source /qualisys_ws/install/setup.bash; fi' >> ~/.bashrc

ENV DEBIAN_FRONTEND=dialog 
