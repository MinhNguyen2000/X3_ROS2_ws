FROM ros:humble-ros-base-jammy

# System tools
RUN apt-get update && apt-get install -y \
    iproute2 \
    iputils-ping \ 
    joystick \ 
    net-tools \
    nano \
&& rm -rf /var/lib/apt/lists/*

# ROS2 base - demo nodes + xacro + robot localization
RUN apt-get update && apt-get install -y \
          ros-${ROS_DISTRO}-demo-nodes-cpp \
          ros-${ROS_DISTRO}-demo-nodes-py \
          ros-${ROS_DISTRO}-xacro \
          ros-${ROS_DISTRO}-robot-localization \
&& rm -rf /var/lib/apt/lists/*