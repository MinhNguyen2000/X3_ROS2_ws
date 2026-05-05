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
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
  && rm -rf /var/lib/apt/lists/*

# Install RPLidar ROS2 driver
RUN apt-get update && apt-get install -y \
    ros-humble-rplidar-ros \
  && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN apt update && apt install -y python3-pip \
    && rm -rf /var/lib/apt/list/*

# Install Rosmaster_lib
RUN sudo pip3 install pyserial

COPY rosmaster_driver_install /tmp/rosmaster_driver_install
RUN cd /tmp/rosmaster_driver_install && python3 setup.py install

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user group, assign UID + group
# Create the home directory and .config dir
RUN groupadd --gid $USER_GID $USERNAME \
          && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
          && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Add sudo support for the non-root user
RUN apt-get update && apt-get install -y sudo \
          && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
          && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Entrypoint script
COPY ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/bin/bash","/ros_entrypoint.sh"]
RUN echo "source /ros_entrypoint.sh" >> ~/.bashrc

# Add user to dialout group to access the serial port
RUN sudo usermod -aG dialout $USERNAME

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

WORKDIR /X1_ROS2_ws
USER root 