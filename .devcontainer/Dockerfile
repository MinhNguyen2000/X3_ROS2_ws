FROM osrf/ros:humble-desktop

# System tools
RUN apt-get update && apt-get install -y \
    iproute2 \
    iputils-ping \ 
    joystick \ 
    net-tools \
    nano \
  && rm -rf /var/lib/apt/lists/*

# ROS2 base - diagnostic tools
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-plotjuggler-ros \
  && rm -rf /var/lib/apt/lists/*

# ROS2 Gazebo - simulation testing
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
  && rm -rf /var/lib/apt/lists/*

# ROS2 camera - packages to visualize the camera stream
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-image-publisher \
  && rm -rf /var/lib/apt/lists/*

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Entrypoint script
COPY ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/bin/bash","/ros_entrypoint.sh"]
RUN echo "source /ros_entrypoint.sh" >> /home/$USERNAME/.bashrc

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Install python 3 and pip
RUN sudo apt-get install -y python3 python3-pip && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Pip install constraints
COPY constraints.txt /tmp/constraints.txt

# Install necessary pip packages
COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir --upgrade pip
RUN pip install --no-cache-dir --constraint /tmp/constraints.txt --upgrade "setuptools>=30.3.0,<80" "packaging>=23.2,<25"
RUN pip install --no-cache-dir --constraint /tmp/constraints.txt -r /tmp/requirements.txt
RUN pip install --no-cache-dir torch==2.9.0+cpu --index-url https://download.pytorch.org/whl/cpu

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################
