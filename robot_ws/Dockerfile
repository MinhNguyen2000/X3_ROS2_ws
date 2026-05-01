FROM ros:humble-ros-base-jammy

# System tools
RUN apt-get update && apt-get install -y \
    iproute2 \
    iputils-ping \ 
    joystick \ 
    net-tools \
    nano \
&& rm -rf /var/lib/apt/lists/*