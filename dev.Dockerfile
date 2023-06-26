ARG BASE_IMAGE=osrf/ros:foxy-desktop
FROM osrf/ros:foxy-desktop
ARG ROS_DISTRO:=foxy

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
ENV ROS_DOMAIN_ID 6
ENV NVIDIA_DRIVER_CAPABILITIES all
SHELL ["/bin/bash", "-c"] 

# Set up workspace
WORKDIR /home/ros_ws/src

# Update package repos
RUN apt-get update

# Install UR robot driver and its dependencies
RUN apt-get install -y ros-${ROS_DISTRO}-ur-robot-driver

# Install moveit servo and its dependencies
RUN apt-get install -y ros-${ROS_DISTRO}-moveit-servo

# Install ros2 control and its dependencies
RUN apt-get install -y ros-${ROS_DISTRO}-ros2-control
RUN apt-get install -y ros-${ROS_DISTRO}-ros2-controllers

RUN apt-get install -y python3-pip python3-tk

# Install Open3D system dependencies and opencv
RUN apt-get install --no-install-recommends -y \
    libgl1 \
    libgomp1 \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# # Install pip requirements
# RUN pip install --upgrade --default-timeout=100 future \
#     transforms3d==0.4.1 \
#     numpy-quaternion==2022.4.3 \
#     numpy \
#     matplotlib \
#     scikit-image \
#     scikit-learn \ 
#     open3d \
#     opencv-python \
#     scipy \ 
#     imutils \
#     python-dotenv \
#     Pillow 

# Add source ROS to .profile
RUN echo -e 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.profile
