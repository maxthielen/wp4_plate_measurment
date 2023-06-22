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

RUN apt-get install -y python3-pip python3-tk
# Install transforms3d
RUN pip install --upgrade --force-reinstall \
    transforms3d==0.4.1 \
    numpy-quaternion==2022.4.3

# Install sensor_msgs_py from common_interfaces
RUN apt-get install -y ros-rolling-sensor-msgs-py

# Install UR robot driver and its dependencies
RUN apt-get install -y ros-${ROS_DISTRO}-ur-robot-driver

# Add source ROS to .profile
RUN echo -e 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.profile

COPY main.py ./

## CMD ["python3", "main.py"]

CMD ["ros2", "rviz", "rviz"]
