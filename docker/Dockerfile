FROM ros:rolling

LABEL maintainer="Clemente Donoso <clemente.donosok@gmail.com>"
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=rolling

###### Install package dependencies
RUN apt-get update
RUN apt-get install -y \
    git \
    libpcl-dev \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions \
    python-is-python3 \
    python3-pip
    
###### Clone the repository
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
RUN git clone https://github.com/CDonosoK/ros2_pcl_segmentation.git

###### Build the workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"

###### Source the workspace
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
