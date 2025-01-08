FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

LABEL maintainer="Clemente Donoso <clemente.donosok@gmail.com>"
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

###### Install ROS2
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y
RUN apt install ros-${ROS_DISTRO}-desktop-full -y
RUN apt install ros-dev-tools -y
RUN rosdep init && rosdep update

###### Install python packages
RUN apt-get install -y \
    python-is-python3 \
    python3-pip

RUN pip install opencv-python
RUN pip install --upgrade numpy==1.24.3
RUN pip install matplotlib==3.6.0
RUN pip install \
    pandas \
    ultralytics
    
    
RUN pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
    

###### Install package dependencies
RUN apt-get install -y \
    git \
    libpcl-dev \
    terminator \
    v4l-utils \
    ntp

###### Install ros dependencies
RUN apt-get install -y \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rviz2

###### Build the workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

###### Source the workspace
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
