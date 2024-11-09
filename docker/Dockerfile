FROM ros:rolling

LABEL maintainer="Clemente Donoso <clemente.donosok@gmail.com>"
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=rolling

###### Install package dependencies
RUN apt-get install -y \
    git \
    python-is-python3 \
    python3-pip
    
###### Clone the repository
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
RUN git clone https://github.com/CDonosoK/ros2_pcl_segmentation.git

###### BUILD
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
WORKDIR /ros2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.bash && colcon build

###### Source the workspace
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
