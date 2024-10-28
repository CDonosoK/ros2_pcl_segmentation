# ros2_pcl_segmentation

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros2 Version](https://img.shields.io/badge/ROS-Rolling-green)](
https://docs.ros.org/en/rolling/index.html)

<img src=https://github.com/CDonosoK/ros2_pcl_segmentation/blob/main/images/ground_segmentation.gif>

ROS2 Package for point cloud segmentation using PCL library. This repository contains multiple segmentation algorithms for point clouds. 

- **[pcl_car_segmentation](https://github.com/CDonosoK/ros2_pcl_segmentation/tree/main/pcl_car_segmentation)**: Package for segmenting cars from a point cloud.
- **[pcl_ground_segmentation](https://github.com/CDonosoK/ros2_pcl_segmentation/tree/main/pcl_ground_segmentation)**: Package for segmenting ground from a point cloud.
- **[pcl_human_segmentation](https://github.com/CDonosoK/ros2_pcl_segmentation/tree/main/pcl_human_segmentation)**: Package for segmenting humans from a point cloud.

## Get Started

### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/CDonosoK/ros2_pcl_segmentation.git
git submodule update --init --recursive
rosdep install -i --from-paths ros2_pcl_segmentation --rosdistro rolling -y
colcon build --packages-select ros2_pcl_segmentation
```

### Prerequisites
To test the package, you can use the kitti dataset. You can download the dataset from the following link: [KITTI Dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) or follow the instructions below:

```bash
cd mkdir ~/ros2_ws/data
cd ~/ros2_ws/data
git clone https://github.com/Deepak3994/Kitti-Dataset.git 
bash raw_data_downloader.sh
```

Then to convert the dataset to a rosbag file, you will need the following package:
```bash
cd ~/ros2_ws/src
git clone https://github.com/umtclskn/ros2_kitti_publishers.git
colcon build --packages-select ros2_kitti_publishers
```

### From Docker

You can also run the package using docker. You can use the following command to build the docker image that is inside the docker folder:
```bash
docker build -t ros2_pcl_segmentation .
```

Then you can run the docker image as follows:
```bash
sh run_docker.sh
```


## Usage

For each ROS2 Package I have created a launch file that will run the segmentation algorithm. You can run the launch file as follows:
- **pcl_car_segmentation**: 
    ```bash
    ros2 launch pcl_car_segmentation bring_kitti.launch.py
    ros2 launch pcl_car_segmentation bring_rviz.launch.py
    ```
- **pcl_ground_segmentation**: 
    ```
    ros2 launch pcl_ground_segmentation bring_kitti.launch.py
    ros2 launch pcl_ground_segmentation bring_rviz.launch.py
    ```
- **pcl_human_segmentation**:
    ```bash
    ros2 launch pcl_human_segmentation bring_kitti.launch.py
    ros2 launch pcl_human_segmentation bring_rviz.launch.py
    ```
