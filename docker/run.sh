# For graphics

isRunning=`docker ps -f name=ros2_pcl_segmentation | grep -c "ros2_pcl_segmentation"`;

if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm ros2_pcl_segmentation
    docker run \
        --name ros2_pcl_segmentation \
        -it \
        --env="DISPLAY" \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        --gpus all \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume `pwd`/../:/ros2_ws/src/ros2_pcl_segmentation \
        --net host \
        --ipc host \
        --pid host \
        --privileged \
        -w /ros2_ws \
        ros2_pcl_segmentation:latest

else
    echo "ros2_pcl_segmentation is already running"
    docker exec -it ros2_pcl_segmentation /bin/bash
fi

