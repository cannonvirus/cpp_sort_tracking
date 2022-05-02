## Introduction
C++ implementation of SORT: Simple, online, and real-time tracking of multiple objects in a video sequence.

Kuhn-Munkres (Hungarian) Algorithm in C++ is forked from:
https://github.com/saebyn/munkres-cpp

## Dependencies
- Ubuntu 18.04
- OpenCV 4.2
- Boost 1.58.0 or up
- 3090 RTX GPU

## Build Docker Image
- --:jp45_nx_boost176_cmake317_NumCpp

## Docker run
```bash
#!/bin/bash
container_name="cannonvirus_test_docker"
docker_image="--:jp45_nx_boost176_cmake317_NumCpp"

xhost +

docker run \
--name=${container_name} \
--net=host \
--privileged \
--ipc=host \
--runtime nvidia \
-w /works \
-v /home/intflow/works:/works \
-v /home/intflow/.Xauthority:/root/.Xauthority:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-e DISPLAY=$DISPLAY \
--rm -it \
${docker_image} bash
```

## install library
```bash
apt-get install libboost-all-dev -y 
apt-get install libeigen3-dev
apt-get install build-essential gdb
```

## Python info 
- -b cow_tr3_r18_neck_Rsort https://github.com/intflow/nvidia-odtk-workspace.git

1. Nas server 안에 있는 pig data 사용
    ```bash
    /DL_data_big/EdgeFarm_pig/SL/piglet/img_per_10frame/1
    ```

2. Create a symbolic link to the dataset
    ```bash
    $ ln -s /DL_data_big/EdgeFarm_pig/SL/piglet/img_per_10frame/1 /path/to/rotated-sort-cpp/data/1/
    ```
3. Run the demo
    ```bash
    $ cd /path/to/sort-cpp
    $ mkdir build && cd "$_"
    $ cmake .. && make
    $ cd /path/to/sort-cpp/bin
    # Without display
    $ ./sort-cpp
    ```

## References
1. https://github.com/abewley/sort
2. https://github.com/mcximing/sort-cpp
3. https://github.com/saebyn/munkres-cpp
