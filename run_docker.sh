#!/bin/bash
container_name="cannonvirus_test_docker"
docker_image="intflow/edgefarm:jp45_nx_boost176_cmake317_NumCpp"

#sudo mkdir /DL_data_big
#sudo mount 192.168.0.18:/DL_data_big /DL_data_big

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
--mount type=bind,src=/DL_data_big,dst=/DL_data_big \
--rm -it \
${docker_image} bash
