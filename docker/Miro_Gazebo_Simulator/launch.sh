#!/usr/bin/env bash

containerName=miro_gazebo_sim
imageName=dcamilleri13/blocklylight:gazebo_only
ros_image=dcamilleri13/miro_ros:latest


if [ $1 == "install" ]
then
	echo "Installing docker image"
	docker pull $imageName
	docker pull $ros_image
elif [ $1 == "update" ]
then
	echo "Updating docker image"
	docker pull $imageName
	docker pull $ros_image
else
	echo "Running in display mode"
	docker stop $containerName
	docker run -dit --rm --env="DISPLAY="$DISPLAY --env="QT_X11_NO_MITSHM=1" --env="BLOCKLY_PORT=9000" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --name=$containerName  \
					--hostname=$HOSTNAME --network=host --privileged $imageName ./mirosim.sh $2
	xdg-open http://localhost:8080/
	docker attach $containerName
fi
