#!/bin/sh

#version_gt() { test "$(echo "$@" | tr " " "\n" | sort -V | tail -n 1)" = "$1"; }
#docker_version=$(docker version | grep 'Client version' | awk '{split($0,a,":"); print a[2]}' | tr -d ' ')
# Docker 1.3.0 or later is required for --device

#if test $# -lt 1; then
	# Get the latest opengl-nvidia build
	# and start with an interactive terminal enabled
#	args="-i -t $(docker images | grep ^opengl-nvidia | head -n 1 | awk '{ print $1":"$2 }')"
#else
        # Use this script with derived images, and pass your 'docker run' args
#	args="$@"
#fi



echo "preparation"

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
echo "XSOCK=${XSOCK}"
echo "XAUTH=${XAUTH}"
echo "xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -"
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

echo "Run"

echo docker run \
	--runtime=nvidia \
	-v $XSOCK:$XSOCK:rw \
	-v $XAUTH:$XAUTH:rw \
	--device=/dev/nvidia0:/dev/nvidia0 \
	--device=/dev/nvidiactl:/dev/nvidiactl \
	--device=/dev/nvidia-uvm:/dev/nvidia-uvm \
	-e DISPLAY=$DISPLAY \
	-e XAUTHORITY=$XAUTH \
	-it \
	bbodin/slambench:cuda-ubuntu-16.04 $@

docker run \
	--runtime=nvidia \
	-v $XSOCK:$XSOCK:rw \
	-v $XAUTH:$XAUTH:rw \
	--device=/dev/nvidia0:/dev/nvidia0 \
	--device=/dev/nvidiactl:/dev/nvidiactl \
	--device=/dev/nvidia-uvm:/dev/nvidia-uvm \
	-e DISPLAY=$DISPLAY \
	-e XAUTHORITY=$XAUTH \
	-it \
	bbodin/slambench:cuda-ubuntu-16.04 $@
