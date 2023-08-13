#!/bin/bash

# Check if the correct number of arguments is provided
if [ $# -lt 3 ]; then
    echo "Usage: $0 [cli|gui] <docker_image> <file_name> [<volume1> ... <volumeN>]"
    exit 1
fi

# Parse command-line arguments
mode="$1"
image="$2"
file="$3"
paths=("${@:4}")

# Start the Docker container
container_name="${image}-container"
deps_dir="/deps"
docker_run_command="--mount source=$file,destination=/slambench/datasets"
end=" "

for path in "${paths[@]}"; do
    name=$(basename "$(dirname "$path")")
    volumes+=("$name-vol")
    end+="$path "
done 
echo "$end"
echo "$volumes"
# Mount volumes in the Docker container
for volume in "${volumes[@]}"; do
    # remove vol suffix
    volume_name="${volume%-vol}"
    docker_run_command+=" -it --mount source=$volume,destination=/deps/$volume_name"
done


case $mode in
    --cli)
        
        docker_run_command="docker run $docker_run_command $image --build-cli $end"
        ;;
    --gui)
        docker_run_command=" docker run -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg     -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY=$DISPLAY     --device /dev/dri/card0 --device /dev/dri/renderD128     -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR $docker_run_command $image --build-gui $end"
        ;;
    *)
        echo "Invalid mode! Usage: $0 [cli|gui] <docker_image> <file_name> [<volume1> ... <volumeN>]"
        exit 1
        ;;
esac


echo "Starting Docker container: $container_name"
echo "Command: $docker_run_command"

eval "$docker_run_command"