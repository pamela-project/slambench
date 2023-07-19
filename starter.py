#!/usr/bin/env python3
import sys
import os
import platform

def is_wsl():
    return "Microsoft" in platform.uname().release

def print_usage():
    print("Usage: {} <mode> <mode arguments> ".format(sys.argv[0]))
    print("\nModes:")
    print("  run                  - Run the Docker container with specified image and options.")
    print("  dataset              - Create a dataset using specified volume and paths.")
    print("\nAdditional arguments:")
    print("  --bench-cli <docker_image> <dataset_location> [<volume1> ... <volumeN>]   - Additional arguments for the benchmark in 'run' mode.")
    print("  --bench-gui <docker_image> <dataset_location> [<volume1> ... <volumeN>]   - Additional arguments for the GUI in 'run' mode.")
    print("  --interactive-cli <dataset_location> <file_name> [<volume1> ... <volumeN>]   - Run in interactive command-line mode.")
    print("  --interactive-gui  <dataset_location> <file_name> [<volume1> ... <volumeN>]  - Run in interactive GUI mode.")
    sys.exit(1)

def make_datset():
    vol_name = sys.argv[3]
    dataset = sys.argv[4]
    return f"docker run --mount source={vol_name},destination=/slambench/datasets slambench/main {dataset}"

def run_handle():
    """
        The function handles the running options of the tool. 
    """
    mode = sys.argv[2]
    image = sys.argv[3]
    file = sys.argv[4]
    paths = sys.argv[5:]

    # Start the Docker container
    container_name = "{}-container".format(image)
    deps_dir = "/deps"
    file_items=items = file.split("/")
    docker_run_command = "--mount source={},destination=/slambench/datasets".format(file_items[1])
    end = file +" "

    volumes = []
    for path in paths:
        name = os.path.basename(os.path.dirname(path))
        volumes.append("{}-vol".format(name))
        end += "{} ".format(path)

    print(end)
    print(volumes)

    # Mount volumes in the Docker container
    for volume in volumes:
        # Remove the 'vol' suffix
        volume_name = volume[:-4]
        docker_run_command += " -it --mount source={},destination=/deps/{}".format(volume, volume_name)

    # command line mode. Outputs the benchmark of the loaded algorithms with the loaded datasets  
    if mode == "--cli":
        docker_run_command = "docker run {} {} --bench-cli {}".format(docker_run_command, image, end)
    # GUI mode. Starts the visualisation tool of SLAMBench 
    elif mode == "--gui":
        if is_wsl:
            docker_run_command = "docker run -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY={} --device /dev/dri/card0 --device /dev/dri/renderD128 -e WAYLAND_DISPLAY={} -e XDG_RUNTIME_DIR={} {} {} --bench-gui {}".format(os.environ["DISPLAY"], os.environ["WAYLAND_DISPLAY"], os.environ["XDG_RUNTIME_DIR"], docker_run_command, image, end)
        else: 
            print("Linux support for GUI in development")
            sys.exit(1)
    # interactive mode. Supportsa windowing for GUI
    elif mode == "--interactive-gui":
        if is_wsl:
            docker_run_command = "docker run -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY={} --device /dev/dri/card0 --device /dev/dri/renderD128 -e WAYLAND_DISPLAY={} -e XDG_RUNTIME_DIR={} {} {} --interactive ".format(os.environ["DISPLAY"], os.environ["WAYLAND_DISPLAY"], os.environ["XDG_RUNTIME_DIR"], docker_run_command, image)
        else:
            print("Linux support for GUI in development")
    # interactive mode. Command line only 
    elif mode == "--interactive-cli":
        docker_run_command = "docker run {} {} --interactive ".format(docker_run_command, image)
    else:
        print("Invalid mode! Usage: {} [cli|gui] <docker_image> <file_name> [<volume1> ... <volumeN>]".format(sys.argv[0]))
        sys.exit(1)

    print("Starting Docker container: {}".format(container_name))
    print("Command: {}".format(docker_run_command))

    return docker_run_command

def main():
    # Check if the correct number of arguments is provided
    if len(sys.argv) < 2:
        print_usage()

    # Parse command-line arguments
    mode = sys.argv[1]
    if mode == 'run':
        docker_run_command = run_handle()
    elif mode == "dataset":
        docker_run_command = make_dataset()
    elif mode == "help":
        print_usage()
        sys.exit(1)

    os.system(docker_run_command)

if __name__ == "__main__":
    main()
