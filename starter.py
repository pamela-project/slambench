#!/usr/bin/env python3
import sys
import os
import platform
import argparse 
import subprocess


def build_handle():
    try:
        output = subprocess.check_output("docker pull slambench/main ", shell=True, text=True)
    except:
        print("Image download Failed...")
    images = subprocess.check_output("docker images", shell=True, text=True)
    if "slambench/main" not in images:
        print("Seems like download failed, cannot find image. Build starting ...")
        current_directory = os.getcwd()
        if current_directory.endswith("/slambench"):
            os.system("docker build . -t slambench/main")
        else: 
            os.system("git clone https://github.com/nikolaradulov/slambench.git")     
            os.system("docker build ./slambench -t slambench/main")
    else:
        print("A version of the container is already present on host and it shall be used. Alternatively the image can manually be build by \'docker build <path to Dockerfile -t slambench/main\'")

def is_wsl():
    return "microsoft" in platform.uname().release

def print_usage():
    print("Usage: {} <mode> [options]".format(sys.argv[0]))
    print("\nModes:")
    print("  run         - Run benchmarks with provided arguments.")
    print("  build       - Download or build the container for SLAMBench.")
    print("  dataset     - Create a specified dataset.")
    print("\nOptions:")
    print("  For 'run' mode:")
    print("Usage: {} run [options] <dataset_location_in_volume> [<library1_path> ... <libraryN_path>]".format(sys.argv[0]))
    print("    bench-cli         - Run a benchmark with provided arguments using the command line as interface.")
    print("    bench-gui         - Run a benchmark with provided arguments with the visualisation tool of SLAMBench.")
    print("    interactive-cli   - Run the container in interactive mode with mounts for datasets and algorithms.")
    print("    interactive-gui   - Run the container in interactive mode with mounts for datasets and algorithms. It also provides windowing support for applications.")
    print("\nFor 'build' mode:")
    print("    Specify 'download' to download the container.")
    print("    Specify 'build' to build the container.")
    print("\nFor 'dataset' mode:")
    print("Usage: {} dataset [options] <volume_name> <dataset>".format(sys.argv[0]))
    print("    list              - print list of available datasets")
    sys.exit(1)

def dataset_handle(run_type, vol_name, dataset):
    if run_type=='list':
        return "docker run slambench/main --list_datasets"
    else: 
        if vol_name is None or dataset is None:
            print("starter.py dataset: error: the following arguments are required: -v/--volume_name, -d/--dataset")
            sys.exit(1)
        return f"docker run --mount source={vol_name},destination=/slambench/datasets slambench/main --dataset {dataset}"

def run_handle(mode, volumes, files, paths, save, extra):
    """
        The function handles the running options of the tool. 
    """

    if not mode.startswith('interactive'):
        if volumes is None or files is None or paths is None:
            print(f"{sys.argv[0]} {mode}: error: the following arguments are required -dv/--dataset_volume, -d/--dataset, -a/--algorithm")
            sys.exit(1)
    image = "slambench/main"
    # Start the Docker container
    container_name = "{}-container".format(image)
    deps_dir = "/deps"
    docker_run_command=""
    for volume in volumes:
        docker_run_command += "--mount source={},destination=/slambench/datasets".format(volume)
    end = ""
    for file in files:
        end+=f"{file} "
    volumes = []
    for path in paths:
        names = path.split('/')
        volumes.append("{}-vol".format(names[0]))
        end += "{} ".format('/deps/'+names[0]+'/lib/'+names[1])

    if len(extra)!=0:
        for val in extra:
            end += f" {val}"
    
    # Mount volumes in the Docker container
    for volume in volumes:
        # Remove the 'vol' suffix
        volume_name = volume[:-4]
        docker_run_command += " -it --mount source={},destination=/deps/{}".format(volume, volume_name)

    # command line mode. Outputs the benchmark of the loaded algorithms with the loaded datasets  
    if mode == "cli":
        docker_run_command = "docker run {} {} --bench-cli {}".format(docker_run_command, image, end)
    # GUI mode. Starts the visualisation tool of SLAMBench 
    elif mode == "gui":
        if is_wsl():
            docker_run_command = "docker run -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY={} --device /dev/dri/card0 --device /dev/dri/renderD128 -e WAYLAND_DISPLAY={} -e XDG_RUNTIME_DIR={} {} {} --bench-gui {}".format(os.environ["DISPLAY"], os.environ["WAYLAND_DISPLAY"], os.environ["XDG_RUNTIME_DIR"], docker_run_command, image, end)
        else: 
            docker_run_command = "docker run --env=\"DISPLAY\"  --env=\"QT_X11_NO_MITSHM=1\" -v /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri/card0 --device /dev/dri/renderD128 {} {} --bench-gui {}".format( docker_run_command, image, end)
            # give access to the docker daemon
            os.system('xhost +local:docker')
            print("Linux support for GUI in development")
            # sys.exit(1)
    # interactive mode. Supportsa windowing for GUI
    elif mode == "interactive-gui":
        if is_wsl():
            docker_run_command = "docker run -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY={} --device /dev/dri/card0 --device /dev/dri/renderD128 -e WAYLAND_DISPLAY={} -e XDG_RUNTIME_DIR={} {} {} --interactive ".format(os.environ["DISPLAY"], os.environ["WAYLAND_DISPLAY"], os.environ["XDG_RUNTIME_DIR"], docker_run_command, image)
        else:
            docker_run_command = "docker run --env=\"DISPLAY\" --device /dev/dri/card0 --device /dev/dri/renderD128 --env=\"QT_X11_NO_MITSHM=1\" -v /tmp/.X11-unix:/tmp/.X11-unix:rw {} {} --interactive ".format( docker_run_command, image)
            # give access to the docker daemon
            os.system('xhost +local:docker')
    # interactive mode. Command line only 
    elif mode == "interactive-cli":
        docker_run_command = "docker run {} {} --interactive ".format(docker_run_command, image)
    else:
        print("Invalid mode! Usage: {} [cli|gui] <docker_image> <file_name> [<volume1> ... <volumeN>]".format(sys.argv[0]))
        sys.exit(1)

    print("Starting Docker container: {}".format(container_name))
    # print("Command: {}".format(docker_run_command))
    
    # save the config file from the provided settings
    if save is not None: 
        config = configparser.ConfigParser()
        config['DEFAULT']={'Mode':mode}
        config['DATASET']={'Volume':volume,
                            'Path': files}
        counter =1
        for path in paths:
            names = path.split('/')
            config['ALGORITHM'+str(counter)]={'Volume': names[0],
                                              'Implentation': path}
        with open(save, 'w') as configfile:
            config.write(configfile)
    return docker_run_command

def main():
    print(is_wsl())
    parser = argparse.ArgumentParser(description="This is a tool to run Docker containers with SLAMBench.")
    subparsers = parser.add_subparsers(dest="mode", help="Select the mode of operation.")

    
    run_parser = subparsers.add_parser("run", help="When running the tool in run mode")
    run_parser.add_argument("-dv","--dataset_volume", nargs=1,  type=str, help="Specify the volume to be mounted for the dataset.")
    run_parser.add_argument("-d", "--dataset",nargs=1,  type=str, help="Specify the dataset file to be used")
    run_parser.add_argument("-a","--algorithm", nargs="*", help="Specify algorithms to be used. Must be of the form <algorithm_name>/<library_name>. Refer to the wiki for more information.")
    run_parser.add_argument("-t", "--type", choices=['cli', 'gui', 'interactive-cli', 'interactive-gui'], required=True)
    run_parser.add_argument("-s","--save_config",  help="Save current configuration to config file")
    run_parser.add_argument("slamopt", nargs="*")
    

    build_parser = subparsers.add_parser("install", help="Downloads the main container/builds it")
    
    

    dataset_parser = subparsers.add_parser("dataset", help="Running the tool in dataset mode")
    dataset_parser.add_argument("-t", "--type", choices=['list','make'], required=True)
    dataset_parser.add_argument("-v", "--volume_name", help="Specify the volume name.")
    dataset_parser.add_argument("-d", "--dataset", help="Specify the dataset.")
    
    args = parser.parse_args()
    if args.mode == "run":
        # get the arguments after the separator
        docker_run_command = run_handle(args.type, args.dataset_volume, args.dataset, args.algorithm, args.save_config, args.slamopt)
    elif args.mode == "install":
        docker_run_command = build_handle()
        sys.exit(0)
    elif args.mode == "dataset": 
        docker_run_command = dataset_handle(args.type, args.volume_name, args.dataset)
    else:
        print("Invalid mode! Use one of the following:")
        print("  run      - Run benchmarks with provided arguments.")
        print("  build    - Download or build the container for SLAMBench.")
        print("  dataset  - Create a specified dataset.")
        sys.exit(1)
    
    print("Command: {}".format(docker_run_command))
    os.system(docker_run_command)
    if not is_wsl():
        os.system('xhost -local:docker')

if __name__ == "__main__":
    main()
