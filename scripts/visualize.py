# Standard libraries
import argparse
import os
import re

# Third-party libraries
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import UnivariateSpline

args = None

def extract_result_from_file(filename):
    
    with open(filename, "r") as f:
        # Read lines from the file
        lines = f.readlines()

    x_idx, y_idx, z_idx = 0, 0, 0
    abs_error_idx, ori_error_idx = 0, 0
    duration_frame_idx = 0
    meanATE_idx, maxATE_idx, RPE_RMSE_idx = 0, 0, 0
    cpu_memory_idx, gpu_memory_idx = 0, 0

    poses = []
    abs_errors = []
    ori_errors = []
    duration_frames = []
    meanATE, maxATE, RPE_RMSE = -1, -1, -1
    cpu_memory, gpu_memory = 0, 0

    has_accuracy = False

    # Loop through each line in the file
    idx = 0
    for line in lines:
        parts = line.strip().split('\t')

        # If the line contains "Frame Number", it's the header
        if "Frame Number" in line:
            # Get the indices of X, Y, Z from the header
            values_to_check = ["AbsoluteError", "OrientationError", "MeanATE", "MaxATE", "RPE_RMSE"]
            has_accuracy = all(value in parts for value in values_to_check)

            if has_accuracy:
                abs_error_idx, ori_error_idx = parts.index("AbsoluteError"), parts.index("OrientationError")
                meanATE_idx, maxATE_idx, RPE_RMSE_idx = parts.index("MeanATE"), parts.index("MaxATE"), parts.index("RPE_RMSE")

            x_idx, y_idx, z_idx = parts.index("X"), parts.index("Y"), parts.index("Z")
            duration_frame_idx = parts.index("Duration_Frame")
            cpu_memory_idx, gpu_memory_idx = parts.index("CPU_Memory"), parts.index("GPU_Memory")

            continue
        
        if parts[abs_error_idx] == "-nan" or parts[abs_error_idx] == "nan":
            continue

        # Check if the first part is a string that represents an integer
        if re.match(r"^\d+$", parts[0]):
            x, y, z = parts[x_idx], parts[y_idx], parts[z_idx]
            poses.append((x, y, z))

            if has_accuracy:
                abs_error, ori_error = float(parts[abs_error_idx]), float(parts[ori_error_idx])
                abs_errors.append(abs_error)
                ori_errors.append(ori_error)
                meanATE, maxATE, RPE_RMSE = float(parts[meanATE_idx]), float(parts[maxATE_idx]), float(parts[RPE_RMSE_idx])

            duration_frames.append(float(parts[duration_frame_idx]))
            cpu_memory, gpu_memory = float(parts[cpu_memory_idx]), float(parts[gpu_memory_idx])

        else:
            continue

    return poses, abs_errors, ori_errors, duration_frames, meanATE, maxATE, RPE_RMSE, cpu_memory, gpu_memory


def extract_xyz_from_gt(filename):
    """Extracts X, Y, Z coordinates from the given file."""
    # Open the file
    with open(filename, "r") as f:
        # Read lines from the file
        lines = f.readlines()

    coordinates = []

    # Loop through each line in the file
    for line in lines:
        # Skip empty or whitespace-only lines
        if line.strip() == '':
            continue
        # Splitting by whitespace
        parts = line.strip().split()
        # Skip lines where split parts are not 16
        if len(parts) != 16 and len(parts) != 12:
            continue

        # Convert the values to float before storing, assuming that's desired
        x, y, z = float(parts[3]), float(parts[7]), float(parts[11])
        coordinates.append((x, y, z))

    return coordinates


def visualize_coordinates(coords, labels):
    """Visualizes a list of 3D coordinates using matplotlib."""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    for coord, label in zip(coords, labels):
        # Splitting the coordinates
        xs, ys, zs = zip(*coord)

        # Convert to float for plotting
        xs = [float(x) for x in xs]
        ys = [float(y) for y in ys]
        zs = [float(z) for z in zs]

        # Interpolating to make the line smoother
        x_spline = UnivariateSpline(range(len(xs)), xs, s=1)
        y_spline = UnivariateSpline(range(len(ys)), ys, s=1)
        z_spline = UnivariateSpline(range(len(zs)), zs, s=1)

        xs_smooth = x_spline(np.linspace(0, len(xs)-1, 300))
        ys_smooth = y_spline(np.linspace(0, len(ys)-1, 300))
        zs_smooth = z_spline(np.linspace(0, len(zs)-1, 300))

        ax.plot(xs_smooth, ys_smooth, zs_smooth, label=label, linewidth=0.9)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_box_aspect([1, 1, 1])
    # Assuming the points from different algorithms are in the same general space
    # to ensure a similar scale on all axes
    max_range = max(np.max(xs) - np.min(xs), np.max(ys) - np.min(ys), np.max(zs) - np.min(zs))
    mean_x = np.mean(xs)
    mean_y = np.mean(ys)
    mean_z = np.mean(zs)

    ax.set_xlim(mean_x - max_range / 2, mean_x + max_range / 2)
    ax.set_ylim(mean_y - max_range / 2, mean_y + max_range / 2)
    ax.set_zlim(mean_z - max_range / 2, mean_z + max_range / 2)
    
    ax.legend()  # Add this to show the labels as legend

    # Save the figure
    if args.output_folder:
        if not os.path.exists(args.output_folder):
            os.makedirs(args.output_folder, exist_ok=True)

        output_filepath = os.path.join(args.output_folder, f"Trajectory.png")
        plt.savefig(output_filepath, dpi=300)
        print(f"Saved Trajectory.png to: {output_filepath}")

    if args.plot:
        plt.show()
    
    plt.close(fig)


def visualize_metrices(values_list, algo_names, metric="NONE"):
    plt.figure(figsize=(12, 8))
    # Ensure the values_list and algo_names have the same length
    if len(values_list) != len(algo_names):
        raise ValueError("The lengths of values_list and algo_names must be the same.")
    
    # Plot every third algorithm's values
    for i, values in enumerate(values_list):
        # Generate x-values that match the length of values
        x_values = range(len(values))
        
        # Select every third x-value and y-value for plotting
        selected_x_values = x_values #[::2]
        selected_values = values #[::2]

        # Plot the selected points
        line, = plt.plot(selected_x_values, selected_values, label=algo_names[i], marker='o', markersize=1, linestyle='')
        
        # Calculate mean of the values and plot it
        mean_value = sum(values) / len(values)
        plt.axhline(mean_value, color=line.get_color(), linestyle='-', linewidth=1.5, 
                    label=f"mean of {algo_names[i]}")

    plt.xlabel("Frames")
    plt.ylabel(metric)
    plt.title(f"Comparison of {metric} between Algorithms")
    
    ax = plt.gca()  # Get the current axis for more control

    # Format the y-axis to a lower precision
    ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.1e'))

    # Limit the number of ticks on the y-axis for clarity
    ax.yaxis.set_major_locator(ticker.MaxNLocator(nbins=10, prune='both'))

    # Rotate y-axis labels for better visibility
    for label in ax.get_yticklabels():
        label.set_rotation(45)
    
    plt.legend()

    # Save the figure
    if args.output_folder:
        if not os.path.exists(args.output_folder):
            os.makedirs(args.output_folder, exist_ok=True)

        output_filepath = os.path.join(args.output_folder, f"{metric}.png")
        plt.savefig(output_filepath, dpi=300)
        print(f"Saved {metric}.png to: {output_filepath}")

    if args.plot:
        plt.tight_layout()
        plt.show()
    
    plt.close()


def visualize_bar_graph(meanATE_list, maxATE_list, RPE_RMSE_list, algo_names):
    # Number of algorithms
    n_algos = len(algo_names)

    # Set the positions and width for the bars
    bar_width = 0.1
    index = np.arange(3)  # Three groups: meanATE, maxATE, and RPE_RMSE

    fig, ax = plt.subplots(figsize=(10, 6))

    for i, (mean, max_, rpe_rmse, name) in enumerate(zip(meanATE_list, maxATE_list, RPE_RMSE_list, algo_names)):
        ax.bar(index + i*bar_width, [mean, max_, rpe_rmse], bar_width, label=name)

    # Add some text for labels, title, and custom x-axis tick labels, etc.
    ax.set_xlabel('Metrices')
    ax.set_ylabel('Values')
    ax.set_title('Comparison of Different Algorithms')
    ax.set_xticks(index + (bar_width * n_algos / 2) - bar_width/2)
    ax.set_xticklabels(['meanATE', 'maxATE', 'RPE_RMSE'])
    ax.legend()

    if args.output_folder:
        if not os.path.exists(args.output_folder):
            os.makedirs(args.output_folder, exist_ok=True)

        output_filepath = os.path.join(args.output_folder, f"ATE_RPE.png")
        plt.savefig(output_filepath, dpi=300)
        print(f"Saved ATE_RPE.png to: {output_filepath}")

    if args.plot:
        plt.tight_layout()
        plt.show()

    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Visualize 3D coordinates.")
    parser.add_argument('-gt', '--groundtruth', help="Path to the groundtruth file (optional).")
    parser.add_argument('-a', '--algorithms', required=True, nargs='+', help="Paths to metrices files of algorithms")
    parser.add_argument('-o', '--output-folder', help="Path to output folder")
    parser.add_argument('-plt', '--plot', action='store_true', help="Plot graph or not")

    global args
    args = parser.parse_args()

    if not args.plot and not args.output_folder:
        print("You need to enable one of --ouput-folder and --plot")
        return

    poses_list = []
    abs_errors_list = []
    ori_errors_list = []

    duration_frames_list = []
    mean_durantion_frame_list = []

    meanATE_list = []
    maxATE_list = []
    RPE_RMSE_list = []

    cpu_memory_list = []
    gpu_memory_list = []

    algo_names = []

    # Handle the groundtruth file if provided
    if args.groundtruth:
        poses_list.append(extract_xyz_from_gt(args.groundtruth))
    
    for filepath in args.algorithms:
        poses, abs_errors, ori_errors, duration_frames, meanATE, maxATE, RPE_RMSE, cpu_memory, gpu_memory = extract_result_from_file(filepath)
        poses_list.append(poses)
        # abs_errors_list.append(abs_errors)
        # ori_errors_list.append(ori_errors)
        duration_frames_list.append(duration_frames)
        mean_durantion_frame_list.append(sum(duration_frames) / len(duration_frames))

        meanATE_list.append(meanATE)
        maxATE_list.append(maxATE)
        RPE_RMSE_list.append(RPE_RMSE)

        cpu_memory_list.append(cpu_memory)
        gpu_memory_list.append(gpu_memory)

        # For visualization, the base name of the file can be used as a label 
        algo_names.append(os.path.basename(filepath).split('_')[0])

    # ===== Plot the figure ======
    if args.groundtruth:
        visualize_coordinates(poses_list, ["GroundTruth"] + algo_names)
    else:
        visualize_coordinates(poses_list, algo_names)

    # if abs_errors_list[0] != [] and ori_errors_list[0] != []:
    #     visualize_metrices(abs_errors_list, algo_names, "Absolute Error")
    #     visualize_metrices(ori_errors_list, algo_names, "Orientation Error")

    visualize_metrices(duration_frames_list, algo_names, "Duration Frame")

    if meanATE_list[0] != -1 and maxATE_list[0] != -1 and RPE_RMSE_list[0] != -1:
        visualize_bar_graph(meanATE_list, maxATE_list, RPE_RMSE_list, algo_names)

    # ===== Save result in txt file ======
    if args.output_folder:
        output_filepath = os.path.join(args.output_folder, f"results.txt")
        with open(output_filepath, "w") as file:
            for name, mean_ate, max_ate, rpe_rmse, mean_dur, cpu_mem, gpu_mem in \
            zip(algo_names, meanATE_list, maxATE_list, RPE_RMSE_list, mean_durantion_frame_list, cpu_memory_list, gpu_memory_list):
                file.write(f"===== {name} =====\n")
                file.write(f"meanATE: {mean_ate}\n")
                file.write(f"maxATE: {max_ate}\n")
                file.write(f"RPE_RMSE: {rpe_rmse}\n")
                file.write(f"meanDuration: {mean_dur}\n")
                file.write(f"CPU_Memory: {cpu_mem}\n")
                file.write(f"GPU_Memory: {gpu_mem}\n\n")
            print(f"Saved results.txt to: {output_filepath}")

if __name__ == "__main__":
    main()


"""
def main():
    parser = argparse.ArgumentParser(description="Visualize 3D coordinates.")
    parser.add_argument('-d', '--dataset', required=True, help="Dataset name in lowercase e.g. kitti")
    parser.add_argument('-a', '--algorithms', required=True, nargs='+', help="Algorithms e.g. legoloam aloam")

    args = parser.parse_args()

    coords_list = []
    for algo in args.algorithms:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        filename = os.path.join(script_dir, "..", "result", args.dataset, algo, f"{algo}-{args.dataset}-result.txt")
        coords_list.append(extract_xyz_from_file(filename))
    
    visualize_coordinates(coords_list, args.algorithms)


if __name__ == "__main__":
    main()
"""
