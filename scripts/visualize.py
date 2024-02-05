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

def calculate_rmse(abs_errors):
    squared_errors = np.square(abs_errors)
    mean_squared_error = np.mean(squared_errors)
    rmse = np.sqrt(mean_squared_error)
    return rmse

def extract_result_from_file(filename, input_gt_length):
    
    with open(filename, "r") as f:
        # Read lines from the file
        lines = f.readlines()

    x_idx, y_idx, z_idx = 0, 0, 0
    duration_frame_idx = 0
    meanATE_idx, maxATE_idx = 0, 0
    ATE_RMSE_idx, RPE_RMSE_idx = 0, 0
    cpu_memory_idx, gpu_memory_idx = 0, 0

    poses = [(0.0, 0.0, 0.0)]
    duration_frames = []
    meanATE, maxATE, ATE_RMSE, RPE_RMSE = -1, -1, -1, -1
    cpu_memory, gpu_memory = 0, 0

    has_accuracy = False

    # Loop through each line in the file
    for line in lines:
        parts = line.strip().split('\t')

        # If the line contains "Frame Number", it's the header
        if "Frame Number" in line:
            # Get the indices of X, Y, Z from the header
            values_to_check = ["MeanATE", "MaxATE", "ATE_RMSE", "RPE_RMSE"]
            has_accuracy = all(value in parts for value in values_to_check)

            if has_accuracy:
                meanATE_idx, maxATE_idx = parts.index("MeanATE"), parts.index("MaxATE")
                ATE_RMSE_idx, RPE_RMSE_idx = parts.index("ATE_RMSE"), parts.index("RPE_RMSE")

            x_idx, y_idx, z_idx = parts.index("X"), parts.index("Y"), parts.index("Z")
            duration_frame_idx = parts.index("Duration_Frame")
            cpu_memory_idx, gpu_memory_idx = parts.index("CPU_Memory"), parts.index("GPU_Memory")

            continue
        
        if parts[ATE_RMSE_idx] in ("-nan", "nan") or parts[maxATE_idx] in ("-nan", "nan") or parts[RPE_RMSE_idx] in ("-nan", "nan"):
            continue

        # Check if the first part is a string that represents an integer
        if re.match(r"\d+", parts[0]):
            # print(parts[0])
            x, y, z = parts[x_idx], parts[y_idx], parts[z_idx]
            poses.append((x, y, z))

            if has_accuracy:
                meanATE, maxATE = float(parts[meanATE_idx]), float(parts[maxATE_idx])
                ATE_RMSE, RPE_RMSE = float(parts[ATE_RMSE_idx]), float(parts[RPE_RMSE_idx])

            duration_frames.append(float(parts[duration_frame_idx]))
            cpu_memory, gpu_memory = float(parts[cpu_memory_idx]), float(parts[gpu_memory_idx])

            if (int(parts[0]) >= input_gt_length):
                break
        else:
            continue

    return poses, duration_frames, meanATE, maxATE, ATE_RMSE, RPE_RMSE, cpu_memory, gpu_memory


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

        if label == "GroundTruth":
            ax.plot(xs_smooth, ys_smooth, zs_smooth, label=label, linewidth=0.9, linestyle="--", color="black")
        else:
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
    
    plt.legend(loc='upper left')

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


def visualize_bar_graph(meanATE_list, maxATE_list, ATE_RMSE_list, algo_names):
    # Number of algorithms
    n_algos = len(algo_names)

    # Set the positions and width for the bars
    bar_width = 0.1
    index = np.arange(3)  # Now four groups: meanATE, maxATE, ATE_RMSE

    fig, ax = plt.subplots(figsize=(10, 6))

    for i, (mean, max_, ate_rmse, name) in enumerate(zip(meanATE_list, maxATE_list, ATE_RMSE_list, algo_names)):
        ax.bar(index + i*bar_width, [mean, max_, ate_rmse], bar_width, label=name)

    # Add some text for labels, title, and custom x-axis tick labels, etc.
    ax.set_xlabel('Metrices')
    ax.set_ylabel('Values')
    ax.set_title('Comparison of Different Algorithms')
    ax.set_xticks(index + (bar_width * n_algos / 2) - bar_width/2)
    ax.set_xticklabels(['meanATE', 'maxATE', 'ATE_RMSE'])
    ax.legend()

    if args.output_folder:
        if not os.path.exists(args.output_folder):
            os.makedirs(args.output_folder, exist_ok=True)

        output_filepath = os.path.join(args.output_folder, f"ATE.png")
        plt.savefig(output_filepath, dpi=300)
        print(f"Saved ATE.png to: {output_filepath}")

    if args.plot:
        plt.tight_layout()
        plt.show()

    plt.close(fig)


def compute_ATE(groundtruth, estimated):
    """
    Compute Absolute Trajectory Error (ATE) for each pose.

    Args:
    - groundtruth: List of groundtruth poses.
    - estimated: List of estimated poses.

    Returns:
    - A list of ATE values for each pose.
    """
    # assert len(groundtruth) == len(estimated), "Both lists must have the same length."

    ate_values = []
    # count = 701
    for gt, est in zip(groundtruth[:701], estimated[:701]):
        # Convert to numpy arrays of float type
        gt_array = np.array(gt, dtype=float)
        est_array = np.array(est, dtype=float)
        
        error = gt_array - est_array
        ate = np.linalg.norm(error)
        ate_values.append(ate)
    
    return ate_values


def compute_RPE(groundtruth, estimated):
    """
    Compute the Relative Pose Error (RPE) for each frame between a ground truth trajectory and an estimated trajectory.
    
    Parameters:
    - groundtruth: list of ground truth poses [(x1, y1, z1), (x2, y2, z2), ...]
    - estimated: list of estimated poses [(x1, y1, z1), (x2, y2, z2), ...]
    
    Returns:
    - rpes: List of RPE values for each frame
    """
    
    # assert len(groundtruth) == len(estimated), "The two lists should have the same length"
    if len(groundtruth) < len(estimated):
        N = len(groundtruth)
    else:
        N = len(estimated)
    rpes = []

    for i in range(2, N-1):
        delta_gt = np.array([
            float(groundtruth[i][0]) - float(groundtruth[i-1][0]),
            float(groundtruth[i][1]) - float(groundtruth[i-1][1]),
            float(groundtruth[i][2]) - float(groundtruth[i-1][2])
        ])
        
        delta_estimated = np.array([
            float(estimated[i][0]) - float(estimated[i-1][0]),
            float(estimated[i][1]) - float(estimated[i-1][1]),
            float(estimated[i][2]) - float(estimated[i-1][2])
        ])

        rpe = np.linalg.norm(delta_gt - delta_estimated)
        rpes.append(rpe)
    
    return rpes


def moving_average(data, window_size=10):
    cumsum_vec = np.cumsum(np.insert(data, 0, 0))
    ma_vec = (cumsum_vec[window_size:] - cumsum_vec[:-window_size]) / window_size
    return ma_vec


def plot_ATE_or_RPE(rpe_values, labels, start_index=0, end_index=0, transparency=1.0, smooth=False, reduce_resolution=False):
    """
    Plot RPE values.

    Args:
    ...
    - smooth: If True, applies moving average to smooth data.
    - reduce_resolution: If True, plots 1 out of every five points.
    """
    
    assert len(rpe_values) == len(labels), "Number of RPE values should match the number of labels"
    
    for idx, rpe_value in enumerate(rpe_values):
        current_end_index = end_index if end_index else len(rpe_value)
        x_values = range(start_index, current_end_index)
        
        if smooth:
            smoothed_rpe = moving_average(rpe_value[start_index:current_end_index])
            plt.plot(x_values[len(x_values)-len(smoothed_rpe):], smoothed_rpe, label=labels[idx], alpha=transparency)
        elif reduce_resolution:
            reduced_x = x_values[::5]
            reduced_rpe = rpe_value[start_index:current_end_index:5]
            plt.plot(reduced_x, reduced_rpe, label=labels[idx], alpha=transparency)
        else:
            plt.plot(x_values, rpe_value[start_index:current_end_index], label=labels[idx], alpha=transparency)

        transparency -= 0.1
        
    plt.xlabel('Frames', fontsize=12)
    plt.ylabel('RPE (m)', fontsize=12)
    # plt.yscale('log')
    plt.grid(True)
    plt.legend(loc="upper left", fontsize=12)

    if args.output_folder:
        if not os.path.exists(args.output_folder):
            os.makedirs(args.output_folder, exist_ok=True)

        output_filepath = os.path.join(args.output_folder, f"RPE.png")
        plt.savefig(output_filepath, dpi=300)
        print(f"Saved RPE.png to: {output_filepath}")

    if args.plot:
        plt.show()

    plt.close()


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

    duration_frames_list = []
    mean_durantion_frame_list = []

    meanATE_list = []
    maxATE_list = []
    ATE_RMSE_list = []
    RPE_RMSE_list = []
    RPE_list = []
    Highest_RPE_list = []

    cpu_memory_list = []
    gpu_memory_list = []

    algo_names = []

    # Handle the groundtruth file if provided
    gt_length = 10000
    if args.groundtruth:
        poses_list.append(extract_xyz_from_gt(args.groundtruth))
        gt_length = len(poses_list[0])

    for filepath in args.algorithms:
        poses, duration_frames, meanATE, maxATE, ATE_RMSE, RPE_RMSE, cpu_memory, gpu_memory = extract_result_from_file(filepath, gt_length)
        poses_list.append(poses)
        duration_frames_list.append(duration_frames)
        mean_durantion_frame_list.append(sum(duration_frames) / len(duration_frames))

        meanATE_list.append(meanATE)
        maxATE_list.append(maxATE)
        ATE_RMSE_list.append(ATE_RMSE)
        RPE_RMSE_list.append(RPE_RMSE)

        cpu_memory_list.append(cpu_memory)
        gpu_memory_list.append(gpu_memory)

        # For visualization, the base name of the file can be used as a label 
        algo_names.append(os.path.basename(filepath).split('_')[0])

    if args.groundtruth:
        for i in range(len(poses_list)):
            if i != 0:
                rpe = compute_RPE(poses_list[0], poses_list[i])
                indexed_rpe = sorted(enumerate(rpe), key=lambda x: x[1], reverse=True)
                top_10_indices = [index for index, value in indexed_rpe[:10]]
                top_10_indices = sorted(top_10_indices)
                Highest_RPE_list.append(top_10_indices)
                RPE_list.append(rpe)
        plot_ATE_or_RPE(RPE_list, algo_names)


    # ===== Plot the figure ======
    if args.groundtruth:
        visualize_coordinates(poses_list, ["GroundTruth"] + algo_names)
    else:
        visualize_coordinates(poses_list, algo_names)

    visualize_metrices(duration_frames_list, algo_names, "Duration Frame")

    if meanATE_list[0] != -1 and maxATE_list[0] != -1 and ATE_RMSE_list[0] != -1:
        visualize_bar_graph(meanATE_list, maxATE_list, ATE_RMSE_list, algo_names)

    # ===== Save result in txt file ======
    if args.output_folder and args.groundtruth:
        output_filepath = os.path.join(args.output_folder, f"results.txt")
        with open(output_filepath, "w") as file:
            file.write(f"Number of Poses: {gt_length}\n\n")
            for name, mean_ate, max_ate, ate_rmse, rpe_rmse, highest_rpe_frame, mean_dur, cpu_mem, gpu_mem in \
            zip(algo_names, meanATE_list, maxATE_list, ATE_RMSE_list, RPE_RMSE_list, Highest_RPE_list, mean_durantion_frame_list, cpu_memory_list, gpu_memory_list):
                file.write(f"===== {name} =====\n")
                file.write(f"meanATE: {mean_ate}\n")
                file.write(f"maxATE: {max_ate}\n")
                file.write(f"ATE_RMSE: {ate_rmse}\n")
                file.write(f"RPE_RMSE: {rpe_rmse}\n")
                file.write("Frame Number with High RPE: " + ",".join(map(str, highest_rpe_frame)) + "\n")
                file.write(f"meanDuration: {mean_dur}\n")
                file.write(f"CPU_Memory: {cpu_mem}\n")
                file.write(f"GPU_Memory: {gpu_mem}\n\n")
            print(f"Saved results.txt to: {output_filepath}")
    elif args.output_folder:
        output_filepath = os.path.join(args.output_folder, f"results.txt")
        with open(output_filepath, "w") as file:
            file.write(f"Number of Poses: {gt_length}\n\n")
            for name, mean_ate, max_ate, ate_rmse, rpe_rmse, mean_dur, cpu_mem, gpu_mem in \
            zip(algo_names, meanATE_list, maxATE_list, ATE_RMSE_list, RPE_RMSE_list, mean_durantion_frame_list, cpu_memory_list, gpu_memory_list):
                file.write(f"===== {name} =====\n")
                file.write(f"meanATE: {mean_ate}\n")
                file.write(f"maxATE: {max_ate}\n")
                file.write(f"ATE_RMSE: {ate_rmse}\n")
                file.write(f"RPE_RMSE: {rpe_rmse}\n")
                file.write(f"meanDuration: {mean_dur}\n")
                file.write(f"CPU_Memory: {cpu_mem}\n")
                file.write(f"GPU_Memory: {gpu_mem}\n\n")
            print(f"Saved results.txt to: {output_filepath}")


if __name__ == "__main__":
    main()
