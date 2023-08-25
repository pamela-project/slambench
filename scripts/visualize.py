# Standard libraries
import argparse
import os
import re

# Third-party libraries
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import UnivariateSpline


def extract_xyz_from_file(filename):
    """Extracts X, Y, Z coordinates from the given file."""
    
    with open(filename, "r") as f:
        # Read lines from the file
        lines = f.readlines()

    x_idx, y_idx, z_idx = None, None, None

    coordinates = []

    # Loop through each line in the file
    for line in lines:
        parts = line.strip().split('\t')

        # If the line contains "Frame Number", it's the header
        if "Frame Number" in line:
            # Get the indices of X, Y, Z from the header
            x_idx, y_idx, z_idx = parts.index("X"), parts.index("Y"), parts.index("Z")
            continue

        # Check if the first part is a string that represents an integer
        if re.match(r"^\d+$", parts[0]):
            x, y, z = parts[x_idx], parts[y_idx], parts[z_idx]
            coordinates.append((x, y, z))
        else:
            continue

    return coordinates


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
    fig = plt.figure()
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
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Visualize 3D coordinates.")
    parser.add_argument('-gt', '--groundtruth', help="Path to the groundtruth file (optional).")
    parser.add_argument('-a', '--algorithms', required=True, nargs='+', help="Paths to result files for algorithms")

    args = parser.parse_args()

    coords_list = []
    algo_names = []

    # Handle the groundtruth file if provided
    if args.groundtruth:
        coords_list.append(extract_xyz_from_gt(args.groundtruth))
        algo_names.append("GroundTruth")
    
    for filepath in args.algorithms:
        coords_list.append(extract_xyz_from_file(filepath))
        # For visualization, the base name of the file can be used as a label 
        algo_names.append(os.path.basename(filepath).split('-')[0])
    
    visualize_coordinates(coords_list, algo_names)

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
