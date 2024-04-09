import matplotlib.pyplot as plt
import numpy as np
import json
import csv


def read_path_from_csv(filepath):
    path = []
    with open(filepath, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            # Assuming each row contains 'x,y'
            x, y = map(int, row)
            path.append((x, y))
    return path


def plot_path_on_map(mapfile, pathfile, wallsfile, step_length=1):
    # Initialize lists to hold the x and y coordinates
    x_coords = []
    y_coords = []

    # Open and read the file
    with open(mapfile, 'r') as file:
        for line in file:
            # Split the line into coordinates and convert them to floats
            coords = line.strip().split(',')
            x1, y1, x2, y2 = map(float, coords)

            # Append the start and end coordinates for plotting
            x_coords.extend([x1, x2, None])  # None to separate lines
            y_coords.extend([y1, y2, None])

    # Plotting
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(x_coords, y_coords, linestyle='-', color='blue')

    # Fill the cells with lines in yellow
    cells_with_lines = set()
    with open(wallsfile, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            x, y = map(int, row)
            x, y = untransform_cell(x, y, grid_size=step_length)
            cells_with_lines.add((x, y))
    for cell_x, cell_y in cells_with_lines:
        # print(f"Cell: ({cell_x}, {cell_y})")
        ax.add_patch(plt.Rectangle((cell_x, cell_y), step_length, step_length, color='yellow', alpha=0.3))

    # Setting a grid
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.grid(True, which='both', color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
    ax.set_aspect('equal', 'box')  # This is where "up to setting up the aspect ratio" ends.

    # Major ticks every 2, minor ticks every 1
    major_ticks = np.arange(-50, 51, step_length)
    ax.set_xticks(major_ticks)
    ax.set_yticks(major_ticks)

    # And a corresponding grid
    ax.grid(which='both', color='grey', linestyle='-', linewidth=0.5, alpha=0.3)

    # Hide markers by setting no tick labels
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    # Move axes to the center and hide the right and top lines
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('zero')
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')

    # Draw arrows
    ax.quiver(0, 0, 5, 0, angles='xy', scale_units='xy', scale=1, color='r', width=0.01)
    ax.quiver(0, 0, 0, 5, angles='xy', scale_units='xy', scale=1, color='g', width=0.01)

    # Unpack the path into X and Y coordinates
    path = read_path_from_csv(pathfile)
    path_real = [untransform_cell(x, y, grid_size=step_length) for x, y in path]
    path_x, path_y = zip(*path_real)
    ax.plot(path_x, path_y, marker='o', linestyle='-', color='red', label='Path', markersize=2)

    # Displaying the plot
    plt.show()


# def plot_map_modified(file_path, step_length=2, path=None):
#     # Initialize lists to hold the x and y coordinates
#     x_coords = []
#     y_coords = []

#     # Open and read the file
#     with open(file_path, 'r') as file:
#         for line in file:
#             # Split the line into coordinates and convert them to floats
#             coords = line.strip().split(',')
#             x1, y1, x2, y2 = map(float, coords)

#             # Append the start and end coordinates for plotting
#             x_coords.extend([x1, x2, None])  # None to separate lines
#             y_coords.extend([y1, y2, None])

#     # Plotting
#     fig, ax = plt.subplots(figsize=(10, 10))
#     ax.plot(x_coords, y_coords, linestyle='-', color='blue')

#     # Setting a grid
#     ax.set_xlim(-50, 50)
#     ax.set_ylim(-50, 50)
#     ax.grid(True, which='both', color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
#     ax.set_aspect('equal', 'box')  # This is where "up to setting up the aspect ratio" ends.

#     # Major ticks every 2, minor ticks every 1
#     major_ticks = np.arange(-50, 51, step_length)
#     ax.set_xticks(major_ticks)
#     ax.set_yticks(major_ticks)

#     # And a corresponding grid
#     ax.grid(which='both', color='grey', linestyle='-', linewidth=0.5, alpha=0.3)

#     # Hide markers by setting no tick labels
#     ax.set_xticklabels([])
#     ax.set_yticklabels([])

#     # Move axes to the center and hide the right and top lines
#     ax.spines['left'].set_position('zero')
#     ax.spines['bottom'].set_position('zero')
#     ax.spines['right'].set_color('none')
#     ax.spines['top'].set_color('none')

#     # Draw arrows
#     ax.quiver(0, 0, 5, 0, angles='xy', scale_units='xy', scale=1, color='r', width=0.01)
#     ax.quiver(0, 0, 0, 5, angles='xy', scale_units='xy', scale=1, color='g', width=0.01)

#     # # Define the diagonal connection plotting
#     # alpha_value = 0.15  # Transparency of the diagonal lines
#     # for x in np.arange(-50, 51, step_length):  # Ensure covering the edge by going one step beyond
#     #     for y in np.arange(-50, 51, step_length):
#     #         # Coordinates of the diagonal neighbors
#     #         diagonals = [(x + step_length, y + step_length), (x - step_length, y + step_length), (x + step_length, y - step_length), (x - step_length, y - step_length)]
#     #         for dx, dy in diagonals:
#     #             # Check if the diagonal point is within the grid bounds
#     #             if -50 <= dx <= 50 and -50 <= dy <= 50:
#     #                 ax.plot([x, dx], [y, dy], linestyle='-', color='red', linewidth=0.5, alpha=alpha_value)

#     # Plotting the path if provided
#     if path is not None:
#         # Unpack the path into X and Y coordinates
#         path_x, path_y = zip(*path)
#         ax.plot(path_x, path_y, marker='o', linestyle='-', color='red', label='Path', markersize=3)

#     # Displaying the plot
#     plt.show()


def untransform_cell(x, y, grid_size=1, grid_min=-50, grid_max=50):
    real_x = (x * grid_size) + grid_min
    real_y = grid_max - (y * grid_size)
    return real_x, real_y


def transform_cell(x, y, grid_size=1, grid_min=-50, grid_max=50):
    new_x = (x - grid_min) // grid_size
    new_y = (grid_max - y) // grid_size
    return int(new_x), int(new_y)


def transform_cells(cells_with_lines, grid_size=1, grid_min=-50, grid_max=50):
    transformed_cells = set()
    # The total number of cells along one axis
    total_cells_one_side = (grid_max - grid_min) // grid_size
    print(f"Total cells along one side: {total_cells_one_side}")
    for x, y in cells_with_lines:
        # Shift the origin to the top left and make y positive downwards
        new_x = (x - grid_min) // grid_size
        new_y = (grid_max - y) // grid_size  # Subtract from max to invert Y-axis
        transformed_cells.add((int(new_x), int(new_y)))
    return transformed_cells


def save_walls_to_csv(file_path, csv_filepath, step_length=1, extra_cells=0):
    # Initialize lists to hold the x and y coordinates
    x_coords = []
    y_coords = []
    # Initialize a set to hold the coordinates of the cells through which lines pass
    cells_with_lines = set()

    # Open and read the file
    with open(file_path, 'r') as file:
        for line in file:
            # Split the line into coordinates and convert them to floats
            coords = line.strip().split(',')
            x1, y1, x2, y2 = map(float, coords)

            # Append the start and end coordinates for plotting
            x_coords.extend([x1, x2, None])  # None to separate lines
            y_coords.extend([y1, y2, None])

            # For each line, determine the cells it passes through or touches
            min_x, max_x = min(x1, x2), max(x1, x2)
            min_y, max_y = min(y1, y2), max(y1, y2)

            # Iterate over cells covered by the line's bounding box
            for cell_x in np.arange(np.floor(min_x / step_length - extra_cells) * step_length, np.ceil(max_x / step_length + extra_cells) * step_length, step_length):
                for cell_y in np.arange(np.floor(min_y / step_length - extra_cells) * step_length, np.ceil(max_y / step_length + extra_cells) * step_length, step_length):
                    cells_with_lines.add((cell_x, cell_y))

    transformed_cells = transform_cells(cells_with_lines, grid_size=step_length)
    with open(csv_filepath, 'w', newline='') as file:
        writer = csv.writer(file)
        for cell in transformed_cells:
            writer.writerow(cell)


def plot_map_modified2(file_path, step_length=1):
    # Initialize lists to hold the x and y coordinates
    x_coords = []
    y_coords = []
    # Initialize a set to hold the coordinates of the cells through which lines pass
    cells_with_lines = set()

    # Open and read the file
    with open(file_path, 'r') as file:
        for line in file:
            # Split the line into coordinates and convert them to floats
            coords = line.strip().split(',')
            x1, y1, x2, y2 = map(float, coords)

            # Append the start and end coordinates for plotting
            x_coords.extend([x1, x2, None])  # None to separate lines
            y_coords.extend([y1, y2, None])

            # For each line, determine the cells it passes through or touches
            min_x, max_x = min(x1, x2), max(x1, x2)
            min_y, max_y = min(y1, y2), max(y1, y2)

            # Iterate over cells covered by the line's bounding box
            for cell_x in np.arange(np.floor(min_x / step_length) * step_length, np.ceil(max_x / step_length) * step_length, step_length):
                for cell_y in np.arange(np.floor(min_y / step_length) * step_length, np.ceil(max_y / step_length) * step_length, step_length):
                    cells_with_lines.add((cell_x, cell_y))

    # Plotting
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(x_coords, y_coords, linestyle='-', color='blue')

    # Fill the cells with lines in yellow
    for cell_x, cell_y in cells_with_lines:
        # print(f"Cell: ({cell_x}, {cell_y})")
        ax.add_patch(plt.Rectangle((cell_x, cell_y), step_length, step_length, color='yellow', alpha=0.3))

    transformed_cells = transform_cells(cells_with_lines, grid_size=step_length)

    # Setting a grid
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.grid(True, which='both', color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
    ax.set_aspect('equal', 'box')

    # Major ticks every 2, minor ticks every 1
    major_ticks = np.arange(-50, 51, step_length)
    ax.set_xticks(major_ticks)
    ax.set_yticks(major_ticks)

    # And a corresponding grid
    ax.grid(which='both', color='grey', linestyle='-', linewidth=0.5, alpha=0.3)

    # Hide markers by setting no tick labels
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    # Move axes to the center and hide the right and top lines
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('zero')
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')

    # Draw arrows
    ax.quiver(0, 0, 5, 0, angles='xy', scale_units='xy', scale=1, color='r', width=0.01)
    ax.quiver(0, 0, 0, 5, angles='xy', scale_units='xy', scale=1, color='g', width=0.01)

    # Define the diagonal connection plotting
    alpha_value = 0.15  # Transparency of the diagonal lines
    for x in np.arange(-50, 51, step_length):  # Ensure covering the edge by going one step beyond
        for y in np.arange(-50, 51, step_length):
            # Coordinates of the diagonal neighbors
            diagonals = [(x + step_length, y + step_length), (x - step_length, y + step_length), (x + step_length, y - step_length), (x - step_length, y - step_length)]
            for dx, dy in diagonals:
                # Check if the diagonal point is within the grid bounds
                if -50 <= dx <= 50 and -50 <= dy <= 50:
                    ax.plot([x, dx], [y, dy], linestyle='-', color='red', linewidth=0.5, alpha=alpha_value)

    # Displaying the plot
    plt.show()


# # Call the function with the path to your .vectormap.txt file
# plot_map_modified2('/home/dynamo/AMRL_Research/repos/amrl_maps/GDC1/GDC1.vectormap.txt', step_length=1)

save_walls_to_csv('/home/dynamo/AMRL_Research/repos/amrl_maps/GDC3/GDC3.vectormap.txt', 'src/navigation/GDC3_walls.csv', step_length=0.5, extra_cells=0)  # 200 x 200
# plot_path_on_map('/home/dynamo/AMRL_Research/repos/amrl_maps/GDC1/GDC1.vectormap.txt', 'src/navigation/GDC1_path.csv', 'src/navigation/GDC1_walls.csv', step_length=0.5)
