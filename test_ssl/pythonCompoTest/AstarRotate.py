import numpy as np
import matplotlib.pyplot as plt
import heapq
import time

def create_grid(rows, cols, obstacle_list=None, circle_obstacles=None):
    """
    Creates a grid with obstacles, including circular obstacles.

    Args:
        rows (int): Number of rows in the grid.
        cols (int): Number of columns in the grid.
        obstacle_list (list, optional): A list of (row, col) tuples representing the coordinates of rectangular obstacles.
            If None, no rectangular obstacles are added. Defaults to None.
        circle_obstacles (list, optional): A list of [x, y, r] lists, where x and y are the center coordinates
            and r is the radius of the circular obstacles. If None, no circular obstacles are added.
            Defaults to None.

    Returns:
        numpy.ndarray: A 2D NumPy array representing the grid, where 0 is a free path and 1 is an obstacle.
    """
    grid = np.zeros((rows, cols), dtype=np.int8)

    if obstacle_list:
        for row, col in obstacle_list:
            if 0 <= row < rows and 0 <= col < cols:
                grid[int(row), int(col)] = 1
            else:
                print(f"Warning: Obstacle coordinate ({row}, {col}) is out of bounds.")

    if circle_obstacles:
        for x, y, r in circle_obstacles:
            # Iterate over a square containing the circle.  This is much faster than checking every cell in the grid.
            for row in range(max(0, int(y - r - 1)), min(rows, int(y + r + 2))):
                for col in range(max(0, int(x - r - 1)), min(cols, int(x + r + 2))):
                    #  Equation of circle is (x-center_x)^2 + (y-center_y)^2 <= radius^2
                    if (col - x)**2 + (row - y)**2 <= r**2:
                        grid[int(row), int(col)] = 1
    return grid

def heuristic(a, b):
    """
    Calculates the Euclidean distance between two points.

    Args:
        a (tuple): The start point (row, col).
        b (tuple): The end point (row, col).

    Returns:
        float: The Euclidean distance between the two points.
    """
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def astar(grid, start, end):
    """
    Implements the A* pathfinding algorithm on a grid.

    Args:
        grid (numpy.ndarray): A 2D NumPy array representing the grid.
        start (tuple): The start point (row, col).
        end (tuple): The end point (row, col).

    Returns:
        tuple: A tuple containing:
            - path (list): The path from start to end as a list of (row, col) tuples, or None if no path is found.
            - visited (set): The set of visited nodes.
    """
    rows, cols = grid.shape
    start_tuple = (int(start[0]), int(start[1]))
    end_tuple = (int(end[0]), int(end[1]))

    if not (0 <= start_tuple[0] < rows and 0 <= start_tuple[1] < cols and
            0 <= end_tuple[0] < rows and 0 <= end_tuple[1] < cols and
            grid[start_tuple] == 0 and grid[end_tuple] == 0):
        print("Error: Start or end point is out of bounds or an obstacle.")
        return None, set()

    open_set = [(0, start_tuple)]
    came_from = {}
    cost_so_far = {start_tuple: 0}
    visited = {start_tuple}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == end_tuple:
            break

        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]: # 8-connectivity
            neighbor_row, neighbor_col = current[0] + dr, current[1] + dc

            if 0 <= neighbor_row < rows and 0 <= neighbor_col < cols and grid[neighbor_row, neighbor_col] == 0:
                neighbor = (neighbor_row, neighbor_col)
                new_cost = cost_so_far[current] + np.sqrt(dr**2 + dc**2) # Diagonal moves have a cost of sqrt(2)

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, end_tuple)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current
                    visited.add(neighbor)

    if current != end_tuple:
        return None, visited

    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(start_tuple)
    path.reverse()
    return path, visited

def plot_grid_and_path(grid, path, start, end, visited, circle_obstacles=None):
    """
    Plots the grid, the path, the start and end points, visited nodes, and circular obstacles.

    Args:
        grid (numpy.ndarray): The grid.
        path (list): The path from start to end.
        start (tuple or np.ndarray): The start point.
        end (tuple or np.ndarray): The end point.
        visited (set): The set of visited nodes.
        circle_obstacles (list, optional):  A list of [x, y, r] lists representing circular obstacles.
    """
    rows, cols = grid.shape
    plt.figure(figsize=(8, 6))
    plt.imshow(grid, cmap='binary', origin='upper')

    # Plot visited nodes
    visited_coords = np.array(list(visited))
    if visited_coords.size > 0:
        plt.scatter(visited_coords[:, 1], visited_coords[:, 0], s=1, c='lightblue', marker='o', label='Visited')

    # Plot the path
    if path:
        path_coords = np.array(path)
        plt.plot(path_coords[:, 1], path_coords[:, 0], 'y-', linewidth=1, label='Path')
        plt.scatter(start[1], start[0], c='g', marker='o', s=50, label='Start')
        plt.scatter(end[1], end[0], c='r', marker='o', s=50, label='End')
    else:
        plt.scatter(start[1], start[0], c='g', marker='o', s=50, label='Start')
        plt.scatter(end[1], end[0], c='r', marker='o', s=50, label='End')
        print("No path found!")

    # Plot circular obstacles
    if circle_obstacles:
        for x, y, r in circle_obstacles:
            circle = plt.Circle((x, y), r, color='r', fill=True, alpha=0.5)  #Added alpha
            plt.gca().add_patch(circle)

    plt.xlim(-0.5, cols - 0.5)
    plt.ylim(rows - 0.5, -0.5) # Invert y-axis for correct orientation
    plt.xlabel('Column')
    plt.ylabel('Row')
    plt.title('A* Pathfinding on Grid')
    plt.legend()
    plt.grid(True, which='major', linestyle='-', alpha=0.5)
    plt.show()

def SO2Foward(start: np.ndarray, end: np.ndarray, obstacle: np.ndarray):
    """
    Transforms the problem into a new coordinate frame aligned with the start and end points
    and then performs A* pathfinding.

    Args:
        start (np.ndarray): The start coordinates [x, y].
        end (np.ndarray): The end coordinates [x, y].
        obstacle (np.ndarray): Array of circular obstacles [x, y, radius].

    Returns:
        tuple: Grid, path, original start, original end, visited nodes, original obstacles.
    """
    rows = int(45*1.5)
    cols = int(60*1.5)

    delta = end - start
    angle = np.arctan2(delta[1], delta[0])

    # Rotation matrix for the inverse transformation
    rotate_inv = np.array([[np.cos(-angle), -np.sin(-angle)],
                           [np.sin(-angle), np.cos(-angle)]])

    # Transform start and end to the new frame
    start_new = (0, 0)
    end_new_rotated = np.dot(rotate_inv, (end - start).T)
    end_new = np.array([end_new_rotated[0], 0]) # Align end with the x-axis

    # Transform obstacles to the new frame
    transformed_centers = np.dot(rotate_inv, (obstacle[:, :2] - start).T).T
    obstacle_new = np.hstack((transformed_centers, obstacle[:, 2:]))

    # Create the grid in the transformed frame
    grid_new = create_grid(rows, cols, circle_obstacles=list(obstacle_new))


    # Perform A* in the transformed frame
    path_new, visited_new = astar(grid_new, start_new, end_new)

   

    return grid_new, path_new, start, end, visited_new, obstacle

def SO2Backward(start: np.ndarray, end: np.ndarray, obstacle: np.ndarray):
    # This function would implement the same logic as SO2Forward but with the roles of start and end reversed.
    # It's left as an exercise for the user.
    pass

if __name__ == "__main__":
    rows = int(45*1.5)
    cols = int(60*1.5)
    start = np.array([10, 10])
    end = np.array([50, 40])

    # Example with circular obstacles
    circle_obstacles = np.array([[30, 25, 10], [45, 35, 10], [15, 40, 10]])  # [x, y, radius]

    grid, path, original_start, original_end, visited, original_obstacles = SO2Foward(start, end, circle_obstacles)
    plot_grid_and_path(grid, None, np.array([grid.shape[0] // 2, 0]), np.array([grid.shape[0] // 2, grid.shape[1] * (np.linalg.norm(end - start) / (60 * 1.5))]), visited, circle_obstacles=None) # Plot in transformed frame

    # Plot in the original frame (requires transforming the grid or replotting)
    if path:
        rows_orig = int(45*1.5)
        cols_orig = int(60*1.5)
        grid_orig = create_grid(rows_orig, cols_orig, circle_obstacles=list(original_obstacles))
        plot_grid_and_path(grid_orig, path, original_start, original_end, visited, circle_obstacles=list(original_obstacles))
    else:
        rows_orig = int(45*1.5)
        cols_orig = int(60*1.5)
        grid_orig = create_grid(rows_orig, cols_orig, circle_obstacles=list(original_obstacles))
        plot_grid_and_path(grid_orig, None, original_start, original_end, visited, circle_obstacles=list(original_obstacles))