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
                grid[row, col] = 1
            else:
                print(f"Warning: Obstacle coordinate ({row}, {col}) is out of bounds.")

    if circle_obstacles:
        for x, y, r in circle_obstacles:
            # Iterate over a square containing the circle.  This is much faster than checking every cell in the grid.
            for row in range(max(0, int(y - r)), min(rows, int(y + r + 1))):
                for col in range(max(0, int(x - r)), min(cols, int(x + r + 1))):
                    #  Equation of circle is (x-center_x)^2 + (y-center_y)^2 <= radius^2
                    if (col - x)**2 + (row - y)**2 <= r**2:
                        grid[row, col] = 1
    return grid

def heuristic(a, b):
    """
    Calculates the Manhattan distance between two points.

    Args:
        a (tuple): The start point (row, col).
        b (tuple): The end point (row, col).

    Returns:
        int: The Manhattan distance between the two points.
    """
    squr = lambda x: x**2
    return np.sqrt(squr(a[0] - b[0])+ squr(a[1] - b[1]))

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
    open_set = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}
    visited = {start}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == end:
            break

        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            neighbor_row, neighbor_col = current[0] + dr, current[1] + dc

            if 0 <= neighbor_row < rows and 0 <= neighbor_col < cols and grid[neighbor_row, neighbor_col] == 0:
                neighbor = (neighbor_row, neighbor_col)
                new_cost = cost_so_far[current] + 1

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, end)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current
                    visited.add(neighbor)

    if current != end:
        return None, visited

    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path, visited

def plot_grid_and_path(grid, path, start, end, visited, circle_obstacles=None):
    """
    Plots the grid, the path, the start and end points, visited nodes, and circular obstacles.

    Args:
        grid (numpy.ndarray): The grid.
        path (list): The path from start to end.
        start (tuple): The start point.
        end (tuple): The end point.
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

    plt.xlabel('Column')
    plt.ylabel('Row')
    plt.title('A* Pathfinding on Grid')
    plt.legend()
    plt.show()

def SO2Foward(start: np.array,end: np.array,obstacle: np.array):
    start = np.array(start)
    end = np.array(end)
    obstacle = np.array(obstacle)

    delPat = start - end
    ang = np.arctan2((delPat[1])/(delPat[0]))

    #inverdRotation
    rotate = np.array([[np.cos(ang),np.sin(ang)]
                       [-np.sin(ang), np.cos(ang)]])
    startNew = (0,0)
    endNew = np.matmul(rotate,(end - start))
    xy_coordinates = obstacle[:,:2]

    tranformed_xy = np.dot(rotate, xy_coordinates.T - end-start).T

    obstacleNew = np.hstack((tranformed_xy,circle_obstacles[:,2:]))

    grid = create_grid(rows, cols, circle_obstacles=list(obstacleNew)) 
    path, visited = astar(list(grid),list(startNew),list(endNew))

    return grid,path,start,end,visited, circle_obstacles

if __name__ == "__main__":
    rows = int(45*2)
    cols = int(60*2)
    start = (0, 0)
    end = (50, 100)

    # Example with circular obstacles
    circle_obstacles = [[20, 20, 10], [40, 30, 10], [10, 40, 10]]  # [x, y, radius]
    grid = create_grid(rows, cols, circle_obstacles=circle_obstacles) # Pass the circular obstacles

    print(f"Grid size: {rows}x{cols}")
    print(f"Start: {start}, End: {end}")
    print(type(grid))

    start_time = time.time()
    path, visited = astar(grid, start, end)
    end_time = time.time()

    print(f"A* execution time: {end_time - start_time:.2f} seconds")

    plot_grid_and_path(grid, path, start, end, visited, circle_obstacles) # Pass circle obstacles to plot
