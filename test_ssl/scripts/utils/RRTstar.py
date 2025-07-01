import numpy as np
import matplotlib.pyplot as plt
import random
import math
from component.misc import Position

class Node:
    """Represents a node in the RRT* tree."""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

def dist(node1, node2):
    """Calculates the Euclidean distance between two nodes."""
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def steer(from_node, to_coords, step_size):
    """Creates a new node by moving from from_node towards to_coords with a maximum step_size."""
    d = dist(from_node, Node(*to_coords))
    if d > step_size:
        ratio = step_size / d
        new_x = from_node.x + (to_coords[0] - from_node.x) * ratio
        new_y = from_node.y + (to_coords[1] - from_node.y) * ratio
        return Node(new_x, new_y)
    else:
        return Node(*to_coords)

def find_nearest(tree, sample):
    """Finds the nearest node in the tree to the given sample."""
    min_dist = float('inf')
    nearest_node = None
    for node in tree:
        d = dist(node, Node(*sample))
        if d < min_dist:
            min_dist = d
            nearest_node = node
    return nearest_node

def find_near(tree, new_node, radius):
    """Finds all nodes in the tree within a given radius of the new node."""
    near_nodes = []
    for node in tree:
        if dist(node, new_node) < radius:
            near_nodes.append(node)
    return near_nodes

def cost(node):
    """Calculates the cost to reach the given node from the start node."""
    current_node = node
    path_cost = 0.0
    while current_node.parent:
        path_cost += dist(current_node, current_node.parent)
        current_node = current_node.parent
    return path_cost

def rrt_star(start: Position, goal: Position, obstacle_list, bounds, num_iterations=1000, step_size=0.1, goal_tolerance=0.05, neighbor_radius=0.2):
    """RRT* algorithm implementation."""
    start_node = Node(*start.to_list())
    goal_node = Node(*goal.to_list())
    tree = [start_node]
    path = None

    for _ in range(num_iterations):
        # Sample a random point in the search space
        if random.random() < 0.1:  # Bias towards the goal
            sample = goal
        else:
            sample = [random.uniform(bounds[0], bounds[1]), random.uniform(bounds[2], bounds[3])]

        # Find the nearest node in the tree to the sample
        nearest_node = find_nearest(tree, sample)

        # Steer from the nearest node towards the sample
        new_node = steer(nearest_node, sample, step_size)

        # Check for collisions with obstacles
        if not is_collision(new_node, obstacle_list):
            # Find near neighbors
            near_nodes = find_near(tree, new_node, neighbor_radius)

            # Choose the parent with the minimum cost
            min_cost = cost(nearest_node) + dist(nearest_node, new_node)
            best_parent = nearest_node
            for near_node in near_nodes:
                new_cost = cost(near_node) + dist(near_node, new_node)
                if new_cost < min_cost:
                    min_cost = new_cost
                    best_parent = near_node

            new_node.parent = best_parent
            new_node.cost = min_cost
            tree.append(new_node)

            # Rewire the tree
            for near_node in near_nodes:
                if near_node != best_parent:
                    new_cost = cost(new_node) + dist(new_node, near_node)
                    if new_cost < cost(near_node):
                        near_node.parent = new_node
                        near_node.cost = new_cost

            # Check if the goal is reached
            if dist(new_node, goal_node) < goal_tolerance:
                goal_node.parent = new_node
                goal_node.cost = cost(new_node) + dist(new_node, goal_node)
                path = get_path(goal_node)
                break

    return tree, path

def is_collision(node, obstacle_list):
    """Checks if a node is in collision with any obstacles."""
    for obstacle in obstacle_list:
        if dist(node, Node(*obstacle[:2])) < obstacle[2]:  # Assuming obstacles are circles (x, y, radius)
            return True
    return False

def get_path(goal_node):
    """Retrieves the path from the start node to the goal node."""
    path = []
    current = goal_node
    while current:
        path.append((current.x, current.y))
        current = current.parent
    return path[::-1]  # Reverse the path to get it from start to goal

def plot_rrt_star(tree, path, start, goal, obstacle_list, bounds):
    """Plots the RRT* tree, path, obstacles, start, and goal."""
    plt.figure(figsize=(8, 8))
    plt.xlim(bounds[0], bounds[1])
    plt.ylim(bounds[2], bounds[3])

    # Plot the tree edges
    for node in tree:
        if node.parent:
            plt.plot([node.parent.x, node.x], [node.parent.y, node.y], 'g-', linewidth=0.5)

    # Plot the path
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, 'r-', linewidth=2)

    # Plot the obstacles
    for obstacle in obstacle_list:
        circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='k', fill=True)
        plt.gca().add_patch(circle)

    # Plot the start and goal
    plt.plot(start[0], start[1], 'bo', markersize=8, label='Start')
    plt.plot(goal[0], goal[1], 'go', markersize=8, label='Goal')

    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("RRT* Path Planning")
    plt.legend()
    plt.grid(True)
    plt.show()