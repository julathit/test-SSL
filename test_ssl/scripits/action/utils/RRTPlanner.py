import random
import math
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def is_collision_free(node, obstacle_coords, obstacle_radius):
    for ox, oy in obstacle_coords:
        if distance(node, Node(ox, oy)) <= obstacle_radius:
            return False
    return True

def get_nearest_node(tree, random_node):
    nearest_node = tree[0]
    min_dist = distance(nearest_node, random_node)
    for node in tree:
        dist = distance(node, random_node)
        if dist < min_dist:
            nearest_node = node
            min_dist = dist
    return nearest_node

def steer(from_node, to_node, step_size):
    angle = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = from_node.x + step_size * math.cos(angle)
    new_y = from_node.y + step_size * math.sin(angle)
    new_node = Node(new_x, new_y)
    new_node.parent = from_node
    return new_node

def generate_path(node):
    path = []
    while node is not None:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

def rrt(start, goal, obstacle_coords, obstacle_radius, x_bounds, y_bounds, max_iters, step_size):
    tree = [Node(start[0], start[1])]
    goal_node = Node(goal[0], goal[1])

    for _ in range(max_iters):
        random_x = random.uniform(x_bounds[0], x_bounds[1])
        random_y = random.uniform(y_bounds[0], y_bounds[1])
        random_node = Node(random_x, random_y)

        nearest_node = get_nearest_node(tree, random_node)
        new_node = steer(nearest_node, random_node, step_size)

        if is_collision_free(new_node, obstacle_coords, obstacle_radius):
            tree.append(new_node)

            if distance(new_node, goal_node) <= step_size:
                goal_node.parent = new_node
                tree.append(goal_node)
                return generate_path(goal_node)

    return None  # No path found

# start = (0, 0)
# goal = (9, 9)
# obstacle_coords = [(3, 3), (5, 5), (7, 7)]
# obstacle_radius = 0.8
# x_bounds = (0, 10)
# y_bounds = (0, 10)
# max_iters = 10000
# step_size = 0.05

# path = rrt(start, goal, obstacle_coords, obstacle_radius, x_bounds, y_bounds, max_iters, step_size)