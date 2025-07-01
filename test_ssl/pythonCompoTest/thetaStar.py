import matplotlib.pyplot as plt
import random

# Define the environment (e.g., boundaries, obstacles)
min_x, max_x = 0, 100
min_y, max_y = 0, 100
obstacles = [(20, 20, 10), (70, 70, 5)]  # (x, y, radius)

# Define start and goal
start = (10, 10)
goal = (90, 90)

# List to store sampled nodes and edges of the tree
nodes = [start]
edges = []

# Simple sampling (replace with informed sampling in a full BIT*)
for _ in range(100):
    new_node = (random.uniform(min_x, max_x), random.uniform(min_y, max_y))
    nodes.append(new_node)
    # In a real BIT*, you'd connect this to the nearest valid neighbor
    if len(nodes) > 1:
        nearest = min(nodes[:-1], key=lambda n: (n[0] - new_node[0])**2 + (n[1] - new_node[1])**2)
        edges.append((nearest, new_node))

# Visualization using matplotlib
fig, ax = plt.subplots()

# Plot environment
ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)
for obs_x, obs_y, obs_r in obstacles:
    circle = plt.Circle((obs_x, obs_y), obs_r, color='gray')
    ax.add_patch(circle)

# Plot start and goal
ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
ax.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

# Plot the search tree (simple connection for illustration)
for u, v in edges:
    ax.plot([u[0], v[0]], [u[1], v[1]], 'b-', linewidth=0.5)

# Plot the sampled nodes
ax.plot([n[0] for n in nodes], [n[1] for n in nodes], 'k.', markersize=2)

ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_title("Simple Sampling (Illustrative - Not BIT*)")
ax.legend()
plt.grid(True)
plt.show()