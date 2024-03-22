import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.patches import Circle

def plot_path_and_obstacles(path_file, obstacles):
    path_data = pd.read_csv(path_file, sep=" ", header=None, names=["x", "y"])
    fig, ax = plt.subplots()
    ax.plot(path_data["x"], path_data["y"], label="Path", color='blue', linewidth=2)

    for (ox, oy, radius) in obstacles:
        circle = Circle((ox, oy), radius, color='red', fill=False)
        ax.add_patch(circle)

    all_x = path_data["x"].tolist() + [o[0] for o in obstacles]
    all_y = path_data["y"].tolist() + [o[1] for o in obstacles]
    ax.set_xlim(min(all_x), max(all_x))
    ax.set_ylim(min(all_y), max(all_y))
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    ax.set_title('Path with Obstacles')

    ax.grid(True)

    ax.legend()
    plt.show()

obstacles = [
    (2.5, 2.5,1),   # Each object is (x, y, radius)
    (5, 4, 0.6),
    # (1, 0.8 ,0.3),    
    # (10,8,2),
]

plot_path_and_obstacles("path.txt", obstacles)
