import os
import random


def generate_complex_maze(width, height, start, goal):
    # Initialize maze with walls
    maze = [["#" for _ in range(2 * width + 1)] for _ in range(2 * height + 1)]

    # Function to add passages
    def add_passages(x, y):
        stack = [(x, y)]

        while stack:
            x, y = stack[-1]
            maze[2 * y + 1][2 * x + 1] = " "

            # Find unvisited neighbours
            neighbours = []
            for nx, ny in [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]:
                if 0 <= nx < width and 0 <= ny < height:
                    if maze[2 * ny + 1][2 * nx + 1] == "#":
                        neighbours.append((nx, ny))

            if neighbours:
                nx, ny = random.choice(neighbours)
                stack.append((nx, ny))

                # Remove wall between cells
                maze[y + ny + 1][x + nx + 1] = " "
            else:
                stack.pop()

    # Start adding passages
    add_passages(random.randint(0, width - 1), random.randint(0, height - 1))

    # Mark the start and goal
    maze[2 * start[1] + 1][2 * start[0] + 1] = "A"
    maze[2 * goal[1] + 1][2 * goal[0] + 1] = "B"

    return maze


def save_maze_to_file(maze, file_path):
    # Ensure the directory exists
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(file_path, "w") as file:
        for row in maze:
            file.write("".join(row) + "\n")


# Do not set a fixed random seed to ensure different mazes are generated
# random.seed()  # Removed for variability

maze_width, maze_height = 15, 10
maze_start, maze_goal = (0, 0), (maze_width - 1, maze_height - 1)

maze = generate_complex_maze(maze_width, maze_height, maze_start, maze_goal)

maze_path = r"/Users/isaac/Documents/a_star_nural/maze.txt"
save_maze_to_file(maze, maze_path)

print(f"Maze saved to {maze_path}")
