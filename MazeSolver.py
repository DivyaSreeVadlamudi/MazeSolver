import heapq

class MazeSolver:
    def __init__(self, maze):
        self.maze = maze
        self.num_rows = len(maze)
        self.num_cols = len(maze[0])

    def is_within_bounds(self, x, y):
        return 0 <= x < self.num_rows and 0 <= y < self.num_cols

    def is_clear(self, x, y):
        return self.is_within_bounds(x, y) and self.maze[x][y] == 0

    def find_path(self, start, goal):
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        open_set = [(heuristic(start, goal), start)]  # Priority queue for A* search
        closed_set = set()  # Set to store visited nodes
        came_from = {}  # Dictionary to store parent nodes for path reconstruction

        # The A* algorithm loop
        while open_set:
            # Get the node with the lowest cost from the priority queue
            current_cost, current_node = heapq.heappop(open_set)

            # Check if the current node is the goal
            if current_node == goal:
                # If the goal is reached, reconstruct the path and return it
                return reconstruct_path(came_from, start, goal)

            # Mark the current node as visited by adding it to the closed set
            closed_set.add(current_node)

            # Explore neighbors of the current node
            for dx, dy in directions:
                neighbor = (current_node[0] + dx, current_node[1] + dy)

                # Check if the neighbor is not in the closed set and is a clear space
                if neighbor not in closed_set and self.is_clear(*neighbor):
                    # Calculate the cost to reach the neighbor
                    neighbor_cost = (
                        current_cost - heuristic(current_node, goal) + 1 + heuristic(neighbor, goal)
                    )

                    # Add the neighbor to the priority queue with its cost
                    heapq.heappush(open_set, (neighbor_cost, neighbor))

                    # Record the parent node for path reconstruction
                    came_from[neighbor] = current_node

        # If the loop completes without reaching the goal, return an empty path
        return []  # No path found

# Function to get heuristic value
def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# Function to reconstruct the path if goal node is reached
def reconstruct_path(came_from, start, goal):
    path = []
    current = goal

    while current != start:
        path.insert(0, current)
        current = came_from[current]

    path.insert(0, start)
    return path

# Function to read the maze from given text file
def read_maze_from_file(file_path):
    with open(file_path, 'r') as file:
        maze = [list(map(int, line.split())) for line in file.readlines()]
    return maze

# Reading the maze file
maze_file = "C:/Users/Divya Sree/Downloads/P1_Option1_Maze/maze.txt"
maze = [list(map(int, line.split())) for line in open(maze_file)]

# Get start and end points from user during run time
start_point = tuple(map(int, input("Enter start point (x y): ").split()))
end_point = tuple(map(int, input("Enter end point (x y): ").split()))

# Creating a MazeSolver instance
maze_solver = MazeSolver(maze)

# Finding the path by calling find_path function and check if it exists
path = maze_solver.find_path(start_point, end_point)

# Printing the result
if path:
    print("YES")
    print("Path:", path)
else:
    print("NO")
