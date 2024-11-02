import pygame
import sys
import time
import heapq

# Step 1: Initialize Pygame
pygame.init()

# Set up the display
WIDTH, HEIGHT = 600, 600  # Set the window size to 600x600 pixels
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Autonomous Robot  Simulation")
~
# Set colors
WHITE = (255, 255, 255)
GRID_COLOR = (0, 255, 0)
DESTINATION_COLOR = (255, 0, 0)
OBSTACLE_COLOR = (255, 165, 0)

# Load the robot image
robot_image = pygame.image.load("robot.png")  # Replace with the actual path to your image
robot_image = pygame.transform.scale(robot_image, (60, 60))  # Resize image to fit the grid


# Step 2: Draw the warehouse grid
def draw_grid():
    cell_size = WIDTH // 10
    for x in range(0, WIDTH, cell_size):
        pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, HEIGHT), 2)
    for y in range(0, HEIGHT, cell_size):
        pygame.draw.line(screen, GRID_COLOR, (0, y), (WIDTH, y), 2)


# Step 3: Draw obstacles in the warehouse
obstacles = [(3, 4), (5, 5), (6, 3)]  # Sample obstacles


def draw_obstacles():
    for obs in obstacles:
        obs_x = obs[0] * (WIDTH // 10)
        obs_y = obs[1] * (HEIGHT // 10)
        pygame.draw.rect(screen, OBSTACLE_COLOR, (obs_x, obs_y, 60, 60))  # Adjust size for cell


# Step 4: Define PathNode and AutonomousRobot classes
class PathNode:
    def __init__(self, position):
        self.position = position
        self.g_cost = 0  # Cost from start to this node
        self.h_cost = 0  # Heuristic cost to goal
        self.f_cost = 0  # Total cost
        self.parent = None  # To trace path back

    def __lt__(self, other):
        return self.f_cost < other.f_cost


def calculate_heuristic(start, goal):
    return abs(start[0] - goal[0]) + abs(start[1] - goal[1])


class AutonomousRobot:
    def __init__(self, start_x, start_y):
        self.start_position = (start_x, start_y)
        self.destination_position = (7, 9)
        self.path = []
        self.current_step = 0
        self.stopped = False
        self.stop_start_time = 0
        self.move_interval = 0.1
        self.stop_interval = 2
        self.last_move_time = time.time()

    def find_neighbors(self, node):
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        for direction in directions:
            neighbor_pos = (node.position[0] + direction[0], node.position[1] + direction[1])
            if 0 <= neighbor_pos[0] < 10 and 0 <= neighbor_pos[1] < 10 and neighbor_pos not in obstacles:
                neighbors.append(PathNode(neighbor_pos))
        return neighbors

    def find_path_with_a_star(self):
        start_node = PathNode(self.start_position)
        goal_node = PathNode(self.destination_position)

        open_set = []
        closed_set = set()
        heapq.heappush(open_set, start_node)

        while open_set:
            current_node = heapq.heappop(open_set)

            if current_node.position == goal_node.position:
                while current_node:
                    self.path.append(current_node.position)
                    current_node = current_node.parent
                self.path.reverse()
                return

            closed_set.add(current_node.position)

            for neighbor in self.find_neighbors(current_node):
                if neighbor.position in closed_set:
                    continue

                neighbor.g_cost = current_node.g_cost + 1
                neighbor.h_cost = calculate_heuristic(neighbor.position, goal_node.position)
                neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                neighbor.parent = current_node

                if any(neighbor.position == node.position and neighbor.g_cost >= node.g_cost for node in open_set):
                    continue

                heapq.heappush(open_set, neighbor)


# Create a robot instance
robot = AutonomousRobot(0, 0)
robot.find_path_with_a_star()

# Step 5: Main simulation loop
clock = pygame.time.Clock()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Draw the warehouse and obstacles every frame
    screen.fill(WHITE)
    draw_grid()
    draw_obstacles()

    # Draw the destination
    dest_x = robot.destination_position[0] * (WIDTH // 10)
    dest_y = robot.destination_position[1] * (HEIGHT // 10)
    pygame.draw.circle(screen, DESTINATION_COLOR, (dest_x + 30, dest_y + 30), 10)

    # Update and draw the robot
    current_time = time.time()
    if robot.current_step < len(robot.path):
        robot_pos = robot.path[robot.current_step]
        robot_x = robot_pos[0] * (WIDTH // 10)
        robot_y = robot_pos[1] * (HEIGHT // 10)

        # Draw the robot image at the current position
        screen.blit(robot_image, (robot_x, robot_y))

        if not robot.stopped:
            if current_time - robot.last_move_time >= robot.move_interval:
                robot.current_step += 1
                robot.stopped = True
                robot.stop_start_time = current_time
                robot.last_move_time = current_time
        else:
            if current_time - robot.stop_start_time >= robot.stop_interval:
                robot.stopped = False

    pygame.display.flip()
    clock.tick(60)  # Control the frame rate to 60 FPS
