import math
import pygame
import sys

start = (0, 0)
goal = (60, 60)

ROWS = 60
COLS = 60
CELL_SIZE = 10

WIDTH = COLS * CELL_SIZE
HEIGHT = ROWS * CELL_SIZE
INFO_HEIGHT = 100

WHITE = (240, 240, 240)
BLACK = (20, 20, 20)
GRAY = (180, 180, 180)
BLUE = (70, 130, 255)
RED = (220, 80, 80)
GREEN = (70, 200, 120)
YELLOW = (240, 220, 80)


class Robot:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def update_pose(self, v, w, dt):
        self.x = self.x + v * math.cos(self.theta) * dt
        self.y = self.y + v * math.sin(self.theta) * dt
        self.theta = math.remainder(self.theta + w * dt, 2 * math.pi)


def world_to_screen(x, y):
    screen_x = int(x * CELL_SIZE)
    screen_y = int(y * CELL_SIZE)
    return screen_x, screen_y


def draw_grid(screen):
    for x in range(0, WIDTH + 1, CELL_SIZE):
        pygame.draw.line(screen, GRAY, (x, 0), (x, HEIGHT), 1)

    for y in range(0, HEIGHT + 1, CELL_SIZE):
        pygame.draw.line(screen, GRAY, (0, y), (WIDTH, y), 1)


def draw_text(screen, font, text, x, y, color=WHITE):
    surface = font.render(text, True, color)
    screen.blit(surface, (x, y))


def draw_robot(screen, robot):
    robot_screen = world_to_screen(robot.x, robot.y)

    pygame.draw.circle(screen, BLUE, robot_screen, 6)

    heading_length = 18
    heading_x = robot_screen[0] + heading_length * math.cos(robot.theta)
    heading_y = robot_screen[1] + heading_length * math.sin(robot.theta)

    pygame.draw.line(screen, BLACK, robot_screen, (heading_x, heading_y), 3)


def draw_goal(screen, goal):
    gx, gy = goal
    pygame.draw.circle(screen, RED, world_to_screen(gx, gy), 7)


def draw_trail(screen, trail):
    for point in trail:
        pygame.draw.circle(screen, YELLOW, world_to_screen(point[0], point[1]), 2)


def run():
    pygame.init()

    screen = pygame.display.set_mode((WIDTH, HEIGHT + INFO_HEIGHT))
    pygame.display.set_caption("Continuous Robot Pose Test")

    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    x, y = start
    X, Y = goal

    robot = Robot(x, y, 0)

    v = 8
    w = 2
    dt = 0.05
    tolerance = 1

    trail = []
    running = True
    paused = False
    finished = False

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_SPACE:
                    paused = not paused

                elif event.key == pygame.K_r:
                    robot = Robot(start[0], start[1], 0)
                    trail = []
                    finished = False
                    paused = False

        distance = math.sqrt((robot.x - X) ** 2 + (robot.y - Y) ** 2)
        desired_theta = math.atan2(Y - robot.y, X - robot.x)
        error = math.remainder(desired_theta - robot.theta, 2 * math.pi)

        action = "done"

        if not paused and not finished:
            if distance < tolerance:
                finished = True
            else:
                if error > 0.3:
                    action = "turn left"
                    robot.update_pose(0, w, dt)
                elif error < -0.3:
                    action = "turn right"
                    robot.update_pose(0, -w, dt)
                else:
                    action = "forward"
                    robot.update_pose(v, 0, dt)

                trail.append((robot.x, robot.y))

        screen.fill(WHITE)

        draw_grid(screen)
        draw_trail(screen, trail)
        draw_goal(screen, goal)
        draw_robot(screen, robot)

        pygame.draw.rect(screen, BLACK, (0, HEIGHT, WIDTH, INFO_HEIGHT))

        mode = "PAUSED" if paused else "RUNNING"
        if finished:
            mode = "FINISHED"

        draw_text(screen, font, f"Mode: {mode}", 10, HEIGHT + 10)
        draw_text(screen, font, f"Action: {action}", 10, HEIGHT + 35)
        draw_text(screen, font, f"Distance: {distance:.2f}", 180, HEIGHT + 10)
        draw_text(screen, font, f"Heading error: {error:.2f}", 180, HEIGHT + 35)
        draw_text(screen, font, "SPACE pause | R reset | ESC quit", 10, HEIGHT + 65, GRAY)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()


run()