import random
import heapq
import pygame
import sys


# ----------------------------
# Config
# ----------------------------

ROWS = 60
COLS = 60
CELL_SIZE = 8

WIDTH = COLS * CELL_SIZE
HEIGHT = ROWS * CELL_SIZE
INFO_HEIGHT = 140

START = (0, 0)
GOAL = (59, 59)

EMPTY = "."
OBSTACLE = "#"

PLANNING = "PLANNING"
MOVING = "MOVING"
REPLANNING = "REPLANNING"
GOAL_REACHED = "GOAL_REACHED"
FAILED = "FAILED"

# Colors
WHITE = (240, 240, 240)
WHITE_DARK = (200, 200, 200)

BLACK = (20, 20, 20)
GRAY = (180, 180, 180)

DARK_GRAY = (60, 60, 60)
DARK_GRAY_HOVER = (35, 35, 35)

BLUE = (70, 130, 255)
BLUE_HOVER = (40, 95, 210)

GREEN = (70, 200, 120)
GREEN_HOVER = (40, 160, 90)

RED = (220, 80, 80)
RED_HOVER = (180, 45, 45)

YELLOW = (240, 220, 80)
YELLOW_HOVER = (190, 170, 40)


ORANGE_DARK = (210, 90, 30)      
ORANGE = (255, 165, 60)          
ORANGE_LIGHT = (255, 210, 120)   

ORANGE_DARK_HOVER = (170, 60, 20)
ORANGE_HOVER = (210, 120, 30)
ORANGE_LIGHT_HOVER = (220, 170, 80)


# ----------------------------
# Grid / world
# ----------------------------

def create_grid():
    return [[EMPTY for _ in range(COLS)] for _ in range(ROWS)]

def create_costmap():
    return [[0 for _ in range(COLS)] for _ in range(ROWS)]


def set_obstacles(grid, count):
    used = {START, GOAL}
    placed = 0

    while placed < count:
        row = random.randint(0, ROWS - 1)
        col = random.randint(0, COLS - 1)

        if (row, col) not in used:
            grid[row][col] = OBSTACLE
            used.add((row, col))
            placed += 1


def setup_world(obstacle_count=200):
    grid = create_grid()
    set_obstacles(grid, obstacle_count)
    return grid


def get_neighbors(cell, grid):
    row, col = cell

    moves = [
        (row - 1, col),
        (row + 1, col),
        (row, col - 1),
        (row, col + 1),
    ]

    result = []

    for next_row, next_col in moves:
        inside_grid = 0 <= next_row < ROWS and 0 <= next_col < COLS

        if inside_grid and grid[next_row][next_col] != OBSTACLE:
            result.append((next_row, next_col))

    return result



# ----------------------------
# A* planner
# ----------------------------

def manhattan_distance(cell, goal):
    row1, col1 = cell
    row2, col2 = goal

    return abs(row1 - row2) + abs(col1 - col2)

def chebyshev_distance(cell, goal):
    row1, col1 = cell
    row2, col2 = goal

    return max(abs(row1 - row2), abs(col1 - col2))



def a_star(grid, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))

    prev = {start: None}
    cost = {start: 0}

    while frontier:
        _, curr = heapq.heappop(frontier)

        if curr == goal:
            return prev

        for neighbor in get_neighbors(curr, grid):
            new_cost = cost[curr] + 1

            if neighbor not in cost or new_cost < cost[neighbor]:
                cost[neighbor] = new_cost
                priority = new_cost + manhattan_distance(neighbor, goal)
                heapq.heappush(frontier, (priority, neighbor))
                prev[neighbor] = curr

    return prev

def a_star_costmap(grid, costmap, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))

    prev = {start: None}
    cost = {start: 0}

    while frontier:
        _, curr = heapq.heappop(frontier)

        if curr == goal:
            return prev

        for neighbor in get_neighbors(curr, grid):
            row,col = neighbor
            penalty = costmap[row][col]/25

            new_cost = cost[curr] + 1 + penalty

            if neighbor not in cost or new_cost < cost[neighbor]:
                cost[neighbor] = new_cost
                priority = new_cost + manhattan_distance(neighbor, goal)
                heapq.heappush(frontier, (priority, neighbor))
                prev[neighbor] = curr

    return prev


def get_path(prev, goal):
    if goal not in prev:
        return None

    curr = goal
    path = []

    while curr is not None:
        path.append(curr)
        curr = prev[curr]

    path.reverse()
    return path


def plan_path(grid, start, goal):
    costmap = get_costmap(grid, 3)
    # prev = a_star(grid, start, goal)
    prev = a_star_costmap(grid, costmap, start, goal)
    return get_path(prev, goal)


# ----------------------------
# Replanning helpers
# ----------------------------

def is_path_blocked(path, grid, index):
    if path is None:
        return True

    future_path = path[index:]

    for row, col in future_path:
        if grid[row][col] == OBSTACLE:
            return True

    return False


def add_obstacle(grid, cell):
    row, col = cell
    grid[row][col] = OBSTACLE


def remove_obstacle(grid, cell):
    row, col = cell
    grid[row][col] = EMPTY


def get_cell_from_mouse(mouse_pos):
    mouse_x, mouse_y = mouse_pos

    col = mouse_x // CELL_SIZE
    row = mouse_y // CELL_SIZE

    if 0 <= row < ROWS and 0 <= col < COLS:
        return row, col

    return None



def draw_text(screen, font, text, x, y, color=WHITE):
    surface = font.render(text, True, color)
    screen.blit(surface, (x, y))


def cell_rect(row, col):
    x = col * CELL_SIZE
    y = row * CELL_SIZE
    return pygame.Rect(x, y, CELL_SIZE, CELL_SIZE)


def darken_color(color):
    if color == WHITE:
        return WHITE_DARK
    if color == DARK_GRAY:
        return DARK_GRAY_HOVER
    if color == YELLOW:
        return YELLOW_HOVER
    if color == BLUE:
        return BLUE_HOVER
    if color == GREEN:
        return GREEN_HOVER
    if color == RED:
        return RED_HOVER
    if color == ORANGE:
        return ORANGE_HOVER
    if color == ORANGE_DARK:
        return ORANGE_DARK_HOVER
    if color == ORANGE_LIGHT:
        return ORANGE_LIGHT_HOVER

    return color

def get_costmap(grid, radius):
    costmap = create_costmap()
    for row in range(ROWS):
        for col in range(COLS): 
            cell = (row,col)
            if grid[row][col] == '#':
                costmap[row][col] = 100
                for r in range(row - radius, row + radius + 1):
                    for c in range(col - radius, col + radius + 1):
                        if r < 0 or r >= ROWS or c < 0 or c >= COLS:
                            continue
                        if grid[r][c] != '#' and (r,c) != START and (r,c) != GOAL:
                            distance = chebyshev_distance(cell, (r,c))
                            if distance == 1:
                                costmap[r][c] = max(75, costmap[r][c])
                            elif distance == 2:
                                costmap[r][c] = max(50, costmap[r][c])
                            elif distance == 3:
                                costmap[r][c] = max(25, costmap[r][c])
    return costmap
    


def get_cell_color(grid, costmap, path, robot_pos, cell, show_costmap):
    row, col = cell

    color = WHITE

    # Costmap visualization only
    if show_costmap:
        if costmap[row][col] == 75:
            color = ORANGE_DARK
        if costmap[row][col] == 50:
            color = ORANGE
        if costmap[row][col] == 25:
            color = ORANGE_LIGHT

        if costmap[row][col] == 100:
            color = DARK_GRAY

    if path is not None and cell in path:
        color = YELLOW

    if grid[row][col] == OBSTACLE:
        color = DARK_GRAY

    if cell == START:
        color = GREEN

    if cell == GOAL:
        color = RED

    if cell == robot_pos:
        color = BLUE

    return color


def draw_world(screen, grid, costmap, path, robot_pos, font, state, metrics, paused, hover_cell, show_costmap):
    screen.fill(BLACK)

    for row in range(ROWS):
        for col in range(COLS):
            cell = (row, col)
            rect = cell_rect(row, col)

            color = get_cell_color(grid, costmap, path, robot_pos, cell, show_costmap)

            if hover_cell == cell:
                color = darken_color(color)

            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, GRAY, rect, 1)

    info_y = HEIGHT
    pygame.draw.rect(screen, BLACK, (0, HEIGHT, WIDTH, INFO_HEIGHT))

    pause_text = "PAUSED" if paused else "RUNNING"
    costmap_text = "ON" if show_costmap else "OFF"

    draw_text(screen, font, f"State: {state}", 10, info_y + 10)
    draw_text(screen, font, f"Mode: {pause_text}", 230, info_y + 10)

    draw_text(screen, font, f"Steps: {metrics['steps']}", 10, info_y + 35)
    draw_text(screen, font, f"Replans: {metrics['replans']}", 140, info_y + 35)

    draw_text(screen, font, f"Obstacle Changes: {metrics['obstacle_changes']}", 10, info_y + 60)

    if path is not None:
        draw_text(screen, font, f"Path Length: {len(path)}", 230, info_y + 60)
    else:
        draw_text(screen, font, "Path Length: None", 230, info_y + 60)

    draw_text(screen, font, f"Costmap: {costmap_text} | C toggle", 10, info_y + 90, GRAY)
    draw_text(screen, font, "SPACE start/pause | Left click obstacle | R reset | ESC quit", 10, info_y + 115, GRAY)



# ----------------------------
# Simulator
# ----------------------------

def create_metrics():
    return {
        "goal_reached": False,
        "steps": 0,
        "replans": 0,
        "obstacle_changes": 0,
    }


def reset_simulation():
    grid = setup_world(obstacle_count=150)
    robot_pos = START
    path = None
    curr_i = 0
    state = PLANNING
    metrics = create_metrics()

    return grid, robot_pos, path, curr_i, state, metrics


def handle_obstacle_click(grid, cell, robot_pos, metrics):
    if cell is None:
        return False

    if cell == START or cell == GOAL or cell == robot_pos:
        return False

    row, col = cell

    if grid[row][col] == OBSTACLE:
        remove_obstacle(grid, cell)
    else:
        add_obstacle(grid, cell)

    metrics["obstacle_changes"] += 1
    return True


def main():
    pygame.init()

    screen = pygame.display.set_mode((WIDTH, HEIGHT + INFO_HEIGHT))
    pygame.display.set_caption("A* Navigation Simulator")

    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    grid, robot_pos, path, curr_i, state, metrics = reset_simulation()

    move_delay_ms = 100
    last_move_time = pygame.time.get_ticks()

    running = True
    paused = True
    show_costmap = False

    while running:
        current_time = pygame.time.get_ticks()
        hover_cell = get_cell_from_mouse(pygame.mouse.get_pos())

        # ----------------------------
        # Events
        # ----------------------------

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_r:
                    grid, robot_pos, path, curr_i, state, metrics = reset_simulation()
                    paused = True
                    last_move_time = pygame.time.get_ticks()

                elif event.key == pygame.K_SPACE:
                    paused = not paused

                    # If failed and user presses SPACE, try planning again.
                    if not paused and state == FAILED:
                        state = PLANNING

                    # If paused in replanning, pressing SPACE should execute replanning.
                    last_move_time = pygame.time.get_ticks()

                elif event.key == pygame.K_c:
                    show_costmap = not show_costmap

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    clicked_cell = get_cell_from_mouse(event.pos)
                    changed = handle_obstacle_click(grid, clicked_cell, robot_pos, metrics)

                    if changed:
                        paused = True

                        if path is None:
                            state = PLANNING

                        elif state in [PLANNING, MOVING, REPLANNING]:
                            if is_path_blocked(path, grid, curr_i):
                                state = REPLANNING


        # ----------------------------
        # State machine update
        # ----------------------------

        if not paused and current_time - last_move_time >= move_delay_ms:
            last_move_time = current_time

            if state == PLANNING:
                path = plan_path(grid, robot_pos, GOAL)
                curr_i = 0

                if path is None:
                    state = FAILED
                else:
                    state = MOVING

            elif state == MOVING:
                robot_pos = path[curr_i]

                if robot_pos == GOAL:
                    state = GOAL_REACHED
                    metrics["goal_reached"] = True

                else:
                    if is_path_blocked(path, grid, curr_i):
                        state = REPLANNING
                    else:
                        curr_i += 1
                        metrics["steps"] += 1

            elif state == REPLANNING:
                metrics["replans"] += 1

                path = plan_path(grid, robot_pos, GOAL)
                curr_i = 0

                if path is None:
                    state = FAILED
                else:
                    state = MOVING

            elif state in [GOAL_REACHED, FAILED]:
                pass

        costmap = get_costmap(grid, 3)

        draw_world(
            screen,
            grid,
            costmap,
            path,
            robot_pos,
            font,
            state,
            metrics,
            paused,
            hover_cell,
            show_costmap,
        )

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()


main()