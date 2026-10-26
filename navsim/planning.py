from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
import time

from .world import Cell, Point, World


@dataclass
class PlanResult:
    path: list[Point]
    expanded: int = 0
    elapsed_ms: float = 0.0
    cost: float = math.inf


class AStarPlanner:
    def __init__(self, world: World, robot_radius: float = 0.32):
        self.world, self.robot_radius = world, robot_radius
        self.blocked_cells: set[Cell] = set()
        self._static_blocked: set[Cell] = set()
        self._clearance: dict[Cell, float] = {}
        self._revision = -1
        self.nominal_speed = 1.6

    @staticmethod
    def _center(cell: Cell) -> Point:
        return cell[0] + 0.5, cell[1] + 0.5

    @staticmethod
    def _octile(first: Cell, second: Cell) -> float:
        dx, dy = abs(first[0] - second[0]), abs(first[1] - second[1])
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    def _refresh_static(self) -> None:
        if self._revision == self.world.revision:
            return
        self._static_blocked.clear()
        self._clearance.clear()
        for row in range(self.world.height):
            for col in range(self.world.width):
                cell, point = (col, row), (col + 0.5, row + 0.5)
                if not self.world.valid_point(point, self.robot_radius):
                    self._static_blocked.add(cell)
                    continue
                distance = min(point[0], point[1], self.world.width - point[0], self.world.height - point[1], 3.0)
                for wall_col, wall_row in self.world._nearby_static(point, point, 3.0):
                    dx = max(wall_col - point[0], 0.0, point[0] - wall_col - 1)
                    dy = max(wall_row - point[1], 0.0, point[1] - wall_row - 1)
                    distance = min(distance, math.hypot(dx, dy))
                self._clearance[cell] = max(0.01, distance - self.robot_radius)
        self._revision = self.world.revision

    def _dynamic_penalties(self, start: Point, horizon: float) -> dict[Cell, float]:
        penalties: dict[Cell, float] = {}
        for actor, trajectory in zip(self.world.obstacles, self.world.predictions(horizon)):
            influence = actor.radius + self.robot_radius + 1.4
            for timestamp, x, y in trajectory:
                for row in range(max(0, int(y - influence)), min(self.world.height, int(y + influence) + 1)):
                    for col in range(max(0, int(x - influence)), min(self.world.width, int(x + influence) + 1)):
                        distance = math.hypot(col + 0.5 - x, row + 0.5 - y)
                        if distance >= influence:
                            continue
                        arrival = math.hypot(col + 0.5 - start[0], row + 0.5 - start[1]) / self.nominal_speed


                        timing = 0.25 + 0.75 * math.exp(-abs(min(arrival, horizon) - timestamp))
                        penalty = 3.0 * timing * (1.0 - distance / influence) ** 2
                        cell = col, row
                        penalties[cell] = max(penalties.get(cell, 0.0), penalty)
        return penalties

    def _connectors(self, point: Point, from_start: bool) -> dict[Cell, float]:
        col, row = math.floor(point[0]), math.floor(point[1])
        connectors = {}
        for y in range(max(0, row - 1), min(self.world.height, row + 2)):
            for x in range(max(0, col - 1), min(self.world.width, col + 2)):
                cell, center = (x, y), (x + 0.5, y + 0.5)
                if cell in self.blocked_cells:
                    continue
                first, second = (point, center) if from_start else (center, point)
                if self.world.collision_free(first, second, self.robot_radius):
                    connectors[cell] = math.dist(point, center)
        return connectors

    def _timed_free(self, start: Point, end: Point, arrival: float, horizon: float) -> bool:

        duration = math.dist(start, end) / self.nominal_speed
        if arrival >= horizon or duration < 1e-8:
            return True
        checked_duration = min(duration, horizon - arrival)
        fraction = checked_duration / duration
        checked_end = (start[0] + (end[0] - start[0]) * fraction, start[1] + (end[1] - start[1]) * fraction)
        return self.world.collision_free(start, checked_end, self.robot_radius, checked_duration, arrival)

    def plan(self, start: Point, goal: Point, prediction_horizon: float = 2.0) -> PlanResult:
        started = time.perf_counter()
        expanded = 0

        def result(path=None, cost=math.inf):
            return PlanResult(path or [], expanded, (time.perf_counter() - started) * 1000.0, cost)

        if prediction_horizon < 0 or not self.world.collision_free(start, start, self.robot_radius) or not self.world.collision_free(goal, goal, self.robot_radius):
            return result()
        if math.dist(start, goal) < 1e-8:
            return result([start], 0.0)
        self._refresh_static()
        self.blocked_cells = set(self._static_blocked)
        for actor in self.world.obstacles:
            radius = actor.radius + self.robot_radius
            for row in range(max(0, int(actor.y - radius - 0.5)), min(self.world.height, int(actor.y + radius + 0.5) + 1)):
                for col in range(max(0, int(actor.x - radius - 0.5)), min(self.world.width, int(actor.x + radius + 0.5) + 1)):
                    if math.hypot(col + 0.5 - actor.x, row + 0.5 - actor.y) <= radius:
                        self.blocked_cells.add((col, row))
        if math.dist(start, goal) < 1.5 and self.world.collision_free(start, goal, self.robot_radius, math.dist(start, goal) / self.nominal_speed):
            return result([start, goal], math.dist(start, goal))
        start_links, goal_links = self._connectors(start, True), self._connectors(goal, False)
        if not start_links or not goal_links:
            return result()
        penalties = self._dynamic_penalties(start, prediction_horizon)
        tie = itertools.count()
        frontier = []
        costs: dict[Cell, float] = {}
        parents: dict[Cell, Cell | None] = {}
        lengths: dict[Cell, float] = {}
        closed: set[Cell] = set()

        def heuristic(cell: Cell) -> float:
            return min(self._octile(cell, destination) + distance for destination, distance in goal_links.items())

        for cell, distance in start_links.items():
            if not self._timed_free(start, self._center(cell), 0.0, prediction_horizon):
                continue
            cost = distance * (1.0 + 0.18 / (self._clearance[cell] + 0.2) + penalties.get(cell, 0.0))
            costs[cell], lengths[cell], parents[cell] = cost, distance, None
            heapq.heappush(frontier, (cost + heuristic(cell), next(tie), cost, cell))
        neighbors = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)))
        destination = None
        best_goal_cost = math.inf
        while frontier:
            estimate, _, queued_cost, cell = heapq.heappop(frontier)
            if estimate >= best_goal_cost:
                break
            if cell in closed or queued_cost > costs[cell] + 1e-9:
                continue
            closed.add(cell)
            expanded += 1
            if cell in goal_links:
                if self._timed_free(self._center(cell), goal, lengths[cell] / self.nominal_speed, prediction_horizon):
                    goal_cost = costs[cell] + goal_links[cell]
                    if goal_cost < best_goal_cost:
                        destination, best_goal_cost = cell, goal_cost
                continue
            center = self._center(cell)
            for dx, dy, distance in neighbors:
                neighbor = cell[0] + dx, cell[1] + dy
                if not (0 <= neighbor[0] < self.world.width and 0 <= neighbor[1] < self.world.height) or neighbor in self.blocked_cells or neighbor in closed:
                    continue
                if dx and dy and ((cell[0] + dx, cell[1]) in self.blocked_cells or (cell[0], cell[1] + dy) in self.blocked_cells):
                    continue
                candidate = self._center(neighbor)
                if not self.world.collision_free(center, candidate, self.robot_radius):
                    continue
                if not self._timed_free(center, candidate, lengths[cell] / self.nominal_speed, prediction_horizon):
                    continue
                edge_cost = distance * (1.0 + 0.18 / (self._clearance[neighbor] + 0.2) + penalties.get(neighbor, 0.0))
                candidate_cost = costs[cell] + edge_cost
                if candidate_cost + 1e-9 >= costs.get(neighbor, math.inf):
                    continue
                costs[neighbor] = candidate_cost
                lengths[neighbor] = lengths[cell] + distance
                parents[neighbor] = cell
                heapq.heappush(frontier, (candidate_cost + heuristic(neighbor), next(tie), candidate_cost, neighbor))
        if destination is None:
            return result()
        cells = []
        cursor = destination
        while cursor is not None:
            cells.append(cursor)
            cursor = parents[cursor]
        path = [start] + [self._center(cell) for cell in reversed(cells)] + [goal]
        path = [point for index, point in enumerate(path) if index == 0 or math.dist(point, path[index - 1]) > 1e-8]
        return result(self._smooth(path, prediction_horizon), best_goal_cost)

    def _smooth(self, path: list[Point], horizon: float) -> list[Point]:

        if len(path) < 3:
            return path
        smoothed = [path[0]]
        index, arrival = 0, 0.0
        while index < len(path) - 1:
            chosen = None

            for candidate in range(min(len(path) - 1, index + 8), index, -1):
                if not self.world.collision_free(path[index], path[candidate], self.robot_radius):
                    continue
                if candidate > index + 1 and not self.world.collision_free(path[index], path[candidate], self.robot_radius + 0.12):
                    continue
                if not self._timed_free(path[index], path[candidate], arrival, horizon):
                    continue
                chosen = candidate
                break
            if chosen is None:


                return path
            arrival += math.dist(path[index], path[chosen]) / self.nominal_speed
            smoothed.append(path[chosen])
            index = chosen
        return smoothed
