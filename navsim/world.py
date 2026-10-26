from __future__ import annotations

from dataclasses import dataclass, field
import math
import random
from typing import Iterable

Point = tuple[float, float]
Cell = tuple[int, int]
_EPS = 1e-8


def _point_segment_distance_sq(point: Point, start: Point, end: Point) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    fraction = 0.0 if length_sq < _EPS else max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq))
    return (point[0] - start[0] - fraction * dx) ** 2 + (point[1] - start[1] - fraction * dy) ** 2


def _slab_hit(start: Point, delta: Point, bounds: tuple[float, float, float, float]):

    enter, leave = -math.inf, math.inf
    normal = (0.0, 0.0)
    for axis, (low, high) in enumerate(((bounds[0], bounds[2]), (bounds[1], bounds[3]))):
        origin, velocity = start[axis], delta[axis]
        if abs(velocity) < _EPS:
            if origin < low or origin > high:
                return None
            continue
        first, last = (low - origin) / velocity, (high - origin) / velocity
        direction = -1.0
        if first > last:
            first, last, direction = last, first, 1.0
        if first > enter + _EPS:
            enter = first
            normal = (direction, 0.0) if axis == 0 else (0.0, direction)
        elif abs(first - enter) <= _EPS:
            normal = (direction, normal[1]) if axis == 0 else (normal[0], direction)
        leave = min(leave, last)
        if enter > leave + _EPS:
            return None
    if leave < 0.0 or enter > 1.0 or enter < -_EPS:
        return None
    return max(0.0, enter), normal


def _segment_rectangle_distance_sq(start: Point, end: Point, col: int, row: int) -> float:
    bounds = (float(col), float(row), col + 1.0, row + 1.0)
    if col <= start[0] <= col + 1 and row <= start[1] <= row + 1:
        return 0.0
    if _slab_hit(start, (end[0] - start[0], end[1] - start[1]), bounds) is not None:
        return 0.0
    distance = math.inf
    for x, y in (start, end):
        dx = max(col - x, 0.0, x - col - 1)
        dy = max(row - y, 0.0, y - row - 1)
        distance = min(distance, dx * dx + dy * dy)
    for corner in ((col, row), (col + 1, row), (col, row + 1), (col + 1, row + 1)):
        distance = min(distance, _point_segment_distance_sq(corner, start, end))
    return distance


def _disk_rectangle_hit(start: Point, delta: Point, col: int, row: int, radius: float):

    hits = []
    for axis, face, lower, upper, direction in (
        (0, col - radius, row, row + 1, -1.0),
        (0, col + 1 + radius, row, row + 1, 1.0),
        (1, row - radius, col, col + 1, -1.0),
        (1, row + 1 + radius, col, col + 1, 1.0),
    ):
        if delta[axis] * direction >= -_EPS:
            continue
        fraction = (face - start[axis]) / delta[axis]
        along = start[1 - axis] + delta[1 - axis] * fraction
        if -_EPS <= fraction <= 1.0 and lower - _EPS <= along <= upper + _EPS:
            normal = (direction, 0.0) if axis == 0 else (0.0, direction)
            hits.append((max(0.0, fraction), normal))
    length_sq = delta[0] ** 2 + delta[1] ** 2
    if length_sq > _EPS and radius > 0:
        for corner_x, corner_y, x_sign, y_sign in ((col, row, -1, -1), (col + 1, row, 1, -1), (col, row + 1, -1, 1), (col + 1, row + 1, 1, 1)):
            px, py = start[0] - corner_x, start[1] - corner_y
            half_b = px * delta[0] + py * delta[1]
            discriminant = half_b * half_b - length_sq * (px * px + py * py - radius * radius)
            if discriminant < 0:
                continue
            fraction = (-half_b - math.sqrt(discriminant)) / length_sq
            if not -_EPS <= fraction <= 1.0:
                continue
            nx, ny = px + delta[0] * fraction, py + delta[1] * fraction
            if nx * x_sign < -_EPS or ny * y_sign < -_EPS or nx * delta[0] + ny * delta[1] >= _EPS:
                continue
            hits.append((max(0.0, fraction), (nx / radius, ny / radius)))
    return min(hits, key=lambda hit: hit[0]) if hits else None


@dataclass
class MovingObstacle:
    x: float
    y: float
    vx: float
    vy: float
    radius: float = 0.45
    _world: World | None = field(default=None, repr=False, compare=False, init=False)

    def predict(self, horizon: float, step: float = 0.25) -> list[tuple[float, float, float]]:

        if self._world is not None:
            return self._world._sample_motion(self, horizon, step)
        if horizon < 0 or step <= 0:
            raise ValueError("Prediction horizon must be nonnegative and step positive")
        times = [min(index * step, horizon) for index in range(math.ceil(horizon / step) + 1)]
        return [(time, self.x + self.vx * time, self.y + self.vy * time) for time in times]


class World:


    def __init__(self, width: int = 48, height: int = 36, scenario: str = "warehouse", seed: int = 7):
        if width < 8 or height < 8:
            raise ValueError("The world must be at least 8 by 8 cells")
        if scenario not in {"warehouse", "crossing", "open"}:
            raise ValueError(f"Unknown scenario: {scenario}")
        self.width, self.height = width, height
        self.scenario, self.seed = scenario, seed
        self.start: Point = (2.5, 2.5)
        self.goal: Point = (width - 2.5, height - 2.5)
        self.static: set[Cell] = set()
        self.obstacles: list[MovingObstacle] = []
        self.revision = 0
        self._forecast_cache: dict[float, list[list[tuple[float, float, float]]]] = {}
        self._forecast_signature: tuple = ()
        self._build_scenario(random.Random(seed))

    def _build_scenario(self, rng: random.Random) -> None:
        if self.scenario == "warehouse":

            for x_fraction in (0.14, 0.28, 0.42, 0.56, 0.70, 0.84):
                left = int(self.width * x_fraction)
                shelf_width = max(1, int(self.width * 0.07))
                for lower, upper in ((0.17, 0.39), (0.60, 0.81)):
                    for col in range(left, min(self.width - 3, left + shelf_width)):
                        for row in range(int(self.height * lower), int(self.height * upper)):
                            self.static.add((col, row))
            actors = (
                (0.30, 0.47, 1.15, 0.0), (0.72, 0.47, -0.85, 0.0),
                (0.48, 0.54, -0.90, 0.0), (0.90, 0.54, 0.80, 0.0),
                (0.08, 0.36, 0.0, 0.80), (0.95, 0.65, 0.0, -0.75),
                (0.55, 0.10, -0.75, 0.0), (0.30, 0.90, 0.85, 0.0),
            )
        elif self.scenario == "crossing":

            for x_fraction, y_fraction in (
                (0.23, 0.22), (0.67, 0.22), (0.23, 0.67), (0.67, 0.67),
                (0.36, 0.15), (0.60, 0.79), (0.10, 0.43), (0.81, 0.43),
            ):
                for col in range(int(self.width * x_fraction), int(self.width * x_fraction) + max(2, self.width // 10)):
                    for row in range(int(self.height * y_fraction), int(self.height * y_fraction) + max(2, self.height // 10)):
                        if col < self.width - 3 and row < self.height - 3:
                            self.static.add((col, row))
            actors = (
                (0.50, 0.18, 0.0, 1.25), (0.50, 0.76, 0.0, -1.0),
                (0.20, 0.53, 1.10, 0.0), (0.78, 0.53, -1.30, 0.0),
                (0.42, 0.30, 0.0, 0.85), (0.60, 0.65, 0.0, -0.90),
                (0.60, 0.11, 0.75, 0.0), (0.30, 0.90, -0.80, 0.0),
            )
        else:

            for x_fraction, y_fraction in (
                (0.20, 0.20), (0.55, 0.18), (0.75, 0.27), (0.34, 0.38),
                (0.65, 0.49), (0.16, 0.65), (0.44, 0.73), (0.76, 0.77),
            ):
                for col in range(int(self.width * x_fraction), int(self.width * x_fraction) + max(2, self.width // 14)):
                    for row in range(int(self.height * y_fraction), int(self.height * y_fraction) + max(2, self.height // 14)):
                        if col < self.width - 3 and row < self.height - 3:
                            self.static.add((col, row))
            actors = (
                (0.45, 0.28, 0.40, 0.90), (0.65, 0.68, -0.80, -0.35),
                (0.28, 0.65, 1.0, -0.2), (0.13, 0.35, 0.0, 0.75),
                (0.87, 0.58, -0.65, 0.0), (0.32, 0.88, 0.80, 0.0),
                (0.72, 0.10, -0.75, 0.20), (0.50, 0.52, 0.60, -0.50),
            )

        self.static = {cell for cell in self.static if all(_segment_rectangle_distance_sq(point, point, *cell) > 1.0 for point in (self.start, self.goal))}
        for x, y, vx, vy in actors:
            actor = MovingObstacle(x * self.width, y * self.height, vx, vy, 0.42 + rng.random() * 0.10)
            if not self.valid_point((actor.x, actor.y), actor.radius + 0.04):
                continue
            if min(math.hypot(actor.x - p[0], actor.y - p[1]) for p in (self.start, self.goal)) < 2:
                continue
            if any(math.hypot(actor.x - other.x, actor.y - other.y) <= actor.radius + other.radius + 0.08 for other in self.obstacles):
                continue
            actor._world = self
            self.obstacles.append(actor)

    def _nearby_static(self, start: Point, end: Point, radius: float) -> Iterable[Cell]:
        left = max(0, math.floor(min(start[0], end[0]) - radius))
        right = min(self.width - 1, math.floor(max(start[0], end[0]) + radius))
        top = max(0, math.floor(min(start[1], end[1]) - radius))
        bottom = min(self.height - 1, math.floor(max(start[1], end[1]) + radius))
        for row in range(top, bottom + 1):
            for col in range(left, right + 1):
                if (col, row) in self.static:
                    yield col, row

    def valid_point(self, point: Point, radius: float = 0.32) -> bool:

        if radius < 0 or not all(math.isfinite(value) for value in point):
            return False
        x, y = point
        if x < radius or y < radius or x > self.width - radius or y > self.height - radius:
            return False
        return all(_segment_rectangle_distance_sq(point, point, *cell) > radius * radius + _EPS for cell in self._nearby_static(point, point, radius))

    def _static_free(self, start: Point, end: Point, radius: float) -> bool:
        if radius < 0 or not all(math.isfinite(value) for value in (*start, *end)):
            return False
        if min(start[0], end[0]) < radius or min(start[1], end[1]) < radius or max(start[0], end[0]) > self.width - radius or max(start[1], end[1]) > self.height - radius:
            return False
        radius_sq = radius * radius + _EPS
        return all(_segment_rectangle_distance_sq(start, end, *cell) > radius_sq for cell in self._nearby_static(start, end, radius))

    def _first_impact(self, start: Point, delta: Point, radius: float):
        earliest, normal = 1.0 + _EPS, (0.0, 0.0)
        for axis, limit in ((0, self.width), (1, self.height)):
            if abs(delta[axis]) < _EPS:
                continue
            wall = limit - radius if delta[axis] > 0 else radius
            fraction = (wall - start[axis]) / delta[axis]
            if -_EPS <= fraction <= earliest and fraction <= 1.0:
                direction = -1.0 if delta[axis] > 0 else 1.0
                if abs(fraction - earliest) <= _EPS:
                    normal = (direction, normal[1]) if axis == 0 else (normal[0], direction)
                else:
                    normal = (direction, 0.0) if axis == 0 else (0.0, direction)
                earliest = max(0.0, fraction)
        end = start[0] + delta[0], start[1] + delta[1]
        for col, row in self._nearby_static(start, end, radius):
            hit = _slab_hit(start, delta, (col - radius, row - radius, col + 1 + radius, row + 1 + radius))


            if col - radius <= start[0] <= col + 1 + radius and row - radius <= start[1] <= row + 1 + radius:
                hit = _disk_rectangle_hit(start, delta, col, row, radius)
            if hit is None:
                continue
            fraction, candidate_normal = hit
            if fraction < earliest - _EPS:
                earliest, normal = fraction, candidate_normal
            elif abs(fraction - earliest) <= _EPS:
                normal = (candidate_normal[0] or normal[0], candidate_normal[1] or normal[1])
        return None if earliest > 1.0 else (earliest, normal)

    def _advance(self, actor: MovingObstacle, duration: float):

        x, y, vx, vy = actor.x, actor.y, actor.vx, actor.vy
        remaining, elapsed = max(0.0, duration), 0.0
        segments = [(0.0, x, y)]
        for _ in range(64):
            if remaining < _EPS:
                break
            delta = vx * remaining, vy * remaining
            hit = self._first_impact((x, y), delta, actor.radius)
            if hit is None:
                x, y = x + delta[0], y + delta[1]
                elapsed += remaining
                remaining = 0.0
                segments.append((elapsed, x, y))
                break
            fraction, normal = hit
            used = remaining * fraction
            x += delta[0] * fraction + normal[0] * _EPS * 4
            y += delta[1] * fraction + normal[1] * _EPS * 4
            elapsed += used
            remaining -= used
            segments.append((elapsed, x, y))
            if normal[0]:
                vx = -vx
            if normal[1]:
                vy = -vy

        if segments[-1][0] < duration:
            segments.append((duration, x, y))
        return (x, y, vx, vy), segments

    @staticmethod
    def _interpolate(segments, time: float) -> Point:
        for index in range(1, len(segments)):
            after = segments[index]
            if after[0] + _EPS >= time:
                before = segments[index - 1]
                fraction = 0.0 if after[0] - before[0] < _EPS else (time - before[0]) / (after[0] - before[0])
                return before[1] + (after[1] - before[1]) * fraction, before[2] + (after[2] - before[2]) * fraction
        return segments[-1][1], segments[-1][2]

    def _sample_motion(self, actor: MovingObstacle, horizon: float, step: float):
        if horizon < 0 or step <= 0:
            raise ValueError("Prediction horizon must be nonnegative and step positive")
        _, segments = self._advance(actor, horizon)
        times = [min(index * step, horizon) for index in range(math.ceil(horizon / step) + 1)]
        return [(time, *self._interpolate(segments, time)) for time in times]

    def predictions(self, horizon: float, step: float = 0.25) -> list[list[tuple[float, float, float]]]:
        return [self._sample_motion(actor, horizon, step) for actor in self.obstacles]

    def _future_segments(self, horizon: float):
        signature = (self.revision, tuple((o.x, o.y, o.vx, o.vy, o.radius) for o in self.obstacles))
        if signature != self._forecast_signature:
            self._forecast_signature = signature
            self._forecast_cache.clear()

        cache_horizon = max(0.5, math.ceil(horizon * 2.0) / 2.0)
        if cache_horizon not in self._forecast_cache:
            if len(self._forecast_cache) > 12:
                self._forecast_cache.clear()
            self._forecast_cache[cache_horizon] = [self._advance(actor, cache_horizon)[1] for actor in self.obstacles]
        return self._forecast_cache[cache_horizon]

    def collision_free(self, start: Point, end: Point, radius: float = 0.32, time_horizon: float = 0.0, start_time: float = 0.0) -> bool:

        if time_horizon < 0 or start_time < 0 or not math.isfinite(time_horizon) or not math.isfinite(start_time):
            return False
        if not self._static_free(start, end, radius):
            return False
        if not self.obstacles:
            return True
        if start_time == 0.0 and time_horizon == 0.0:
            return all(_point_segment_distance_sq((o.x, o.y), start, end) > (radius + o.radius) ** 2 + _EPS for o in self.obstacles)
        trajectories = self._future_segments(start_time + time_horizon)
        for actor, segments in zip(self.obstacles, trajectories):
            combined_sq = (radius + actor.radius) ** 2 + _EPS
            if time_horizon <= _EPS:
                if _point_segment_distance_sq(self._interpolate(segments, start_time), start, end) <= combined_sq:
                    return False
                continue
            finish_time = start_time + time_horizon
            times = [start_time] + [point[0] for point in segments if start_time < point[0] < finish_time] + [finish_time]
            for before, after in zip(times, times[1:]):
                relative = []
                for time in (before, after):
                    fraction = (time - start_time) / time_horizon
                    robot = start[0] + fraction * (end[0] - start[0]), start[1] + fraction * (end[1] - start[1])
                    mover = self._interpolate(segments, time)
                    relative.append((robot[0] - mover[0], robot[1] - mover[1]))
                if _point_segment_distance_sq((0.0, 0.0), relative[0], relative[1]) <= combined_sq:
                    return False
        return True

    def update(self, dt: float) -> None:
        if dt < 0 or not math.isfinite(dt):
            raise ValueError("World time step must be finite and nonnegative")
        for actor in self.obstacles:
            actor._world = self
            state, _ = self._advance(actor, dt)
            actor.x, actor.y, actor.vx, actor.vy = state
        self._forecast_cache.clear()

    def occupancy(self) -> set[Cell]:

        cells = set(self.static)
        for actor in self.obstacles:
            for row in range(max(0, math.floor(actor.y - actor.radius)), min(self.height, math.floor(actor.y + actor.radius) + 1)):
                for col in range(max(0, math.floor(actor.x - actor.radius)), min(self.width, math.floor(actor.x + actor.radius) + 1)):
                    if _segment_rectangle_distance_sq((actor.x, actor.y), (actor.x, actor.y), col, row) <= actor.radius ** 2 + _EPS:
                        cells.add((col, row))
        return cells

    def set_obstacle(self, cell: Cell, occupied: bool) -> bool:
        col, row = cell
        if not (isinstance(col, int) and isinstance(row, int) and 0 <= col < self.width and 0 <= row < self.height):
            return False
        if occupied:
            if cell in self.static:
                return False
            if any(_segment_rectangle_distance_sq(point, point, col, row) <= 0.32 ** 2 + _EPS for point in (self.start, self.goal)):
                return False
            if any(_segment_rectangle_distance_sq((o.x, o.y), (o.x, o.y), col, row) <= o.radius ** 2 + _EPS for o in self.obstacles):
                return False
            self.static.add(cell)
        else:
            if cell not in self.static:
                return False
            self.static.remove(cell)
        self.revision += 1
        self._forecast_cache.clear()
        return True
