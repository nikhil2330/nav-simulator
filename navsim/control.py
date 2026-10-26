from __future__ import annotations

import bisect
import math
from typing import Sequence

from .estimation import integrate_pose, wrap_angle

Point = tuple[float, float]


class WaypointController:


    def __init__(
        self,
        max_speed: float = 2.8,
        max_turn_rate: float = 2.5,
        lookahead: float = 0.65,
        goal_tolerance: float = 0.28,
    ) -> None:
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (max_speed, max_turn_rate, lookahead, goal_tolerance)
        ):
            raise ValueError("Controller limits and distances must be positive")
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate
        self.lookahead = lookahead
        self.goal_tolerance = goal_tolerance
        self.waypoint_index = 0
        self.target: Point | None = None
        self.reached = False
        self._path: list[Point] = []
        self._lengths: list[float] = []
        self._corners: list[int] = []
        self._corner_cursor = 0
        self._progress = 0.0
        self._turning_corner = False

    def set_path(self, path: Sequence[Point]) -> None:
        self._path = []
        for point in path:
            value = (float(point[0]), float(point[1]))
            if not all(math.isfinite(coordinate) for coordinate in value):
                raise ValueError("Path coordinates must be finite")
            if not self._path or math.dist(value, self._path[-1]) > 1e-9:
                self._path.append(value)
        self._lengths = [0.0] if self._path else []
        for first, second in zip(self._path, self._path[1:]):
            self._lengths.append(self._lengths[-1] + math.dist(first, second))
        self._corners = []
        for i in range(1, len(self._path) - 1):
            before = math.atan2(
                self._path[i][1] - self._path[i - 1][1],
                self._path[i][0] - self._path[i - 1][0],
            )
            after = math.atan2(
                self._path[i + 1][1] - self._path[i][1],
                self._path[i + 1][0] - self._path[i][0],
            )
            if abs(wrap_angle(after - before)) > math.radians(35.0):
                self._corners.append(i)
        self._corner_cursor = 0
        self._progress = 0.0
        self._turning_corner = False
        self.waypoint_index = min(1, len(self._path) - 1) if self._path else 0
        self.target = self._path[0] if self._path else None
        self.reached = False

    def command(
        self, pose: Sequence[float], path: Sequence[Point], dt: float
    ) -> tuple[float, float]:
        if not math.isfinite(dt) or dt < 0.0:
            raise ValueError("Time step must be finite and nonnegative")


        candidate = [(float(point[0]), float(point[1])) for point in path]
        compact = [point for i, point in enumerate(candidate) if i == 0 or math.dist(point, candidate[i - 1]) > 1e-9]
        if compact != self._path:
            self.set_path(compact)
        if not self._path:
            self.target = None
            return (0.0, 0.0)
        x, y, heading = pose
        if not all(math.isfinite(value) for value in (x, y, heading)):
            raise ValueError("Pose must be finite")
        goal_distance = math.dist((x, y), self._path[-1])
        if self.reached or goal_distance <= self.goal_tolerance:
            self.reached = True
            self.waypoint_index = len(self._path) - 1
            self.target = self._path[-1]
            return (0.0, 0.0)

        corner = (
            self._corners[self._corner_cursor]
            if self._corner_cursor < len(self._corners)
            else None
        )
        end_segment = corner - 1 if corner is not None else len(self._path) - 2
        self._advance_progress((x, y), end_segment)
        corner_distance = math.dist((x, y), self._path[corner]) if corner is not None else math.inf


        if corner is not None and (self._turning_corner or corner_distance <= 0.11):
            self._turning_corner = True
            outgoing = self._path[corner + 1]
            vertex = self._path[corner]
            desired = math.atan2(outgoing[1] - vertex[1], outgoing[0] - vertex[0])
            error = wrap_angle(desired - heading)
            self.target = vertex
            self.waypoint_index = corner
            if abs(error) > 0.10:
                return (0.0, self._clamp_turn(3.2 * error))
            self._progress = self._lengths[corner]
            self._corner_cursor += 1
            self._turning_corner = False
            corner = (
                self._corners[self._corner_cursor]
                if self._corner_cursor < len(self._corners)
                else None
            )
            corner_distance = math.dist((x, y), self._path[corner]) if corner is not None else math.inf

        target_distance = min(self._lengths[-1], self._progress + self.lookahead)
        if corner is not None:
            target_distance = min(target_distance, self._lengths[corner])
        self.target = self._point_at(target_distance)
        self.waypoint_index = min(
            len(self._path) - 1, bisect.bisect_right(self._lengths, self._progress + 1e-8)
        )
        dx, dy = self.target[0] - x, self.target[1] - y
        distance = math.hypot(dx, dy)
        error = wrap_angle(math.atan2(dy, dx) - heading)
        if abs(error) > 0.95:
            return (0.0, self._clamp_turn(3.2 * error))
        speed = self.max_speed * max(0.12, math.cos(error)) ** 2
        speed = min(speed, 1.8 * goal_distance)
        if corner is not None:

            speed = min(speed, max(0.16, math.sqrt(2.2 * max(0.0, corner_distance - 0.075))))
        turn = 2.0 * speed * math.sin(error) / max(distance, 0.10)

        if abs(turn) > self.max_turn_rate:
            speed *= self.max_turn_rate / abs(turn)
            turn = math.copysign(self.max_turn_rate, turn)
        if dt > 0.0 and corner is not None:
            speed = min(speed, max(0.0, corner_distance - 0.07) / dt)
        return (max(0.0, speed), self._clamp_turn(turn))

    def _clamp_turn(self, value: float) -> float:
        return max(-self.max_turn_rate, min(self.max_turn_rate, value))

    def _advance_progress(self, position: Point, end_segment: int) -> None:
        if len(self._path) < 2:
            return
        first_segment = max(0, bisect.bisect_right(self._lengths, self._progress) - 1)
        best_distance = math.inf
        best_progress = self._progress
        for i in range(first_segment, end_segment + 1):
            first, second = self._path[i], self._path[i + 1]
            dx, dy = second[0] - first[0], second[1] - first[1]
            length_squared = dx * dx + dy * dy
            fraction = max(0.0, min(1.0, ((position[0] - first[0]) * dx + (position[1] - first[1]) * dy) / length_squared))
            along = self._lengths[i] + fraction * math.sqrt(length_squared)
            if along + 1e-9 < self._progress:
                continue
            projected = (first[0] + fraction * dx, first[1] + fraction * dy)
            distance = math.dist(position, projected)
            if distance < best_distance:
                best_distance = distance
                best_progress = along
        self._progress = max(self._progress, best_progress)

    def _point_at(self, along: float) -> Point:
        if len(self._path) == 1 or along >= self._lengths[-1]:
            return self._path[-1]
        index = max(0, bisect.bisect_right(self._lengths, along) - 1)
        first, second = self._path[index], self._path[index + 1]
        fraction = (along - self._lengths[index]) / (self._lengths[index + 1] - self._lengths[index])
        return (
            first[0] + fraction * (second[0] - first[0]),
            first[1] + fraction * (second[1] - first[1]),
        )
