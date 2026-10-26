from __future__ import annotations

import math
from typing import Sequence


def wrap_angle(angle: float) -> float:

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _sinc(value: float) -> float:
    if abs(value) < 1e-5:
        return 1.0 - value * value / 6.0 + value**4 / 120.0
    return math.sin(value) / value


def integrate_pose(
    pose: Sequence[float], v: float, w: float, dt: float
) -> tuple[float, float, float]:

    x, y, theta = pose
    if not all(math.isfinite(value) for value in (x, y, theta, v, w, dt)):
        raise ValueError("Pose, velocities and time step must be finite")
    if dt < 0.0:
        raise ValueError("Time step cannot be negative")
    half_turn = 0.5 * w * dt
    travel = v * dt * _sinc(half_turn)
    return (
        x + travel * math.cos(theta + half_turn),
        y + travel * math.sin(theta + half_turn),
        wrap_angle(theta + 2.0 * half_turn),
    )


def _transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [list(row) for row in zip(*matrix)]


def _multiply(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))]
        for i in range(len(a))
    ]


def _inverse3(matrix: list[list[float]]) -> list[list[float]]:

    rows = [row[:] + [float(i == j) for j in range(3)] for i, row in enumerate(matrix)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda i: abs(rows[i][column]))
        rows[column], rows[pivot] = rows[pivot], rows[column]
        scale = rows[column][column]
        if abs(scale) < 1e-18:
            raise ValueError("Innovation covariance is singular")
        rows[column] = [value / scale for value in rows[column]]
        for i in range(3):
            if i != column:
                factor = rows[i][column]
                rows[i] = [a - factor * b for a, b in zip(rows[i], rows[column])]
    return [row[3:] for row in rows]


class PoseEKF:


    def __init__(
        self,
        x: float,
        y: float,
        theta: float = 0.0,
        position_noise: float = 0.12,
        heading_noise: float = 0.06,
    ) -> None:
        if not all(math.isfinite(value) for value in (x, y, theta)):
            raise ValueError("Initial pose must be finite")
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (position_noise, heading_noise)
        ):
            raise ValueError("Measurement standard deviations must be positive")
        self.state = [float(x), float(y), wrap_angle(theta)]
        self.position_noise = position_noise
        self.heading_noise = heading_noise
        self.covariance = [
            [position_noise**2, 0.0, 0.0],
            [0.0, position_noise**2, 0.0],
            [0.0, 0.0, heading_noise**2],
        ]

    @property
    def position_std(self) -> float:

        return math.sqrt(max(0.0, (self.covariance[0][0] + self.covariance[1][1]) / 2.0))

    def predict(self, v: float, w: float, dt: float) -> None:
        previous_theta = self.state[2]
        predicted = integrate_pose(self.state, v, w, dt)
        if dt == 0.0:
            return
        half_turn = 0.5 * w * dt
        sinc = _sinc(half_turn)
        if abs(half_turn) < 1e-5:
            sinc_derivative = -half_turn / 3.0 + half_turn**3 / 30.0
        else:
            sinc_derivative = (
                half_turn * math.cos(half_turn) - math.sin(half_turn)
            ) / half_turn**2
        middle_heading = previous_theta + half_turn
        cosine, sine = math.cos(middle_heading), math.sin(middle_heading)
        distance = v * dt * sinc
        transition = [
            [1.0, 0.0, -distance * sine],
            [0.0, 1.0, distance * cosine],
            [0.0, 0.0, 1.0],
        ]

        control = [
            [dt * sinc * cosine, 0.5 * v * dt**2 * (sinc_derivative * cosine - sinc * sine)],
            [dt * sinc * sine, 0.5 * v * dt**2 * (sinc_derivative * sine + sinc * cosine)],
            [0.0, dt],
        ]
        velocity_variance = (0.055 + 0.08 * abs(v)) ** 2
        turn_variance = (0.035 + 0.10 * abs(w)) ** 2
        process_noise = [
            [
                control[i][0] * velocity_variance * control[j][0]
                + control[i][1] * turn_variance * control[j][1]
                + (dt * (0.004 if i < 2 else 0.003) ** 2 if i == j else 0.0)
                for j in range(3)
            ]
            for i in range(3)
        ]
        propagated = _multiply(_multiply(transition, self.covariance), _transpose(transition))
        self.covariance = [
            [propagated[i][j] + process_noise[i][j] for j in range(3)]
            for i in range(3)
        ]
        self.state = list(predicted)
        self._symmetrize()

    def update(self, measurement: Sequence[float]) -> None:
        if len(measurement) != 3 or not all(math.isfinite(value) for value in measurement):
            raise ValueError("Measurement must contain finite x, y and heading")
        variances = [self.position_noise**2, self.position_noise**2, self.heading_noise**2]
        innovation = [measurement[i] - self.state[i] for i in range(3)]
        innovation[2] = wrap_angle(innovation[2])
        innovation_covariance = [
            [self.covariance[i][j] + (variances[i] if i == j else 0.0) for j in range(3)]
            for i in range(3)
        ]
        gain = _multiply(self.covariance, _inverse3(innovation_covariance))
        self.state = [
            self.state[i] + sum(gain[i][j] * innovation[j] for j in range(3))
            for i in range(3)
        ]
        self.state[2] = wrap_angle(self.state[2])
        identity_minus_gain = [
            [float(i == j) - gain[i][j] for j in range(3)] for i in range(3)
        ]

        corrected = _multiply(
            _multiply(identity_minus_gain, self.covariance), _transpose(identity_minus_gain)
        )
        self.covariance = [
            [
                corrected[i][j] + sum(gain[i][k] * variances[k] * gain[j][k] for k in range(3))
                for j in range(3)
            ]
            for i in range(3)
        ]
        self._symmetrize()

    def _symmetrize(self) -> None:
        for i in range(3):
            self.covariance[i][i] = max(1e-15, self.covariance[i][i])
            for j in range(i):
                value = 0.5 * (self.covariance[i][j] + self.covariance[j][i])
                self.covariance[i][j] = self.covariance[j][i] = value
