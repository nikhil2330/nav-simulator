from collections import deque
from dataclasses import asdict, dataclass
import math
import random

from .control import WaypointController, integrate_pose
from .estimation import PoseEKF, wrap_angle
from .planning import AStarPlanner
from .world import MovingObstacle, World


@dataclass
class Metrics:
    time_s: float = 0.0
    distance_m: float = 0.0
    plans: int = 0
    replans: int = 0
    planning_ms: float = 0.0
    expanded_nodes: int = 0
    collision_checks: int = 0
    safety_stops: int = 0
    collisions: int = 0
    sensor_updates: int = 0
    occupancy_updates: int = 0
    position_error: float = 0.0
    position_rmse: float = 0.0


class Simulation:


    ROBOT_RADIUS = 0.32
    SENSOR_PERIOD = 0.2
    REPLAN_PERIOD = 1.2

    def __init__(self, seed=7, scenario="warehouse"):
        self.seed = seed
        self.scenario = scenario
        self.rng = random.Random(seed + 1000)
        self._obstacle_rng = random.Random(seed + 2000)
        self.world = World(scenario=scenario, seed=seed)
        self.pose = (*self.world.start, 0.0)
        self.estimator = PoseEKF(*self.pose)
        self.controller = WaypointController()

        self.planner = AStarPlanner(self.world, self.ROBOT_RADIUS + 0.13)


        self._tight_planner = AStarPlanner(self.world, self.ROBOT_RADIUS)
        self._tight_clearance = False
        self.metrics = Metrics()
        self.state = "READY"
        self.path = []
        self.trail = deque([self.pose[:2]], maxlen=4000)
        self.estimate_trail = deque([self.pose[:2]], maxlen=4000)
        self.measurements = deque(maxlen=45)
        self.events = deque(maxlen=3)
        self.error_history = deque(maxlen=180)
        self.speed_history = deque(maxlen=180)
        self.occupancy = self.world.occupancy()
        self.speed = 0.0
        self.turn_rate = 0.0
        self.sensor_noise = True
        self.prediction_enabled = True
        self._sensor_time = 0.0
        self._plan_time = 0.0
        self._occupancy_time = 0.0
        self._error_sum = 0.0
        self._samples = 0
        self._stopped = False
        self._safety_hold_logged = False
        self._waiting_reason = ""
        self._last_revision = self.world.revision
        self._initial_distance = math.dist(self.world.start, self.world.goal)
        self.replan("Mission initialized")

    @property
    def holding(self):
        return self._stopped and self.state == "TRACKING"

    @property
    def status_reason(self):

        if self.state == "COLLISION":
            return "Collision detected. Reset to try again."
        if self.state == "WAITING":
            return self._waiting_reason or "No route to goal. Erase walls or move the goal."
        if self.holding:
            return "Obstacle too close. Stopped for safety; retrying."
        if self.state == "GOAL_REACHED":
            return "Goal reached."
        return "Following the route to the goal."

    @property
    def progress(self):
        if self.state == "GOAL_REACHED":
            return 1.0
        distance = math.dist(self.pose[:2], self.world.goal)
        return max(0.0, min(1.0, 1.0 - distance / max(self._initial_distance, 0.01)))

    def log(self, kind, message):
        self.events.appendleft((self.metrics.time_s, kind, message))

    def replan(self, reason="World updated"):
        previous_state = self.state
        self._plan_time = 0.0
        self._last_revision = self.world.revision
        start = tuple(self.estimator.state[:2])
        horizon = 2.0 if self.prediction_enabled else 0.0
        result = self.planner.plan(start, self.world.goal, prediction_horizon=horizon)
        self._tight_clearance = False
        if not result.path:
            fallback = self._tight_planner.plan(start, self.world.goal, prediction_horizon=horizon)
            fallback.expanded += result.expanded
            fallback.elapsed_ms += result.elapsed_ms
            result = fallback
            self._tight_clearance = bool(result.path)
        self.metrics.plans += 1
        self.metrics.replans = max(0, self.metrics.plans - 1)
        self.metrics.planning_ms = result.elapsed_ms
        self.metrics.expanded_nodes = result.expanded
        self.path = result.path
        self.controller.set_path(self.path)
        if self.path:
            self._waiting_reason = ""
            self.state = "TRACKING"
            if reason == "Mission initialized":
                self.log("PLAN", "Ready to navigate.")
            elif reason in ("New goal selected", "Robot repositioned"):
                self.log("PLAN", f"{reason}.")
            elif previous_state == "WAITING":
                self.log("PLAN", "Route available. Continuing.")
        else:
            self.state = "WAITING"
            if not self.world.valid_point(start, self.ROBOT_RADIUS):
                self._waiting_reason = "Pose estimate is too close to a wall. Retrying."
            elif not self.world.valid_point(self.world.goal, self.ROBOT_RADIUS):
                self._waiting_reason = "Goal is too close to a wall. Move the goal."
            elif not self.world.collision_free(start, start, self.ROBOT_RADIUS):
                self._waiting_reason = "A moving obstacle is too close. Waiting for it to pass."
            else:
                self._waiting_reason = "No route to goal. Erase walls or move the goal."
            if previous_state != "WAITING":
                self.log("WAIT", self._waiting_reason)

    def _obstacles_changed(self, reason):

        self.world._forecast_cache.clear()
        self.occupancy = self.world.occupancy()
        self.metrics.occupancy_updates += 1
        if self.state in ("GOAL_REACHED", "COLLISION"):

            self.speed = self.turn_rate = 0.0
        else:
            self.replan(reason)

    def spawn_obstacle(self) -> bool:

        protected = (self.pose[:2], self.estimator.state[:2], self.world.start, self.world.goal)
        for _ in range(160):
            radius = self._obstacle_rng.uniform(0.42, 0.52)
            speed = self._obstacle_rng.uniform(0.6, 1.3)
            angle = self._obstacle_rng.uniform(0.0, math.tau)
            margin = radius + 0.08
            point = (
                self._obstacle_rng.uniform(margin, self.world.width - margin),
                self._obstacle_rng.uniform(margin, self.world.height - margin),
            )
            gap = radius + self.ROBOT_RADIUS + 1.5
            if any(math.dist(point, endpoint) < gap for endpoint in protected):
                continue
            vx, vy = speed * math.cos(angle), speed * math.sin(angle)
            duration = 0.4
            end = point[0] + vx * duration, point[1] + vy * duration
            if not self.world.collision_free(point, end, margin, time_horizon=duration):
                continue
            actor = MovingObstacle(*point, vx, vy, radius)
            actor._world = self.world
            self.world.obstacles.append(actor)
            self._obstacles_changed("Moving obstacle added")
            return True
        return False

    def despawn_obstacle(self) -> bool:

        if not self.world.obstacles:
            return False
        index = self._obstacle_rng.randrange(len(self.world.obstacles))
        actor = self.world.obstacles.pop(index)
        actor._world = None
        self._obstacles_changed("Moving obstacle removed")
        return True

    def edit_obstacle(self, cell, occupied):

        if occupied:
            for x, y in (self.pose[:2], self.estimator.state[:2]):
                cx = max(cell[0], min(x, cell[0] + 1))
                cy = max(cell[1], min(y, cell[1] + 1))
                if math.hypot(cx - x, cy - y) < self.ROBOT_RADIUS + 0.1:
                    return False
        if self.world.set_obstacle(cell, occupied):
            self.occupancy = self.world.occupancy()
            self.replan("Occupancy grid changed")
            return True
        return False

    def set_goal(self, point):
        if not self.world.collision_free(point, point, self.ROBOT_RADIUS):
            self.log("WAIT", "Goal needs a clear robot footprint")
            return False
        self.world.goal = point
        self._initial_distance = math.dist(self.pose[:2], point)
        self.replan("New goal selected")
        return True

    def set_start(self, point):
        if not self.world.collision_free(point, point, self.ROBOT_RADIUS):
            return False
        self.world.start = point
        self.pose = (*point, 0.0)
        self.estimator = PoseEKF(*self.pose)
        self.trail.clear()
        self.estimate_trail.clear()
        self.measurements.clear()
        self.error_history.clear()
        self.speed_history.clear()
        self._error_sum = 0.0
        self._samples = 0
        self.metrics.position_error = self.metrics.position_rmse = 0.0
        self.speed = self.turn_rate = 0.0
        self._safety_hold_logged = False
        self._initial_distance = math.dist(point, self.world.goal)
        self.replan("Robot repositioned")
        return True

    def _command_safe(self, velocity, turn_rate, dt, horizon=0.6):

        pose = self.pose
        elapsed = 0.0
        while elapsed < horizon - 1e-9:
            interval = min(0.1, horizon - elapsed)
            end = integrate_pose(pose, velocity, turn_rate, interval)
            self.metrics.collision_checks += 1
            if not self.world.collision_free(
                pose[:2], end[:2], self.ROBOT_RADIUS + self._arc_padding(velocity, turn_rate, interval),
                time_horizon=interval, start_time=elapsed,
            ):
                return False
            pose = end
            elapsed += interval
        return True

    @staticmethod
    def _arc_padding(velocity, turn_rate, interval):

        if abs(turn_rate) <= 1e-8:
            return 0.0
        return abs(velocity / turn_rate) * (1.0 - math.cos(abs(turn_rate) * interval / 2.0))

    def step(self, dt=0.04):
        if not math.isfinite(dt) or dt <= 0 or dt > 0.1:
            raise ValueError("Use fixed time steps between 0 and 0.1 seconds")
        if self.state in ("GOAL_REACHED", "COLLISION"):
            return
        self._plan_time += dt
        self._sensor_time += dt
        self._occupancy_time += dt
        if self.world.revision != self._last_revision:
            self.replan("Occupancy grid changed")
        elif self._plan_time >= self.REPLAN_PERIOD:
            self.replan("Prediction refresh")

        velocity, turn = 0.0, 0.0
        if self.path:
            velocity, turn = self.controller.command(
                tuple(self.estimator.state), self.path, dt,
            )
            if self._tight_clearance and velocity > 0.8:

                turn *= 0.8 / velocity
                velocity = 0.8
            if self.controller.reached:

                if math.dist(self.pose[:2], self.world.goal) < 0.42:
                    self.state = "GOAL_REACHED"
                    self.speed = self.turn_rate = 0.0
                    self.log("GOAL", "Mission complete. Robot stopped.")
                    return
                self.replan("Refining final approach")

        requested_speed = velocity
        if velocity > 0:
            for scale in (1.0, 0.6, 0.25):
                if self._command_safe(requested_speed * scale, turn * scale, dt):
                    velocity = requested_speed * scale
                    turn *= scale
                    break
            else:
                velocity = 0.0
        stopped = requested_speed > 0.05 and velocity == 0.0
        if stopped and not self._stopped:
            self.metrics.safety_stops += 1
            if not self._safety_hold_logged:
                self.log("SAFE", "Obstacle ahead. Waiting.")
                self._safety_hold_logged = True
            if self._plan_time > 0.25:
                self.replan("Predicted collision")
        self._stopped = stopped

        end = integrate_pose(self.pose, velocity, turn, dt)
        contact = False
        self.metrics.collision_checks += 1
        if not self.world.collision_free(
            self.pose[:2], end[:2], self.ROBOT_RADIUS + self._arc_padding(velocity, turn, dt), time_horizon=dt,
        ):


            if not self.world.collision_free(
                self.pose[:2], self.pose[:2], self.ROBOT_RADIUS, time_horizon=dt,
            ):
                self.state = "COLLISION"
                self.metrics.collisions += 1


                contact = True
                turn = 0.0
            velocity = 0.0
            end = integrate_pose(self.pose, 0.0, turn, dt)
        self.world.update(dt)
        self.metrics.time_s += dt
        self.metrics.distance_m += math.dist(self.pose[:2], end[:2])
        self.pose = end
        self.speed, self.turn_rate = velocity, turn
        if velocity > 0:
            self._safety_hold_logged = False

        noise_v = self.rng.gauss(0.0, 0.06) if self.sensor_noise else 0.0
        noise_w = self.rng.gauss(0.0, 0.025) if self.sensor_noise else 0.0
        self.estimator.predict(velocity + noise_v, turn + noise_w, dt)
        if self._sensor_time >= self.SENSOR_PERIOD - 1e-9:
            self._sensor_time -= self.SENSOR_PERIOD
            if self.sensor_noise:
                measurement = (
                    self.pose[0] + self.rng.gauss(0.0, 0.12),
                    self.pose[1] + self.rng.gauss(0.0, 0.12),
                    wrap_angle(self.pose[2] + self.rng.gauss(0.0, 0.06)),
                )
            else:
                measurement = self.pose
            self.estimator.update(measurement)
            self.measurements.append(measurement[:2])
            self.metrics.sensor_updates += 1
            self.error_history.append(math.dist(self.pose[:2], self.estimator.state[:2]))
            self.speed_history.append(velocity)
        if self._occupancy_time >= 0.2 - 1e-9:
            self._occupancy_time = 0.0
            self.occupancy = self.world.occupancy()
            self.metrics.occupancy_updates += 1
        error = math.dist(self.pose[:2], self.estimator.state[:2])
        self.metrics.position_error = error
        self._error_sum += error * error
        self._samples += 1
        self.metrics.position_rmse = math.sqrt(self._error_sum / self._samples)
        self.trail.append(self.pose[:2])
        self.estimate_trail.append(tuple(self.estimator.state[:2]))
        if contact:
            self.log("STOP", "Moving obstacle contact / reset mission")

    def report(self):
        return {
            "state": self.state,
            "scenario": self.scenario,
            "seed": self.seed,
            "units": {"distance": "metres", "time": "seconds", "angle": "radians"},
            "metrics": asdict(self.metrics),
            "true_pose": self.pose,
            "estimated_pose": self.estimator.state,
            "covariance": self.estimator.covariance,
            "goal": self.world.goal,
            "progress": self.progress,
            "status_reason": self.status_reason,
        }
