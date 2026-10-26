import argparse
import json
import math
import os
from pathlib import Path

from .simulation import Simulation

BG, PANEL, MAP_BG = (18, 20, 24), (27, 30, 36), (21, 24, 29)
BORDER, GRID, TEXT, MUTED = (96, 106, 122), (44, 49, 58), (242, 244, 248), (193, 200, 212)
BLUE, PURPLE = (89, 162, 255), (190, 161, 255)
ORANGE, RED, GREEN = (244, 185, 76), (245, 104, 104), (91, 207, 133)
ENVIRONMENTS = (("env1", "Env 1", "warehouse"), ("env2", "Env 2", "crossing"), ("env3", "Env 3", "open"))


class Dashboard:
    WIDTH, HEIGHT = 1000, 600
    MAP_X, MAP_Y, SCALE = 16, 64, 13

    def __init__(self, simulation):
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        os.environ.setdefault("SDL_VIDEO_CENTERED", "1")
        import pygame

        self.pg = pygame
        pygame.init()
        desktop_width, desktop_height = pygame.display.get_desktop_sizes()[0]
        initial_scale = min(1.0, max(0.1, (desktop_width - 24) / self.WIDTH),
                            max(0.1, (desktop_height - 100) / self.HEIGHT))
        initial_size = round(self.WIDTH * initial_scale), round(self.HEIGHT * initial_scale)
        self.screen = pygame.display.set_mode(initial_size, pygame.RESIZABLE)
        pygame.display.set_caption("Navigation Simulator")
        self.canvas = pygame.Surface((self.WIDTH, self.HEIGHT))
        self.clock = pygame.time.Clock()
        self.sim = simulation
        self.paused = True
        self.running = True
        self.speed_multiplier = 1.0
        self.layers = {"costmap": False, "estimate": False, "trail": False}
        self.show_help = False
        self.notice = ""
        self.notice_until = 0
        self.buttons, self.fonts = [], {}
        self.hover, self.paint = None, None
        self.painted_cells = set()
        self.accumulator = 0.0
        self._cost_revision, self._cost_surface = None, None
        self.view_scale, self.view_offset = 1.0, (0, 0)

    def font(self, size, mono=False, bold=False):
        key = size, mono, bold
        if key not in self.fonts:
            self.fonts[key] = self.pg.font.SysFont("consolas" if mono else "segoeui", size, bold=bold)
        return self.fonts[key]

    def text(self, text, x, y, size=14, color=TEXT, mono=False, bold=False):
        surface = self.font(size, mono, bold).render(str(text), True, color)
        self.canvas.blit(surface, (int(x), int(y)))
        return surface.get_width()

    def line(self, a, b, color=BORDER, width=1):
        self.pg.draw.line(self.canvas, color, a, b, width)

    def panel(self, rect, color=PANEL, border=True, radius=4):
        self.pg.draw.rect(self.canvas, color, rect, border_radius=radius)
        if border:
            self.pg.draw.rect(self.canvas, BORDER, rect, 1, border_radius=radius)

    def button(self, rect, label, action, active=False, accent=False, color=TEXT):
        hovered = self.pg.Rect(rect).collidepoint(self.mouse())
        fill = BLUE if accent else ((45, 76, 112) if active else PANEL)
        if hovered:
            fill = tuple(max(0, c - 8) for c in fill)
        self.panel(rect, fill, border=not accent)
        surface = self.font(14, bold=accent).render(label, True, BG if accent else color)
        self.canvas.blit(surface, surface.get_rect(center=self.pg.Rect(rect).center))
        self.buttons.append((self.pg.Rect(rect), action))

    def mouse(self, event_pos=None):
        x, y = event_pos or self.pg.mouse.get_pos()
        return ((x - self.view_offset[0]) / self.view_scale,
                (y - self.view_offset[1]) / self.view_scale)

    def world_point(self, point):
        wx, wy = (point[0] - self.MAP_X) / self.SCALE, (point[1] - self.MAP_Y) / self.SCALE
        return (wx, wy) if 0 <= wx < self.sim.world.width and 0 <= wy < self.sim.world.height else None

    def screen_point(self, point):
        return round(self.MAP_X + point[0] * self.SCALE), round(self.MAP_Y + point[1] * self.SCALE)

    def path_line(self, points, color, width=2):
        if len(points) > 1:
            self.pg.draw.lines(self.canvas, color, False, [self.screen_point(p) for p in points], width)

    def draw_costmap(self):
        world = self.sim.world
        if self._cost_revision != world.revision or self._cost_surface is None:
            surface = self.pg.Surface((world.width * self.SCALE, world.height * self.SCALE))
            surface.fill(MAP_BG)
            costs = {}
            for x, y in world.static:
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        cell, distance = (x + dx, y + dy), math.hypot(dx, dy)
                        if cell not in world.static and distance <= 2.5:
                            costs[cell] = max(costs.get(cell, 0), int(52 * (1 - distance / 3)))
            for (x, y), cost in costs.items():
                color = (106, 75, 36) if cost >= 26 else (67, 53, 35)
                self.pg.draw.rect(surface, color, (x * self.SCALE, y * self.SCALE, self.SCALE, self.SCALE))
            self._cost_surface, self._cost_revision = surface, world.revision
        self.canvas.blit(self._cost_surface, (self.MAP_X, self.MAP_Y))

    def draw_map(self):
        pg, sim, world, scale = self.pg, self.sim, self.sim.world, self.SCALE
        bounds = pg.Rect(self.MAP_X, self.MAP_Y, world.width * scale, world.height * scale)
        pg.draw.rect(self.canvas, MAP_BG, bounds)
        self.canvas.set_clip(bounds)
        if self.layers["costmap"]:
            self.draw_costmap()
        for col in range(world.width + 1):
            x = self.MAP_X + col * scale
            self.line((x, bounds.top), (x, bounds.bottom), GRID)
        for row in range(world.height + 1):
            y = self.MAP_Y + row * scale
            self.line((bounds.left, y), (bounds.right, y), GRID)
        for x, y in world.static:
            pg.draw.rect(self.canvas, (136, 147, 163),
                         (self.MAP_X + x * scale + 1, self.MAP_Y + y * scale + 1, scale - 2, scale - 2))
        if self.layers["trail"]:
            self.path_line(list(sim.trail), (166, 178, 195), 1)
        if sim.path:
            index = max(0, sim.controller.waypoint_index - 1)
            self.path_line(sim.path[index:], BLUE, 2)
        pg.draw.circle(self.canvas, (135, 183, 246), self.screen_point(world.start), 4)
        pg.draw.circle(self.canvas, GREEN, self.screen_point(world.goal), 7)
        for obstacle in world.obstacles:
            pg.draw.circle(self.canvas, RED, self.screen_point((obstacle.x, obstacle.y)),
                           round(obstacle.radius * scale))
        if self.layers["estimate"]:
            ex, ey = self.screen_point(sim.estimator.state[:2])
            pg.draw.rect(self.canvas, PURPLE, (ex - 5, ey - 5, 10, 10), 1)
        x, y, heading = sim.pose
        center = self.screen_point((x, y))
        pg.draw.circle(self.canvas, BLUE, center, round(sim.ROBOT_RADIUS * scale))
        self.line(center, self.screen_point((x + math.cos(heading) * 0.65,
                                            y + math.sin(heading) * 0.65)), BLUE, 2)
        self.hover = self.world_point(self.mouse())
        if self.hover:
            col, row = map(int, self.hover)
            pg.draw.rect(self.canvas, MUTED,
                         (self.MAP_X + col * scale, self.MAP_Y + row * scale, scale, scale), 1)
        self.canvas.set_clip(None)
        pg.draw.rect(self.canvas, BORDER, bounds, 1)

    def checkbox(self, key, label, y):
        rect = self.pg.Rect(672, y, 296, 28)
        box = self.pg.Rect(672, y + 4, 16, 16)
        self.pg.draw.rect(self.canvas, BLUE if self.layers[key] else PANEL, box)
        self.pg.draw.rect(self.canvas, BLUE if self.layers[key] else BORDER, box, 1)
        if self.layers[key]:
            self.line((675, y + 12), (679, y + 16), BG, 2)
            self.line((679, y + 16), (685, y + 8), BG, 2)
        self.text(label, 698, y + 1, 14)
        self.buttons.append((rect, "layer:" + key))

    def wrapped_text(self, message, x, y, width, size=13, color=MUTED, max_lines=None):
        lines, line = [], ""
        for word in message.split():
            candidate = f"{line} {word}".strip()
            if line and self.font(size).size(candidate)[0] > width:
                lines.append(line)
                line = word
            else:
                line = candidate
        if line:
            lines.append(line)
        if max_lines is not None and len(lines) > max_lines:
            lines = lines[:max_lines]
            while self.font(size).size(lines[-1] + "...")[0] > width:
                lines[-1] = lines[-1][:-1]
            lines[-1] += "..."
        for index, value in enumerate(lines):
            self.text(value, x, y + index * (size + 5), size, color)
        return len(lines) * (size + 5)

    def draw_help(self):
        self.panel((100, 52, 800, 496))
        self.text("How to use the simulator", 120, 68, 22, bold=True)
        self.text("Opening Help pauses the run. Close it and press Run to continue.", 120, 101, 13, MUTED)
        notes = (
            ("Map colors", "Blue dot = robot; blue line = its planned route; green dot = goal; gray squares = walls; red dots = moving obstacles."),
            ("Driven path", "Optional line showing where the robot has already driven. It is different from the route it plans to follow next."),
            ("Estimated position", "The Kalman filter's best guess of robot position from noisy motion and pose measurements. It is shown as a small purple square."),
            ("Obstacle clearance", "Highlights space around walls. Planning reserves extra room for the robot's body and position uncertainty."),
            ("No route", "The goal cannot currently be reached with enough clearance. Erase walls, move the goal, or reset. A moving obstacle can also block a route temporarily."),
            ("Predictions", "Used internally to estimate where moving obstacles will be in the next two seconds. Forecast drawings are hidden."),
            ("Moving obstacles", "Spawn adds a moving obstacle in a random clear spot with a random direction and speed. Despawn removes one random moving obstacle. Both work while paused or running."),
        )
        y = 135
        for label, explanation in notes:
            self.text(label, 120, y, 14, bold=True)
            used = self.wrapped_text(explanation, 270, y, 608, 13, TEXT)
            y += max(32, used + 10)
        self.text("Mouse: left-drag walls / right-click goal / Shift + click robot", 120, 475, 13, MUTED)
        self.button((772, 502, 108, 30), "Close help", "help", accent=True)

    def draw(self):
        sim, m = self.sim, self.sim.metrics
        self.buttons = []
        self.canvas.fill(BG)
        self.text("Navigation simulator", 16, 10, 22, bold=True)
        self.text("Move the blue robot to the green goal.", 16, 38, 13, MUTED)
        terminal = sim.state in ("GOAL_REACHED", "COLLISION")
        label = "Restart" if terminal else ("Run" if self.paused else "Pause")
        self.button((548, 16, 92, 32), label, "run", accent=True)
        for x, label, action, width in ((648, "Step", "step", 72), (728, "Reset", "reset", 80),
                                       (816, f"{self.speed_multiplier:g}x", "speed", 64),
                                       (888, "Help", "help", 96)):
            self.button((x, 16, width, 32), label, action)
        self.draw_map()

        self.panel((656, 64, 328, 468))
        self.text("Status", 672, 79, 16, bold=True)
        status, color = {"GOAL_REACHED": ("Goal reached", GREEN),
                         "WAITING": ("No route to goal", ORANGE),
                         "COLLISION": ("Collision detected", RED)}.get(
                             sim.state, ("Paused" if self.paused else "Running", TEXT))
        if sim.holding and not self.paused:
            status, color = "Stopped for an obstacle", ORANGE
        self.text(status, 672, 105, 15, color, bold=True)
        if terminal or sim.state == "WAITING" or sim.holding:
            reason = sim.status_reason
        elif self.pg.time.get_ticks() < self.notice_until:
            reason = self.notice
        elif self.paused:
            reason = "Press Run to start. Help explains the controls."
        else:
            reason = "Following the blue route to the green goal."
        self.wrapped_text(reason, 672, 132, 296, 13, max_lines=2)
        self.text(f"Time: {m.time_s:.1f} s", 672, 174, 14)
        self.text(f"Speed: {sim.speed:.2f} m/s", 828, 174, 14)
        self.text(f"Replans: {m.replans}", 672, 198, 14)
        if self.layers["estimate"]:
            self.text(f"Pose error: {m.position_error:.3f} m", 828, 198, 13)
        self.line((672, 224), (968, 224))
        self.text("Environment", 672, 236, 15, bold=True)
        for x, (_, label, scenario) in zip((672, 774, 876), ENVIRONMENTS):
            self.button((x, 262, 92, 28), label, "scenario:" + scenario,
                        active=sim.scenario == scenario)
        self.line((672, 302), (968, 302))
        self.text(f"Moving obstacles: {len(sim.world.obstacles)}", 672, 314, 15, bold=True)
        self.button((672, 340, 144, 30), "Spawn", "spawn")
        self.button((824, 340, 144, 30), "Despawn", "despawn")
        self.line((672, 382), (968, 382))
        self.text("Show on map", 672, 394, 15, bold=True)
        for key, label, y in (("trail", "Driven path", 420),
                              ("estimate", "Estimated position", 452),
                              ("costmap", "Obstacle clearance", 484)):
            self.checkbox(key, label, y)

        x = 16
        for color, label in ((BLUE, "Robot / planned route"), (GREEN, "Goal"),
                             ((136, 147, 163), "Wall"), (RED, "Moving obstacle")):
            self.pg.draw.circle(self.canvas, color, (x + 4, 551), 4)
            x += self.text(label, x + 15, 542, 12, MUTED) + 32
        self.text("Left drag: walls / Right click: goal", 720, 542, 12, MUTED)
        self.text("Space: run / pause    N: step    R: reset    E: save report    H: help    Esc: quit", 16, 578, 12, MUTED)
        self.text("1 square = 1 metre", 872, 578, 11, MUTED)
        if self.show_help:
            self.buttons = []
            self.draw_help()

        width, height = self.screen.get_size()
        self.view_scale = min(width / self.WIDTH, height / self.HEIGHT)
        size = round(self.WIDTH * self.view_scale), round(self.HEIGHT * self.view_scale)
        self.view_offset = ((width - size[0]) // 2, (height - size[1]) // 2)
        self.screen.fill(BG)
        self.screen.blit(self.canvas if size == self.canvas.get_size()
                         else self.pg.transform.smoothscale(self.canvas, size), self.view_offset)
        self.pg.display.flip()

    def reset(self, scenario=None):
        self.sim = Simulation(self.sim.seed, scenario or self.sim.scenario)
        self.paused, self.accumulator = True, 0.0
        self._cost_revision, self._cost_surface = None, None
        self.notice, self.notice_until = "", 0
        self.paint = None
        self.painted_cells.clear()

    def export(self):
        path = Path("mission-report.json")
        path.write_text(json.dumps(self.sim.report(), indent=2), encoding="utf-8")
        self.sim.log("SYSTEM", f"Saved {path.name}")
        self.notice = f"Saved {path.name} in the project folder."
        self.notice_until = self.pg.time.get_ticks() + 4000

    def action(self, action):
        if action == "help":
            self.show_help = not self.show_help
            if self.show_help:
                self.paused = True
                self.paint = None
        elif action in ("spawn", "despawn"):
            changed = self.sim.spawn_obstacle() if action == "spawn" else self.sim.despawn_obstacle()
            if changed:
                self.notice = f"Moving obstacle {'added' if action == 'spawn' else 'removed'}. Total: {len(self.sim.world.obstacles)}."
            else:
                self.notice = "Couldn't find a clear spot. Try again or erase walls." if action == "spawn" else "No moving obstacles to remove."
            self.notice_until = self.pg.time.get_ticks() + 3000
        elif action == "run":
            if self.sim.state in ("GOAL_REACHED", "COLLISION"):
                self.reset()
            self.paused = not self.paused
        elif action == "step":
            self.paused = True
            self.sim.step()
        elif action == "reset":
            self.reset()
        elif action == "speed":
            speeds = (0.5, 1.0, 2.0, 4.0)
            self.speed_multiplier = speeds[(speeds.index(self.speed_multiplier) + 1) % len(speeds)]
        elif action.startswith("scenario:"):
            self.reset(action.split(":")[1])
        elif action.startswith("layer:"):
            key = action.split(":")[1]
            self.layers[key] = not self.layers[key]

    def paint_cell(self, point):
        cell = int(point[0]), int(point[1])
        if cell not in self.painted_cells:
            self.painted_cells.add(cell)
            self.sim.edit_obstacle(cell, self.paint)

    def events(self):
        pg = self.pg
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self.running = False
            elif event.type == pg.KEYDOWN:
                if event.key in (pg.K_h, pg.K_F1) or (self.show_help and event.key == pg.K_ESCAPE):
                    self.action("help")
                    continue
                if self.show_help:
                    continue
                if event.key == pg.K_ESCAPE:
                    self.running = False
                mapping = {pg.K_SPACE: "run", pg.K_n: "step", pg.K_r: "reset", pg.K_c: "layer:costmap"}
                if event.key in mapping:
                    self.action(mapping[event.key])
                elif event.key == pg.K_e:
                    self.export()
            elif event.type == pg.MOUSEBUTTONDOWN:
                mouse = self.mouse(event.pos)
                if event.button == 1:
                    button = next((action for rect, action in self.buttons if rect.collidepoint(mouse)), None)
                    if button:
                        self.action(button)
                        continue
                if self.show_help:
                    continue
                point = self.world_point(mouse)
                if point:
                    point_center = int(point[0]) + 0.5, int(point[1]) + 0.5
                    if event.button == 3:
                        self.sim.set_goal(point_center)
                    elif event.button == 1 and pg.key.get_mods() & pg.KMOD_SHIFT:
                        self.sim.set_start(point_center)
                    elif event.button == 1:
                        self.paint = tuple(map(int, point)) not in self.sim.world.static
                        self.painted_cells.clear()
                        self.paint_cell(point)
            elif event.type == pg.MOUSEBUTTONUP and event.button == 1:
                self.paint = None
                self.painted_cells.clear()
            elif event.type == pg.MOUSEMOTION and self.paint is not None:
                point = self.world_point(self.mouse(event.pos))
                if point:
                    self.paint_cell(point)

    def run(self, frames=None, screenshot=None):
        frame_count = 0
        self.draw()
        while self.running:
            elapsed = min(self.clock.tick(60) / 1000.0, 0.1)
            self.events()
            if not self.paused:
                self.accumulator += elapsed * self.speed_multiplier
                while self.accumulator >= 0.04:
                    self.sim.step(0.04)
                    self.accumulator -= 0.04
            else:
                self.accumulator = 0.0
            self.draw()
            frame_count += 1
            if frames is not None and frame_count >= frames:
                break
        if screenshot:
            self.pg.image.save(self.canvas, str(screenshot))
        self.pg.quit()


def main(argv=None):
    parser = argparse.ArgumentParser(description="2D autonomous navigation simulator")
    parser.add_argument("--seed", type=int, default=7, help="reproducible world and sensor seed")
    parser.add_argument("--scenario", choices=("env1", "env2", "env3", "warehouse", "crossing", "open"),
                        default="env1", help="environment; earlier scenario names remain accepted")
    parser.add_argument("--headless", action="store_true", help="run without Pygame or a display")
    parser.add_argument("--steps", type=int, default=5000, help="maximum headless steps (25 Hz)")
    parser.add_argument("--export", type=Path, help="save mission metrics as JSON")
    parser.add_argument("--frames", type=int, help="close the dashboard after N frames")
    parser.add_argument("--screenshot", type=Path, help="save a dashboard PNG on exit")
    args = parser.parse_args(argv)
    if args.steps < 1 or (args.frames is not None and args.frames < 1):
        parser.error("steps and frames must be positive")
    scenario = {alias: name for alias, _, name in ENVIRONMENTS}.get(args.scenario, args.scenario)
    sim = Simulation(seed=args.seed, scenario=scenario)
    if args.headless:
        for _ in range(args.steps):
            sim.step()
            if sim.state in ("GOAL_REACHED", "COLLISION"):
                break
        report = sim.report()
        print(json.dumps(report, indent=2))
    else:
        app = Dashboard(sim)
        app.run(frames=args.frames, screenshot=args.screenshot)
        report = app.sim.report()
    if args.export:
        args.export.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return int(args.headless and sim.state != "GOAL_REACHED")
