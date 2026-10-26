# How the simulator works

The main idea is: **find a route, estimate where the robot is, steer along that
route, and check that the next movement is safe.** Repeat while the world moves.

## The files

Run `grid.py`. It calls `main()` in `navsim/app.py`, which creates a `Simulation`
and opens the Pygame dashboard. In a headless run, it steps the same `Simulation`
without opening a window. The algorithms are shared by both modes.

`viz.py` is a separate older experiment with its own robot and window. It is not
part of this execution path. Its robot simply turns toward a goal and drives;
running it will not show the current navigation project.

## One simulation step

The display renders up to 60 frames a second. Motion advances in fixed
0.04-second steps through `Simulation.step()` in `navsim/simulation.py`:

1. **Check whether the route needs updating.** A wall edit or adding/removing a
   moving obstacle triggers planning immediately during an active mission;
   otherwise the route is refreshed every 1.2
   seconds. `Simulation.replan()` calls `AStarPlanner.plan()` using the estimated
   robot position and selected goal.
2. **Choose a movement.** `WaypointController.command()` looks a short distance
   along the route and returns forward speed and turn rate. The robot slows as
   it reaches the goal and can turn in place for a sharp bend.
3. **Check that movement before executing it.** `_command_safe()` rolls the
   command forward for 0.6 seconds. `World.collision_free()` checks the whole
   circular robot body along the motion, not just its start and end points.
   The simulator tries a slower speed or stops if the command is unsafe.
4. **Move the world.** The robot advances and the red obstacles move and reflect
   off fixed walls. An independent check records actual contact, including a
   moving obstacle striking a robot that has already stopped.
5. **Update the pose estimate.** `PoseEKF.predict()` uses noisy simulated
   odometry each step. Every 0.2 seconds, `PoseEKF.update()` corrects the estimate
   using a noisy measurement of x, y, and heading. EKF means extended Kalman
   filter; it is used because turning motion is nonlinear.
6. **Refresh the occupancy and display data.** Every 0.2 seconds,
   `World.occupancy()` records cells covered by fixed walls and moving obstacles.
   The dashboard draws the current world, route, and status.

The planner and waypoint controller use the **estimated pose**. The simulator
also knows the **true pose**, which it needs to simulate sensors, draw the actual
robot, measure estimation error, and independently check contact.

## How A* and occupancy fit together

A* searches a grid with horizontal, vertical, and diagonal neighbors. It rejects
walls, cells without enough room for the robot, and diagonal moves that cut
through a blocked corner. Cells near walls cost a little more, so the route
prefers clearance. After search, collision-checked shortcuts reduce unnecessary
waypoints.

The first search reserves a little more room than the physical body needs to
allow for position-estimation error. If it fails, a second search uses the
physical body radius. This fixes cases where the preferred margin rejected a
usable narrow route; it does not allow driving through walls. Both searches
keep body collision and moving-obstacle checks.

The moving occupancy grid is a snapshot of **where things are now**. The planner
refreshes its wall clearance when you edit the map and includes the current
moving-obstacle footprints at every plan. It also checks motion geometrically;
the occupancy set is not the only collision test.

## What obstacle prediction means here

Each red circle has a known position and velocity. `World.predictions()` advances
that motion into the next two seconds, including reflections from fixed walls.
The planner adds costs near predicted positions and rejects near-term path
segments that conflict with a moving obstacle. The safety monitor checks the
predicted relative motion of the robot and obstacles during a proposed command.

These calculations stay active even though the prediction lines and their
checkbox have been removed. There is no neural network: the forecast assumes
the obstacle continues its programmed motion. Obstacles do not react to each
other, and the planner is not learning their velocities from sensor data.

Use **Spawn** under **Moving obstacles** to add a red circle at a random clear
spot, with random direction and speed. Use **Despawn** to remove one random red
circle. The count updates after each change. Both buttons work while paused or
running and refresh occupancy and the active route immediately. Spawn reports when
the map has no room; Despawn does nothing when there are no movers. **Reset**
restores the selected environment's original eight movers. A finished mission
stays stopped until you select **Restart**.

## What the map and bottom information mean

The **blue route** is the plan ahead. **Driven path** is the record behind the
robot; the earlier label "Robot trail" meant exactly that. Showing or hiding it
only changes the drawing. **Estimated position** shows the filter's estimate,
which may differ slightly from the solid robot because the sensors are noisy.

The bottom color key identifies robot/planned route, goal, walls, and moving
obstacles. The second line lists keyboard shortcuts. Status explanations now
appear at the top of the right sidebar, including what to do when the robot
cannot continue. **Replans** counts route calculations after the first one; it
can rise even while navigation is working normally because the planner
refreshes periodically. Opening Help with its button, H, or F1 pauses the run;
close it and press Run to continue.

If you opened `viz.py`, its different bottom panel reads **Mode**, **Action**,
**Distance**, and **Heading error**. Action describes its simple turning/forward
controller; distance is straight-line distance to its goal, and heading error
is the difference between its facing direction and the goal direction in
radians. Use `grid.py` to see the full project.

## Small experiments

- **Planning and replanning:** press Run, then draw a wall across part of the
  blue route while leaving another passage. The blue route should change.
- **Dynamic occupancy and prediction:** watch red obstacles cross the route.
  Their occupied cells change as they move; planning and the safety monitor
  account for where they will move next. Press Spawn to add more traffic, or
  Despawn to remove a random moving obstacle.
- **Kalman tracking:** enable Estimated position. Its marker follows the robot
  with small differences because it uses noisy measurements. The estimate is
  used for navigation even while its marker is hidden.
- **Waypoint control:** give the robot a goal around a wall. Its heading changes
  as it follows the route and it slows near the goal.
- **Blocked route:** completely close a passage. The robot should stop and retry
  instead of driving through the wall. Erase part of the wall while running to
  let it find a route again.

A temporary stop in traffic or a stop when no route exists is expected. A
persistently stopped robot with a usable route can indicate a limitation of
this controller and planner combination. It cannot guarantee progress through
every layout you draw, and contact with a moving obstacle remains possible.
