# Nav Simulator

WIP navigation simulator experiments in Python.

Right now the repo is focused on `grid.py`: a pygame-based grid navigation simulator that uses A* path planning, dynamic obstacle edits, replanning, and a simple costmap view. Other navigation pieces will be added later.

## Current Demo

`grid.py` opens a 60x60 grid where a robot plans a path from the top-left corner to the bottom-right corner. The simulator starts paused so you can inspect the world, then step into the run.

Features currently included:

- A* path planning on a grid world
- Random obstacle generation on reset
- Left-click obstacle toggling while the app is running
- Automatic replanning when the current path becomes blocked
- Costmap penalties around obstacles
- Visual stats for steps, replans, obstacle changes, and path length

## Requirements

- Python 3.12+
- pygame

Install dependencies:

```bash
pip install -r requirements.txt
```

## Run

```bash
python grid.py
```

## Controls

- `Space`: start or pause the simulation
- Left click: add or remove an obstacle
- `C`: toggle costmap view
- `R`: reset the simulation
- `Esc`: quit

## Project Status

This is a work in progress. The README and repo are intentionally centered on `grid.py` for now; additional simulator and visualization pieces will be added as they mature.
