# voyager-nasa-apps-challenge-project
for voyager nasa apps challenge project
# MarsEye — NASA Space Apps Challenge 2026

**Challenge:** Interplanetary Survival Guide: Martian Map

## Project Overview

MarsEye is a 3D Mars exploration and mission-planning prototype that combines NASA terrain datasets with risk-aware pathfinding algorithms.

The system aims to help future astronauts explore scientifically valuable locations by comparing shortest routes with terrain-aware alternatives.

## Pilot Region

Gale Crater, Mars.

## Key Features

- 3D God's Eye View of Martian terrain
- NASA terrain and geological datasets
- Slope-based terrain risk analysis
- A* shortest-path and risk-aware navigation
- Route comparison and scientific exploration targets

## Current Development Status

**Initial navigation prototype completed.**

The A* pathfinding algorithm has been tested on synthetic terrain.

| Metric | Shortest Route | Risk-Aware Route |
|---|---:|---:|
| Distance | 178.00 m | 219.42 m |
| Maximum slope | 29.70° | 9.98° |

These are synthetic test results, **not measurements from Gale Crater**.

The next milestone is to integrate actual NASA elevation data and visualize both routes on a 3D Martian terrain model.
