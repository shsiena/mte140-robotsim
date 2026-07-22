// =============================================================================
//  YOUR ROBOT ALGORITHM GOES HERE.
//
//  Edit this file, then hit "Run" in the app (Vite hot-reloads it and resets
//  the sim). Everything is in centimetres / degrees so it ports to the VEX IQ
//  with minimal changes. See src/robot/types.ts for the full Robot API.
//
//  Conventions:
//    - World: origin bottom-left, +x right, +y up (cm).
//    - Heading: 0° = north (+y), clockwise positive.
//    - r.grid is your own boolean world-model, drawn live as a cell overlay.
//
//  The example below is a simple reactive obstacle-avoider — replace it with
//  your algorithm.
// =============================================================================

import {
  CELL_CM,
  SENSOR_HALF_CONE_DEG,
  SENSOR_FORWARD_CM,
  SENSOR_RIGHT_CM,
  SENSOR_MAX_CM,
} from "../config";
import type { Robot } from "./types";

// export async function run(r: Robot): Promise<void> {
//   const goal = r.goal();
//   const SAFE_CM = 12; // stop / steer when something is this close
//
//   while (true) {
//     // Point roughly at the goal.
//     const p = r.position();
//     await r.turnTo(bearingTo(p, goal));
//
//     // Obstacle ahead? Note it in the world-model and steer around it.
//     if (r.distance() < SAFE_CM) {
//       markSensedCell(r);
//       while (r.distance() < SAFE_CM) {
//         await r.turn(15); // rotate right until the path clears
//         markSensedCell(r);
//       }
//       await r.driveFor(6); // sidestep, then re-aim next loop
//       continue;
//     }
//
//     // Drive forward while watching the sensor; the sim ends the run on its own
//     // when the robot's centre reaches the goal.
//     r.setDrive(15);
//     while (r.distance() >= SAFE_CM) {
//       await r.step();
//     }
//     r.stop();
//   }
// }

// ===========================================================================
//  Approach B — occupancy mapping via an inverse sensor model.
//
//  Convention: grid cell `true` = potential obstacle / unknown,
//              grid cell `false` = known clear.
//
//  Start with everything unknown, carve a known-clear bubble around the start,
//  then rotate in place sweeping the IR sensor. For every measurement, any cell
//  that is inside the cone AND nearer than the hit distance is provably empty,
//  so mark it clear. Cells at/beyond the hit are never touched, so obstacle
//  faces and the shadows behind them stay "unknown" — exactly the map a
//  pathfinder should treat as blocked.
//
//  No ray fan: each cell is tested directly against the cone, so there are no
//  divergence gaps at range.
//
//  (Next step, not done here: run pathfinding over the resulting grid.)
// ===========================================================================

interface Vec2 {
  x: number;
  y: number;
}

const SWEEP_STEP_DEG = 1; // sensor sweep resolution
const SWEEP_TOTAL_DEG = 360; // a full turn maps everything around the start
const CLEAR_MARGIN_CM = 1.5; // stop clearing just short of the hit surface
const START_CLEAR_RADIUS_CM = 14; // guaranteed-clear bubble around the start

export async function run(r: Robot): Promise<void> {
  // 1. Everything is a potential obstacle until proven clear.
  r.grid.fill(true);

  // 2. The robot can't be sitting inside an obstacle — clear its start bubble.
  clearStartZone(r);

  // 3. Sweep in place, clearing visible free space at each sample.
  markFree(r);
  for (let swept = 0; swept < SWEEP_TOTAL_DEG; swept += SWEEP_STEP_DEG) {
    await r.turn(SWEEP_STEP_DEG);
    markFree(r);
  }

  r.log("sweep complete");
}

/** World position of the IR sensor origin for the current pose. */
function sensorOrigin(r: Robot): Vec2 {
  const p = r.position();
  const rad = (r.heading() * Math.PI) / 180;
  const fwd: Vec2 = { x: Math.sin(rad), y: Math.cos(rad) };
  const right: Vec2 = { x: Math.cos(rad), y: -Math.sin(rad) };
  return {
    x: p.x + fwd.x * SENSOR_FORWARD_CM + right.x * SENSOR_RIGHT_CM,
    y: p.y + fwd.y * SENSOR_FORWARD_CM + right.y * SENSOR_RIGHT_CM,
  };
}

/** Smallest signed angle a->b in (-180, 180]. */
function angleDiff(a: number, b: number): number {
  let d = (((b - a) % 360) + 360) % 360;
  if (d > 180) d -= 360;
  return d;
}

/**
 * Inverse sensor model for one measurement: every cell inside the cone and
 * nearer than the hit distance is provably empty -> mark it clear.
 */
function markFree(r: Robot): void {
  const o = sensorOrigin(r);
  const heading = r.heading();
  const d = r.distance();
  // How far we can trust "free" along the sensor: up to the hit (minus a
  // margin so we don't erase the obstacle face), or full range if nothing hit.
  const reach = Number.isFinite(d) ? d - CLEAR_MARGIN_CM : SENSOR_MAX_CM;
  if (reach <= 0) return;

  // Square bounding box around the reachable area; the cone test trims it.
  const lo = clampCell(o.x - reach, o.y - reach, r);
  const hi = clampCell(o.x + reach, o.y + reach, r);
  const reach2 = reach * reach;

  for (let cy = lo.cy; cy <= hi.cy; cy++) {
    for (let cx = lo.cx; cx <= hi.cx; cx++) {
      const dx = (cx + 0.5) * CELL_CM - o.x;
      const dy = (cy + 0.5) * CELL_CM - o.y;
      if (dx * dx + dy * dy > reach2) continue; // out of range
      const bearing = (Math.atan2(dx, dy) * 180) / Math.PI; // 0=N, CW+
      if (Math.abs(angleDiff(heading, bearing)) <= SENSOR_HALF_CONE_DEG) {
        r.grid.set(cx, cy, false); // known clear
      }
    }
  }
}

/** Clear a small disc of cells around the robot's start position. */
function clearStartZone(r: Robot): void {
  const p = r.position();
  const lo = clampCell(p.x - START_CLEAR_RADIUS_CM, p.y - START_CLEAR_RADIUS_CM, r);
  const hi = clampCell(p.x + START_CLEAR_RADIUS_CM, p.y + START_CLEAR_RADIUS_CM, r);
  const rad2 = START_CLEAR_RADIUS_CM * START_CLEAR_RADIUS_CM;
  for (let cy = lo.cy; cy <= hi.cy; cy++) {
    for (let cx = lo.cx; cx <= hi.cx; cx++) {
      const dx = (cx + 0.5) * CELL_CM - p.x;
      const dy = (cy + 0.5) * CELL_CM - p.y;
      if (dx * dx + dy * dy <= rad2) r.grid.set(cx, cy, false);
    }
  }
}

/** Clamp a world point (cm) to valid cell indices. */
function clampCell(x: number, y: number, r: Robot): { cx: number; cy: number } {
  return {
    cx: Math.max(0, Math.min(r.grid.cols - 1, Math.floor(x / CELL_CM))),
    cy: Math.max(0, Math.min(r.grid.rows - 1, Math.floor(y / CELL_CM))),
  };
}

// ---- previous attempt, kept verbatim (commented out) ----
/*
export async function run (r: Robot): Promise<void> {
  // const goal = r.goal();
  
  const DIST_THRESHHOLD = 1; // cm
  const SENSOR_SPREAD_ANGLE = 12.09; // degrees

  r.grid.fill(true);

  let lastDistance = Infinity;
  let minDistance = Infinity;
  let startHeading = 0;
  let obstacle: boolean = false;

  while (r.heading() < 90) {
    await r.turn(0.5);
    let dist = r.distance();
    
    if (dist < SENSOR_MAX_CM + 1) {
      if (!obstacle) {
        startHeading = r.heading();
        obstacle = true;
        minDistance = dist;
      } else {
        minDistance = Math.min(minDistance, dist);
      }

    } else { // obstacle passed
      if (obstacle) {
        markDanger(
          { 
            x: r.position().x,
            y: r.position().y
          },
          startHeading + SENSOR_SPREAD_ANGLE, 
          r.heading() - SENSOR_SPREAD_ANGLE,
          minDistance,
          r,
          20
        );
        obstacle = false;
        minDistance = Infinity;
      }
    }

    // if (obstacle) {
    //   minDistance = Math.min(dist, minDistance);
    // }
    //
    // if (Math.abs(lastDistance - dist) > DIST_THRESHHOLD) {
    //   r.log("MARK DANGER")
    //   markDanger(
    //     { 
    //       x: r.position().x,
    //       y: r.position().y
    //     },
    //     startHeading + SENSOR_SPREAD_ANGLE, 
    //     r.heading() - SENSOR_SPREAD_ANGLE,
    //     dist,
    //     r,
    //     0.5
    //   );
    // }

    lastDistance = dist;
  }

}

interface Vec2 { x: number; y: number; }

function markDanger(start: Vec2, startHeading: number, endHeading: number, distance: number, r: Robot, segments: number) {
  let segDeg = (endHeading - startHeading) / segments;
  for (let i = 0; i <= segments; i++) {
    markRayCells(r, start, startHeading + i * segDeg, distance + SENSOR_FORWARD_CM);
  }
}

function markRayCells(
  r: Robot,
  start: Vec2,           // world-space origin
  headingDeg: number,    // 0 = up, clockwise positive, degrees
  offset: number,        // dead lead-in; no marking happens across this span
  maxSteps = 100_000,    // hard cap so a bad ray can't spin forever
): void {

  if (r.grid.rows === 0 || r.grid.cols === 0) return;

  const rad = (headingDeg * Math.PI) / 180;
  const dir: Vec2 = { x: Math.sin(rad), y: Math.cos(rad) };

  const p0: Vec2 = {
    x: start.x + dir.x * offset,
    y: start.y + dir.y * offset,
  };

  let cellX = Math.floor(p0.x / CELL_CM);
  let cellY = Math.floor(p0.y / CELL_CM);

  const stepX = Math.sign(dir.x); // -1, 0, or +1
  const stepY = Math.sign(dir.y);

  const tDeltaX = dir.x !== 0 ? Math.abs(CELL_CM / dir.x) : Infinity;
  const tDeltaY = dir.y !== 0 ? Math.abs(CELL_CM / dir.y) : Infinity;

  const firstBoundary = (cell: number, step: number) =>
    (step > 0 ? cell + 1 : cell) * CELL_CM;

  let tMaxX = dir.x !== 0 ? (firstBoundary(cellX, stepX) - p0.x) / dir.x : Infinity;
  let tMaxY = dir.y !== 0 ? (firstBoundary(cellY, stepY) - p0.y) / dir.y : Infinity;

  for (let i = 0; i < maxSteps; i++) {
    if (
      (stepX > 0 && cellX >= r.grid.cols) || (stepX < 0 && cellX < 0) ||
      (stepY > 0 && cellY >= r.grid.rows) || (stepY < 0 && cellY < 0)
    ) break;

    if (cellX >= 0 && cellX < r.grid.cols && cellY >= 0 && cellY < r.grid.rows) {
      // r.grid.set(cellX, r.grid.rows - 1 - cellY, false);
      r.grid.set(cellX, cellY, false);
    }

    if (tMaxX < tMaxY) {
      tMaxX += tDeltaX;
      cellX += stepX;
    } else {
      tMaxY += tDeltaY;
      cellY += stepY;
    }
  }
}
*/
// ---- end previous attempt ----

// /** Compass bearing (0=N, CW+) from point a to point b. */
// function bearingTo(a: { x: number; y: number }, b: { x: number; y: number }): number {
//   const deg = (Math.atan2(b.x - a.x, b.y - a.y) * 180) / Math.PI;
//   return (deg % 360 + 360) % 360;
// }

// /** Project the current IR hit onto the grid and flag that cell in the overlay. */
// function markSensedCell(r: Robot): void {
//   const d = r.distance();
//   if (!Number.isFinite(d)) return;
//   const p = r.position();
//   const rad = (r.heading() * Math.PI) / 180;
//   const hx = p.x + Math.sin(rad) * d;
//   const hy = p.y + Math.cos(rad) * d;
//   const { cellCm } = r.gridSize();
//   r.grid.set(Math.floor(hx / cellCm), Math.floor(hy / cellCm), true);
// }
