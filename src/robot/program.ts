// Explores an unknown board and drives to the goal using only northward and
// eastward travel.
//
// Three layers sit on top of each other:
//
//   1. r.grid       occupancy. True means blocked OR unknown; markFree only
//                   ever clears, so knowledge grows and never retracts.
//   2. clearance    four grids saying which body poses fit where, each built by
//                   testing r.grid against a precomputed swept-footprint mask.
//   3. route        a dynamic program over the clearance grids that minimises
//                   90 degree turns, since every monotone route is the same
//                   length and turns are the only cost that varies.

import {
  CELL_CM,
  PIVOT_FROM_REAR_CM,
  PIVOT_TO_FRONT_CM,
  ROBOT_WIDTH_CM,
  SENSOR_FORWARD_CM,
  SENSOR_HALF_CONE_DEG,
  SENSOR_MAX_CM,
  SENSOR_RIGHT_CM,
  SUBCELL_CM,
} from "../config";
import type { Robot } from "./types";

interface Vec2 {
  x: number;
  y: number;
}

interface Cell {
  cx: number;
  cy: number;
}

const NORTH_DEG = 0;
const EAST_DEG = 90;
const QUARTER_TURN_DEG = 90;
const FULL_TURN_DEG = 360;

const EPS = 1e-6;
const HALF_SUBCELL_CM = SUBCELL_CM / 2;

// The body is not centred on the pivot: two thirds of its length trails behind,
// so the rear overhang dominates every clearance figure below.
const SAFETY_CM = 1;
const AHEAD_CM = PIVOT_TO_FRONT_CM + SAFETY_CM;
const BEHIND_CM = PIVOT_FROM_REAR_CM + SAFETY_CM;
const HALF_WIDTH_CM = ROBOT_WIDTH_CM / 2 + SAFETY_CM;
const BODY_HALF_LENGTH_CM = (AHEAD_CM + BEHIND_CM) / 2;
const BODY_CENTRE_OFFSET_CM = (AHEAD_CM - BEHIND_CM) / 2;
const TURN_RADIUS_CM = Math.hypot(BEHIND_CM, HALF_WIDTH_CM);

const SWEEP_STEP_DEG = 1;
const CLEAR_MARGIN_CM = 1.5;
const START_CLEAR_RADIUS_CM = TURN_RADIUS_CM + CELL_CM;
const MIN_STEP_CM = CELL_CM;
const MIN_MOTION_CM = 0.01;
const MAX_STEPS = 500;

// Fine enough that the outermost corner advances a quarter of a sub-cell per
// step. Sampling coarser lets a thin obstacle hide between two orientations and
// be reported as clear.
const TRACE_STEP_DEG = ((SUBCELL_CM / (4 * TURN_RADIUS_CM)) * 180) / Math.PI;

// Sub-cell offsets the body covers while turning between two headings. Fixed
// geometry, so these are traced once here rather than per cell per frame; the
// TODO: bake these tables into flash as const tables in the C++ port
const FACING_NORTH = sweptOffsets(NORTH_DEG, NORTH_DEG);
const FACING_EAST = sweptOffsets(EAST_DEG, EAST_DEG);
const QUARTER_TURN = sweptOffsets(NORTH_DEG, EAST_DEG);
const FULL_TURN = sweptOffsets(0, FULL_TURN_DEG);

export async function run(r: Robot): Promise<void> {
  r.grid.fill(true);
  clearStartZone(r);
  computeClearance(r);

  for (let step = 0; step < MAX_STEPS; step++) {
    await scan(r);
    computeClearance(r);
    if (withinGoal(r.goal(), r.position())) {
      r.log("goal reached");
      return;
    }

    const route = planRoute(r);
    if (route) {
      r.log(route.reachesGoal ? "route to goal" : "route to closer vantage");
      await driveRoute(r, route.cells);
      continue;
    }
    if (!(await exploreOutward(r))) {
      r.log("stuck: no drivable north or east run");
      return;
    }
  }
  r.log("step budget exhausted");
}

// --- clearance ------------------------------------------------------------

function isClear(r: Robot, cx: number, cy: number, mask: Int16Array): boolean {
  for (let k = 0; k < mask.length; k += 2) {
    if (r.grid.get(cx + mask[k], cy + mask[k + 1])) return false;
  }
  return true;
}

/**
 * A quarter turn passes through both fixed headings and a full turn contains a
 * quarter one, so the masks nest and each grid implies the ones below it. That
 * lets the cheaper test short-circuit the dearer one.
 */
function computeClearance(r: Robot): void {
  for (let cy = 0; cy < r.grid.rows; cy++) {
    for (let cx = 0; cx < r.grid.cols; cx++) {
      const corners = isClear(r, cx, cy, QUARTER_TURN);
      r.turn90.set(cx, cy, corners);
      r.reachable.set(cx, cy, corners && isClear(r, cx, cy, FULL_TURN));
      r.driveUp.set(cx, cy, corners || isClear(r, cx, cy, FACING_NORTH));
      r.driveEast.set(cx, cy, corners || isClear(r, cx, cy, FACING_EAST));
    }
  }
}

function sweptOffsets(fromDeg: number, toDeg: number): Int16Array {
  const reach = Math.ceil(TURN_RADIUS_CM / SUBCELL_CM);
  const span = 2 * reach + 1;
  const covered = new Uint8Array(span * span);
  for (let deg = fromDeg; deg <= toDeg + EPS; deg += TRACE_STEP_DEG) {
    const radians = (deg * Math.PI) / 180;
    const sin = Math.sin(radians);
    const cos = Math.cos(radians);
    for (let dy = -reach; dy <= reach; dy++) {
      for (let dx = -reach; dx <= reach; dx++) {
        const index = (dy + reach) * span + (dx + reach);
        if (!covered[index] && bodyCoversCell(dx, dy, sin, cos)) {
          covered[index] = 1;
        }
      }
    }
  }

  const offsets: Vec2[] = [];
  for (let dy = -reach; dy <= reach; dy++) {
    for (let dx = -reach; dx <= reach; dx++) {
      if (covered[(dy + reach) * span + (dx + reach)]) offsets.push({ x: dx, y: dy });
    }
  }
  // Nearest first, so something blocked beside the pivot rejects on the first
  // few entries rather than after walking the whole mask.
  offsets.sort((a, b) => a.x * a.x + a.y * a.y - (b.x * b.x + b.y * b.y));

  const packed = new Int16Array(offsets.length * 2);
  for (let k = 0; k < offsets.length; k++) {
    packed[2 * k] = offsets[k].x;
    packed[2 * k + 1] = offsets[k].y;
  }
  return packed;
}

/** Separating-axis overlap between the body at a heading and one sub-cell. */
function bodyCoversCell(dx: number, dy: number, sin: number, cos: number): boolean {
  const absSin = Math.abs(sin);
  const absCos = Math.abs(cos);
  const offX = dx * SUBCELL_CM - sin * BODY_CENTRE_OFFSET_CM;
  const offY = dy * SUBCELL_CM - cos * BODY_CENTRE_OFFSET_CM;
  const cellReach = HALF_SUBCELL_CM * (absSin + absCos);

  const along = offX * sin + offY * cos;
  if (Math.abs(along) > BODY_HALF_LENGTH_CM + cellReach) return false;
  const across = offX * cos - offY * sin;
  if (Math.abs(across) > HALF_WIDTH_CM + cellReach) return false;

  const bodySpanX = BODY_HALF_LENGTH_CM * absSin + HALF_WIDTH_CM * absCos;
  if (Math.abs(offX) > bodySpanX + HALF_SUBCELL_CM) return false;
  const bodySpanY = BODY_HALF_LENGTH_CM * absCos + HALF_WIDTH_CM * absSin;
  return Math.abs(offY) <= bodySpanY + HALF_SUBCELL_CM;
}

// --- route planning -------------------------------------------------------

type Arrival = "east" | "north";

/** Turns needed to reach each cell of the search box, per arrival heading. */
interface TurnCosts {
  east: number[][];
  north: number[][];
}

interface Candidate {
  i: number;
  j: number;
  arrival: Arrival;
  turns: number;
  distanceCm: number;
}

interface Route {
  cells: Cell[];
  reachesGoal: boolean;
}

function centreOf(index: number): number {
  return (index + 0.5) * SUBCELL_CM;
}

function withinGoal(goal: { x: number; y: number; radius: number }, p: Vec2): boolean {
  return Math.hypot(goal.x - p.x, goal.y - p.y) <= goal.radius;
}

function cellAt(r: Robot, x: number, y: number): Cell {
  return {
    cx: Math.max(0, Math.min(r.grid.cols - 1, Math.floor(x / SUBCELL_CM))),
    cy: Math.max(0, Math.min(r.grid.rows - 1, Math.floor(y / SUBCELL_CM))),
  };
}

// How far past the goal's row or column a straight run can be while still hitting the goal
function goalOvershootCm(radius: number, offAxisCm: number): number {
  const remaining = radius * radius - offAxisCm * offAxisCm;
  return remaining > 0 ? Math.sqrt(remaining) : 0;
}

function minTurnCosts(r: Robot, start: Cell, width: number, height: number): TurnCosts {
  const east: number[][] = [];
  const north: number[][] = [];
  for (let i = 0; i <= width; i++) {
    east[i] = new Array<number>(height + 1).fill(Infinity);
    north[i] = new Array<number>(height + 1).fill(Infinity);
  }
  east[0][0] = 0;
  north[0][0] = 0;

  const fitsEast = (i: number, j: number) => r.driveEast.get(start.cx + i, start.cy + j);
  const fitsNorth = (i: number, j: number) => r.driveUp.get(start.cx + i, start.cy + j);
  const corners = (i: number, j: number) => r.turn90.get(start.cx + i, start.cy + j);

  for (let j = 0; j <= height; j++) {
    for (let i = 0; i <= width; i++) {
      if (i === 0 && j === 0) continue;
      if (i > 0 && fitsEast(i, j)) {
        east[i][j] = Math.min(
          east[i - 1][j],
          corners(i - 1, j) ? north[i - 1][j] + 1 : Infinity,
        );
      }
      if (j > 0 && fitsNorth(i, j)) {
        north[i][j] = Math.min(
          north[i][j - 1],
          corners(i, j - 1) ? east[i][j - 1] + 1 : Infinity,
        );
      }
    }
  }
  return { east, north };
}

/** Walk the costs back to the start, recovering each step's arrival heading. */
function backtrack(costs: TurnCosts, start: Cell, target: Candidate): Cell[] {
  const cells: Cell[] = [];
  let i = target.i;
  let j = target.j;
  let arrival = target.arrival;
  while (true) {
    cells.push({ cx: start.cx + i, cy: start.cy + j });
    if (i === 0 && j === 0) break;
    if (arrival === "east") {
      arrival = costs.east[i - 1][j] === costs.east[i][j] ? "east" : "north";
      i--;
    } else {
      arrival = costs.north[i][j - 1] === costs.north[i][j] ? "north" : "east";
      j--;
    }
  }
  return cells.reverse();
}

/**
 * Search the box between the robot and the far edge of the goal disc, and
 * return the fewest-turn route into the goal. Failing that, return a route to
 * the nearest vantage point the robot can sweep from, which reveals more map
 * for the next attempt.
 */
function planRoute(r: Robot): Route | null {
  const start = r.cell();
  if (!r.turn90.get(start.cx, start.cy)) return null;

  const goal = r.goal();
  const goalCentre = cellAt(r, goal.x, goal.y);
  // Reaching past the goal's own row and column matters: rounding an obstacle
  // often needs that room, any cell inside the disc finishes the run just as
  // well, and monotone travel gets no second chance at it.
  const pad = Math.ceil(goal.radius / SUBCELL_CM);
  const width = Math.min(r.grid.cols - 1, goalCentre.cx + pad) - start.cx;
  const height = Math.min(r.grid.rows - 1, goalCentre.cy + pad) - start.cy;
  if (width < 0 || height < 0) return null;

  const costs = minTurnCosts(r, start, width, height);
  const distanceTo = (cx: number, cy: number) =>
  Math.hypot(goal.x - centreOf(cx), goal.y - centreOf(cy));
  const startDistance = distanceTo(start.cx, start.cy);

  let bestGoal: Candidate | null = null;
  let bestVantage: Candidate | null = null;

  for (let i = 0; i <= width; i++) {
    for (let j = 0; j <= height; j++) {
      const cx = start.cx + i;
      const cy = start.cy + j;
      const turns = Math.min(costs.east[i][j], costs.north[i][j]);
      if (!Number.isFinite(turns)) continue;
      const candidate: Candidate = {
        i,
        j,
        arrival: costs.east[i][j] <= costs.north[i][j] ? "east" : "north",
        turns,
        distanceCm: distanceTo(cx, cy),
      };

      if (withinGoal(goal, { x: centreOf(cx), y: centreOf(cy) })) {
        if (!bestGoal || turns < bestGoal.turns) bestGoal = candidate;
        continue;
      }

      // Stopping past the goal strands the robot, and a vantage is only useful
      // if it can sweep and if it actually closes the gap — otherwise the robot
      // oscillates between two cells that each look closer from the other.
      
      if (cx > goalCentre.cx || cy > goalCentre.cy) continue;
      if (!r.turn90.get(cx, cy)) continue;
      if (candidate.distanceCm >= startDistance - EPS) continue;
      if (
        !bestVantage ||
        candidate.distanceCm < bestVantage.distanceCm - EPS ||
        (candidate.distanceCm < bestVantage.distanceCm + EPS &&
          turns < bestVantage.turns)
      ) {
        bestVantage = candidate;
      }
    }
  }

  const target = bestGoal ?? bestVantage;
  if (!target) return null;
  return { cells: backtrack(costs, start, target), reachesGoal: bestGoal !== null };
}

// --- driving --------------------------------------------------------------

/** Reduce a cell path to its direction changes. */
function cornerCells(cells: Cell[]): Cell[] {
  if (cells.length <= 2) return cells.slice();
  const corners: Cell[] = [cells[0]];
  for (let k = 1; k < cells.length - 1; k++) {
    const arrivedEast = cells[k].cx !== cells[k - 1].cx;
    const leavesEast = cells[k + 1].cx !== cells[k].cx;
    if (arrivedEast !== leavesEast) corners.push(cells[k]);
  }
  corners.push(cells[cells.length - 1]);
  return corners;
}

// Drive the route, stopping at the first cell a full sweep is possible from
// to reveal more of the map. Corners that only permit a quarter turn can be traversed on a path to reach a sweepable position

async function driveRoute(r: Robot, cells: Cell[]): Promise<void> {
  const corners = cornerCells(cells);
  for (let k = 1; k < corners.length; k++) {
    const from = corners[k - 1];
    const to = corners[k];
    if (to.cx !== from.cx) {
      await turnScanning(r, EAST_DEG);
      await driveScanning(r, centreOf(to.cx) - r.position().x);
    } else {
      await turnScanning(r, NORTH_DEG);
      await driveScanning(r, centreOf(to.cy) - r.position().y);
    }
    if (r.reachable.get(to.cx, to.cy)) return;
  }
}

// Take the longer of the two axis runs purely to gain a fresh vantage point
async function exploreOutward(r: Robot): Promise<boolean> {
  const from = r.position();
  const goal = r.goal();
  const northLimit =
    goal.y + goalOvershootCm(goal.radius, from.x - goal.x) - from.y;
  const eastLimit =
    goal.x + goalOvershootCm(goal.radius, from.y - goal.y) - from.x;
  const north = drivableRun(r, from, 0, 1, Math.max(0, northLimit));
  const east = drivableRun(r, from, 1, 0, Math.max(0, eastLimit));

  if (north < MIN_STEP_CM && east < MIN_STEP_CM) return false;
  if (north >= east) {
    await turnScanning(r, NORTH_DEG);
    await driveScanning(r, north);
  } else {
    await turnScanning(r, EAST_DEG);
    await driveScanning(r, east);
  }
  return true;
}

/**
 * Furthest distance along an axis that the body fits the whole way to and can
 * sweep from on arrival. The corridor itself needs only the drive clearance;
 * demanding sweep clearance along its whole length strands the robot beside any
 * obstacle sitting just inside TURN_RADIUS_CM.
 */
function drivableRun(
  r: Robot,
  origin: Vec2,
  dirX: number,
  dirY: number,
  maxCm: number,
): number {
  const fits = dirY !== 0 ? r.driveUp : r.driveEast;
  let furthest = 0;
  for (let d = 0; d <= maxCm + EPS; d += HALF_SUBCELL_CM) {
    const cx = Math.floor((origin.x + dirX * d) / SUBCELL_CM);
    const cy = Math.floor((origin.y + dirY * d) / SUBCELL_CM);
    if (cx < 0 || cy < 0 || cx >= r.grid.cols || cy >= r.grid.rows) break;
    if (!fits.get(cx, cy)) break;
    if (r.turn90.get(cx, cy)) furthest = d;
  }
  return furthest;
}

async function whileMoving(r: Robot, motion: Promise<void>): Promise<void> {
  let moving = true;
  const settle = () => {
    moving = false;
  };
  motion.then(settle, settle);
  while (moving) {
    await r.step();
    markFree(r);
  }
  await motion; // rethrows a cancellation, e.g. the goal being captured
}

async function driveScanning(r: Robot, cm: number): Promise<void> {
  if (cm <= MIN_MOTION_CM) return;
  await whileMoving(r, r.driveFor(cm));
}

async function turnScanning(r: Robot, headingDeg: number): Promise<void> {
  await whileMoving(r, r.turnTo(headingDeg));
}

// --- SENSING ---

// scan as wide as the current tile allows
async function scan(r: Robot): Promise<void> {
  markFree(r);
  const here = r.cell();
  if (r.reachable.get(here.cx, here.cy)) {
    await sweep(r, FULL_TURN_DEG);
  } else if (r.turn90.get(here.cx, here.cy)) {
    await turnScanning(r, NORTH_DEG);
    await sweep(r, QUARTER_TURN_DEG);
  }
}

// scan an arbitrary number of degrees
async function sweep(r: Robot, arcDeg: number): Promise<void> {
  for (let turned = 0; turned < arcDeg; turned += SWEEP_STEP_DEG) {
    await r.turn(SWEEP_STEP_DEG);
    markFree(r);
  }
}

// helper to get the world-space coordinates of the ir sensor
function sensorOrigin(r: Robot): Vec2 {
  const p = r.position();
  const radians = (r.heading() * Math.PI) / 180;
  const forward = { x: Math.sin(radians), y: Math.cos(radians) };
  const right = { x: Math.cos(radians), y: -Math.sin(radians) };
  return {
    x: p.x + forward.x * SENSOR_FORWARD_CM + right.x * SENSOR_RIGHT_CM,
    y: p.y + forward.y * SENSOR_FORWARD_CM + right.y * SENSOR_RIGHT_CM,
  };
}

// check if point is within sensor zone; called on sub-cell corners
function inWedge(
  dx: number,
  dy: number,
  forwardX: number,
  forwardY: number,
  reachSq: number,
  cosHalfConeSq: number,
): boolean {
  const distanceSq = dx * dx + dy * dy;
  if (distanceSq > reachSq) return false;
  const along = dx * forwardX + dy * forwardY;
  if (along <= 0) return distanceSq === 0;
  return along * along >= cosHalfConeSq * distanceSq;
}

// Two rows of the sub-cell corner lattice, reused between calls. Each lattice
// point is shared by four cells, so sweeping row pairs tests it once.
let cornersBelow = new Uint8Array(0);
let cornersAbove = new Uint8Array(0);


// Clear every sub-cell entirely inside the sensor zone
function markFree(r: Robot): void {
  const origin = sensorOrigin(r);
  const measured = r.distance();
  const reach = Number.isFinite(measured)
    ? measured - CLEAR_MARGIN_CM
    : SENSOR_MAX_CM;
  if (reach <= 0) return;

  const radians = (r.heading() * Math.PI) / 180;
  const forwardX = Math.sin(radians);
  const forwardY = Math.cos(radians);
  const reachSq = reach * reach;
  const cosHalfCone = Math.cos((SENSOR_HALF_CONE_DEG * Math.PI) / 180);
  const cosHalfConeSq = cosHalfCone * cosHalfCone;

  const lo = cellAt(r, origin.x - reach, origin.y - reach);
  const hi = cellAt(r, origin.x + reach, origin.y + reach);
  const width = hi.cx - lo.cx + 2; // one more corner column than cell columns
  if (cornersBelow.length < width) {
    cornersBelow = new Uint8Array(width);
    cornersAbove = new Uint8Array(width);
  }

  const fillRow = (row: Uint8Array, cy: number): void => {
    const dy = cy * SUBCELL_CM - origin.y;
    for (let k = 0; k < width; k++) {
      const dx = (lo.cx + k) * SUBCELL_CM - origin.x;
      row[k] = inWedge(dx, dy, forwardX, forwardY, reachSq, cosHalfConeSq) ? 1 : 0;
    }
  };

  let below = cornersBelow;
  let above = cornersAbove;
  fillRow(below, lo.cy);
  for (let cy = lo.cy; cy <= hi.cy; cy++) {
    fillRow(above, cy + 1);
    for (let k = 0; k + 1 < width; k++) {
      if (below[k] && below[k + 1] && above[k] && above[k + 1]) {
        r.grid.set(lo.cx + k, cy, false);
      }
    }
    const reused = below;
    below = above;
    above = reused;
  }
}

// clear the area directly surrounding the start zone; this assumes the robot can make a full 360 spin from the starting position
function clearStartZone(r: Robot): void {
  const p = r.position();
  const lo = cellAt(r, p.x - START_CLEAR_RADIUS_CM, p.y - START_CLEAR_RADIUS_CM);
  const hi = cellAt(r, p.x + START_CLEAR_RADIUS_CM, p.y + START_CLEAR_RADIUS_CM);
  const radiusSq = START_CLEAR_RADIUS_CM * START_CLEAR_RADIUS_CM;
  for (let cy = lo.cy; cy <= hi.cy; cy++) {
    for (let cx = lo.cx; cx <= hi.cx; cx++) {
      const dx = centreOf(cx) - p.x;
      const dy = centreOf(cy) - p.y;
      if (dx * dx + dy * dy <= radiusSq) r.grid.set(cx, cy, false);
    }
  }
}
