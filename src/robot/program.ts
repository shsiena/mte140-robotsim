
import {
  CELL_CM,
  PIVOT_FROM_REAR_CM,
  PIVOT_TO_FRONT_CM,
  ROBOT_WIDTH_CM,
  SENSOR_HALF_CONE_DEG,
  SENSOR_FORWARD_CM,
  SENSOR_RIGHT_CM,
  SENSOR_MAX_CM,
} from "../config";
import type { Robot } from "./types";


interface Vec2 {
  x: number;
  y: number;
}

const SWEEP_STEP_DEG = 1; // sensor sweep resolution
const SWEEP_TOTAL_DEG = 360; // full spin at each waypoint before re-planning
const CLEAR_MARGIN_CM = 1.5; // stop clearing just short of the hit surface
// Guaranteed-clear bubble around the start. Must cover the footprint at every
// heading (hypot(BEHIND, HALF_WID)) or the robot cannot pick an opening move.
const START_CLEAR_RADIUS_CM = 24;

// Every clearance test compares the footprint against blocked CELL CENTRES, so
// each extent carries two allowances beyond the body itself: half a cell
// diagonal because solid material can sit that far past the nearest cell the
// map calls blocked, and half a cell diagonal because the pivot can sit that
// far from the centre of the cell it is in. Plus a plain safety buffer.
const CELL_HALF_DIAGONAL_CM = (CELL_CM / 2) * Math.SQRT2; // 1.41
const SAFETY_CM = 1;
const FIT_MARGIN_CM = SAFETY_CM + 2 * CELL_HALF_DIAGONAL_CM; // 3.83

// Inflated footprint, measured from the PIVOT. The body is asymmetric about it
// — two thirds of the length trails behind — so the rear overhang dominates
// everything that follows.
const AHEAD_CM = PIVOT_TO_FRONT_CM + FIT_MARGIN_CM; // 11.16
const BEHIND_CM = PIVOT_FROM_REAR_CM + FIT_MARGIN_CM; // 18.50
const HALF_WID_CM = ROBOT_WIDTH_CM / 2 + FIT_MARGIN_CM; // 11.83
// Radius of the disc the body sweeps through a full spin about the pivot.
// A "turn" cell needs everything inside this clear, so it also fits at every
// fixed heading (turn ⊆ up ∩ east).
const ROBOT_RADIUS_CM = Math.hypot(BEHIND_CM, HALF_WID_CM); // 21.96

const STEP_MAX_CM = 10; // furthest one blind exploration step may commit to
const MIN_STEP_CM = CELL_CM; // a step shorter than this isn't worth taking
const MAX_STEPS = 500; // hard cap so a stuck run can't loop forever

export async function run(r: Robot): Promise<void> {
  // Everything is a potential obstacle until proven clear; clear the start bubble.
  r.grid.fill(true);
  clearStartZone(r);

  // Each round: spin a full 360° at the current waypoint, rebuild the clearance
  // grids, then advance ONE segment toward the goal — scanning the whole way.
  // Re-planning at every waypoint lets a shorter path appear as the map fills in.
  for (let i = 0; i < MAX_STEPS; i++) {
    await scan(r);
    computeCells(r);

    const p = r.position();
    const goal = r.goal();
    if (Math.hypot(goal.x - p.x, goal.y - p.y) <= goal.radius) {
      r.log("goal reached");
      return;
    }

    // Choose the next move from a FULL multi-move path:
    //   1. the minimal-turn path to the goal, if one exists; else
    //   2. the path to the orange turn cell closest to the goal (Euclidean)
    //      — reached over several valid moves, not just a single straight move.
    // Either way we drive only its first segment, then loop to re-scan/re-plan.
    let path = findPath(r);
    if (path) {
      r.log("path to goal — advancing to next waypoint");
    } else {
      path = findGreedyPath(r);
      if (path) r.log("no goal path — advancing toward closest reachable turn cell");
    }

    if (path) {
      await driveToNextWaypoint(r, path);
      continue;
    }

    // Last resort only: no reachable turn cell is closer to the goal than we
    // already are, so there's nothing to plan toward until we reveal more map.
    // Take one step along the longer orange run to get a fresh vantage/scan.
    const northRun = reachableRun(r, p, 0, 1, Math.max(0, goal.y - p.y));
    const eastRun = reachableRun(r, p, 1, 0, Math.max(0, goal.x - p.x));
    if (northRun < MIN_STEP_CM && eastRun < MIN_STEP_CM) {
      r.log("stuck — no clear north or east step");
      return;
    }
    if (northRun >= eastRun) {
      await turnScanning(r, 0); // face north
      await driveScanning(r, Math.min(STEP_MAX_CM, northRun));
    } else {
      await turnScanning(r, 90); // face east
      await driveScanning(r, Math.min(STEP_MAX_CM, eastRun));
    }
  }

  r.log("step budget exhausted");
}

interface Cell {
  cx: number;
  cy: number;
}

/** Goal position projected onto its grid cell. */
function goalCell(r: Robot): Cell {
  const g = r.goal();
  return {
    cx: Math.max(0, Math.min(r.grid.cols - 1, Math.floor(g.x / CELL_CM))),
    cy: Math.max(0, Math.min(r.grid.rows - 1, Math.floor(g.y / CELL_CM))),
  };
}

/**
 * Fill the minimal-turn DP over the [start .. start+(W,H)] box. Drives north
 * only through up-cells, east only through east-cells, and only turns on
 * turn-cells. cost[i][j] = [turnsIfArrivedEast, turnsIfArrivedNorth]; Infinity
 * means no valid path reaches that cell that way. O(W*H) time and memory — no
 * priority queue, no per-node heap, so it stays tiny for the VEX port.
 */
function buildCostDP(
  r: Robot,
  s: Cell,
  W: number,
  H: number,
): [number, number][][] {
  const up = (i: number, j: number) => r.driveUp.get(s.cx + i, s.cy + j);
  const east = (i: number, j: number) => r.driveEast.get(s.cx + i, s.cy + j);
  const turn = (i: number, j: number) => r.reachable.get(s.cx + i, s.cy + j);

  const cost: [number, number][][] = [];
  for (let i = 0; i <= W; i++) {
    cost[i] = [];
    for (let j = 0; j <= H; j++) cost[i][j] = [Infinity, Infinity];
  }
  cost[0][0] = [0, 0]; // start (a turn cell): first move never counts as a turn

  for (let j = 0; j <= H; j++) {
    for (let i = 0; i <= W; i++) {
      if (i === 0 && j === 0) continue;
      let e = Infinity;
      let n = Infinity;
      // Arrive moving east into (i,j): the cell must be drivable facing east,
      // and any east<-north turn must happen on a turn cell at the predecessor.
      if (i > 0 && east(i, j)) {
        const p = cost[i - 1][j];
        e = Math.min(p[0], turn(i - 1, j) ? p[1] + 1 : Infinity);
      }
      // Arrive moving north into (i,j): cell must be drivable facing north.
      if (j > 0 && up(i, j)) {
        const p = cost[i][j - 1];
        n = Math.min(p[1], turn(i, j - 1) ? p[0] + 1 : Infinity);
      }
      cost[i][j] = [e, n];
    }
  }
  return cost;
}

/** Backtrack the DP from box-offset (I,J), arrived via dEnd (0=E,1=N), to the
 *  start, returning the absolute cell path start..target. */
function backtrack(
  cost: [number, number][][],
  s: Cell,
  I: number,
  J: number,
  dEnd: number,
): Cell[] {
  const cells: Cell[] = [];
  let i = I;
  let j = J;
  let d = dEnd;
  while (true) {
    cells.push({ cx: s.cx + i, cy: s.cy + j });
    if (i === 0 && j === 0) break;
    if (d === 0) {
      const cur = cost[i][j][0];
      d = cost[i - 1][j][0] === cur ? 0 : 1; // predecessor reached E or N?
      i -= 1;
    } else {
      const cur = cost[i][j][1];
      d = cost[i][j - 1][1] === cur ? 1 : 0;
      j -= 1;
    }
  }
  cells.reverse();
  return cells;
}

/** Minimal-turn N/E path from the robot's cell to the goal cell, or null. */
function findPath(r: Robot): Cell[] | null {
  const s = r.cell();
  const g = goalCell(r);
  // Monotone N/E can only reach a goal that is up and to the right.
  if (g.cx < s.cx || g.cy < s.cy) return null;
  // Must be able to pick an initial heading at the start (a turn cell).
  if (!r.reachable.get(s.cx, s.cy)) return null;

  const W = g.cx - s.cx;
  const H = g.cy - s.cy;
  const cost = buildCostDP(r, s, W, H);
  const [gE, gN] = cost[W][H];
  if (!Number.isFinite(Math.min(gE, gN))) return null;
  return backtrack(cost, s, W, H, gE <= gN ? 0 : 1);
}

/**
 * Fallback for when no N/E path reaches the goal. Among turn (orange) cells
 * reachable by a valid path, return a path to the one whose centre is closest
 * to the goal — but only if it is strictly closer to the goal than the robot's
 * own cell already is. That guard is what stops the robot oscillating: without
 * it the "closest" cell can be one we just came from. Returns null otherwise,
 * so the caller falls back to a blind exploration step.
 */
function findGreedyPath(r: Robot): Cell[] | null {
  const s = r.cell();
  const g = goalCell(r);
  if (g.cx < s.cx || g.cy < s.cy) return null;
  if (!r.reachable.get(s.cx, s.cy)) return null;

  const W = g.cx - s.cx;
  const H = g.cy - s.cy;
  const cost = buildCostDP(r, s, W, H);

  const goal = r.goal();
  const distToGoal = (cx: number, cy: number) =>
    Math.hypot(goal.x - (cx + 0.5) * CELL_CM, goal.y - (cy + 0.5) * CELL_CM);
  const d0 = distToGoal(s.cx, s.cy); // robot's own cell distance to the goal

  let best: {
    i: number;
    j: number;
    d: number;
    dist: number;
    turns: number;
  } | null = null;
  for (let i = 0; i <= W; i++) {
    for (let j = 0; j <= H; j++) {
      if (!r.reachable.get(s.cx + i, s.cy + j)) continue; // orange tiles only
      const [e, n] = cost[i][j];
      const turns = Math.min(e, n);
      if (!Number.isFinite(turns)) continue; // not reachable via a valid path
      const dist = distToGoal(s.cx + i, s.cy + j);
      if (
        best === null ||
        dist < best.dist - 1e-9 ||
        (Math.abs(dist - best.dist) < 1e-9 && turns < best.turns)
      ) {
        best = { i, j, d: e <= n ? 0 : 1, dist, turns };
      }
    }
  }

  if (!best || best.dist >= d0 - 1e-6) return null; // nothing strictly closer
  return backtrack(cost, s, best.i, best.j, best.d);
}

/** Reduce a cell path to just its corners (direction-change points). */
function cornerCells(cells: Cell[]): Cell[] {
  if (cells.length <= 2) return cells.slice();
  const out: Cell[] = [cells[0]];
  for (let k = 1; k < cells.length - 1; k++) {
    const before = cells[k].cx !== cells[k - 1].cx ? "E" : "N";
    const after = cells[k + 1].cx !== cells[k].cx ? "E" : "N";
    if (before !== after) out.push(cells[k]);
  }
  out.push(cells[cells.length - 1]);
  return out;
}

/**
 * Drive to the path's next turn point (or onto the goal, if the first segment
 * reaches it), scanning the whole way. The caller then spins and re-plans, so a
 * shorter path can appear before we commit any further.
 */
async function driveToNextWaypoint(r: Robot, cells: Cell[]): Promise<void> {
  const gc = goalCell(r);
  const corners = cornerCells(cells);
  if (corners.length >= 2) {
    const from = corners[0];
    const to = corners[1];
    if (to.cx !== from.cx) {
      await turnScanning(r, 90); // east segment
      await driveScanning(r, (to.cx + 0.5) * CELL_CM - r.position().x);
    } else {
      await turnScanning(r, 0); // north segment
      await driveScanning(r, (to.cy + 0.5) * CELL_CM - r.position().y);
    }
    if (to.cx === gc.cx && to.cy === gc.cy) await nudgeToGoal(r);
  } else {
    // Path is a single cell (already on the goal cell) — settle onto the goal.
    await nudgeToGoal(r);
  }
}

/**
 * Final approach onto the goal (east then north), scanning. Aims for the goal
 * tile's centre so the robot stays tile-aligned; falls back to the exact goal
 * point only if the tile centre wouldn't count as reaching it.
 */
async function nudgeToGoal(r: Robot): Promise<void> {
  const goal = r.goal();
  const gc = goalCell(r);
  const cx = (gc.cx + 0.5) * CELL_CM;
  const cy = (gc.cy + 0.5) * CELL_CM;
  const useCenter = Math.hypot(goal.x - cx, goal.y - cy) <= goal.radius;
  const tx = useCenter ? cx : goal.x;
  const ty = useCenter ? cy : goal.y;
  if (tx - r.position().x > 0.1) {
    await turnScanning(r, 90);
    await driveScanning(r, tx - r.position().x);
  }
  if (ty - r.position().y > 0.1) {
    await turnScanning(r, 0);
    await driveScanning(r, ty - r.position().y);
  }
}

/** Drive forward `cm`, carving free cells every rendered frame. */
async function driveScanning(r: Robot, cm: number): Promise<void> {
  if (cm <= 1e-6) return;
  let done = false;
  const motion = r.driveFor(cm);
  motion.then(
    () => {
      done = true;
    },
    () => {
      done = true; // cancelled (e.g. goal captured); re-thrown by `await motion`
    },
  );
  while (!done) {
    await r.step();
    markFree(r);
  }
  await motion;
}

/** Rotate to `headingDeg`, carving free cells every rendered frame. */
async function turnScanning(r: Robot, headingDeg: number): Promise<void> {
  let done = false;
  const motion = r.turnTo(headingDeg);
  motion.then(
    () => {
      done = true;
    },
    () => {
      done = true;
    },
  );
  while (!done) {
    await r.step();
    markFree(r);
  }
  await motion;
}

/**
 * Full 360° sweep at a waypoint, carving known-clear cells into r.grid. We spin
 * all the way round — not just the quadrant we travel into — because the
 * clearance grids depend on obstacles on every side of a candidate cell:
 * learning what's behind/beside us unlocks cells ahead as reachable, and it
 * gives the freshest possible map to re-plan on.
 */
async function scan(r: Robot): Promise<void> {
  markFree(r);
  for (let swept = 0; swept < SWEEP_TOTAL_DEG; swept += SWEEP_STEP_DEG) {
    await r.turn(SWEEP_STEP_DEG);
    markFree(r);
  }
}

/**
 * Rebuild the three clearance grids from the occupancy map. A cell must first be
 * known-clear itself, then:
 *   - reachable (orange): nothing known-blocked within the rotation disc.
 *   - driveUp (blue): the robot's north-facing footprint (width along x, length
 *     along y) is clear — safe to drive north through.
 *   - driveEast (pink): the east-facing footprint (length along x, width along
 *     y) is clear — safe to drive east through.
 * Out-of-bounds neighbours are ignored (board edges don't collide).
 */
function computeCells(r: Robot): void {
  const rCells = Math.ceil(ROBOT_RADIUS_CM / CELL_CM);
  const r2 = ROBOT_RADIUS_CM * ROBOT_RADIUS_CM;
  r.reachable.fill(false);
  r.driveUp.fill(false);
  r.driveEast.fill(false);
  for (let cy = 0; cy < r.grid.rows; cy++) {
    for (let cx = 0; cx < r.grid.cols; cx++) {
      if (r.grid.get(cx, cy)) continue; // the cell itself must be known-clear
      if (rotationClear(r, cx, cy, rCells, r2)) r.reachable.set(cx, cy, true);
      // The body trails behind the pivot, so "facing north" reaches BEHIND_CM
      // down and only AHEAD_CM up; facing east swaps the axes.
      if (boxClear(r, cx, cy, -HALF_WID_CM, HALF_WID_CM, -BEHIND_CM, AHEAD_CM)) {
        r.driveUp.set(cx, cy, true);
      }
      if (boxClear(r, cx, cy, -BEHIND_CM, AHEAD_CM, -HALF_WID_CM, HALF_WID_CM)) {
        r.driveEast.set(cx, cy, true);
      }
    }
  }
}

/** True if no known-blocked cell lies within the robot's radius of (cx, cy). */
function rotationClear(
  r: Robot,
  cx: number,
  cy: number,
  rCells: number,
  r2: number,
): boolean {
  for (let dy = -rCells; dy <= rCells; dy++) {
    for (let dx = -rCells; dx <= rCells; dx++) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || ny < 0 || nx >= r.grid.cols || ny >= r.grid.rows) continue;
      if (!r.grid.get(nx, ny)) continue; // clear neighbour — fine
      // Blocked/unknown neighbour: does it fall inside the rotation disc?
      const wx = dx * CELL_CM;
      const wy = dy * CELL_CM;
      if (wx * wx + wy * wy <= r2) return false;
    }
  }
  return true;
}

/**
 * True if the axis-aligned box spanning [minX, maxX] x [minY, maxY] cm about
 * the pivot at cell (cx, cy) overlaps no known-blocked cell. The extents are
 * signed because the robot's body is not centred on its pivot.
 */
function boxClear(
  r: Robot,
  cx: number,
  cy: number,
  minX: number,
  maxX: number,
  minY: number,
  maxY: number,
): boolean {
  const loX = Math.floor(minX / CELL_CM);
  const hiX = Math.ceil(maxX / CELL_CM);
  const loY = Math.floor(minY / CELL_CM);
  const hiY = Math.ceil(maxY / CELL_CM);
  for (let dy = loY; dy <= hiY; dy++) {
    for (let dx = loX; dx <= hiX; dx++) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || ny < 0 || nx >= r.grid.cols || ny >= r.grid.rows) continue;
      if (!r.grid.get(nx, ny)) continue; // clear neighbour — fine
      const wx = dx * CELL_CM;
      const wy = dy * CELL_CM;
      if (wx >= minX && wx <= maxX && wy >= minY && wy <= maxY) return false;
    }
  }
  return true;
}

/**
 * March from `origin` along a unit direction and return how far we can go while
 * staying on reachable (orange) cells, up to `maxCm`.
 */
function reachableRun(
  r: Robot,
  origin: Vec2,
  dirX: number,
  dirY: number,
  maxCm: number,
): number {
  const inc = CELL_CM / 2;
  let last = 0;
  for (let d = 0; d <= maxCm + 1e-9; d += inc) {
    const cx = Math.floor((origin.x + dirX * d) / CELL_CM);
    const cy = Math.floor((origin.y + dirY * d) / CELL_CM);
    if (cx < 0 || cy < 0 || cx >= r.grid.cols || cy >= r.grid.rows) break;
    if (!r.reachable.get(cx, cy)) break;
    last = d;
  }
  return last;
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
