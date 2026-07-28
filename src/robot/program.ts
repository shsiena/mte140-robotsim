
import {
  CELL_CM,
  SUBCELL_CM,
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
const SWEEP_TOTAL_DEG = 360; // full spin, where the robot can spin
const QUARTER_SWEEP_DEG = 90; // the north<->east arc, where it can only corner
const CLEAR_MARGIN_CM = 1.5; // stop clearing just short of the hit surface
// Guaranteed-clear bubble around the start. Must cover the footprint at every
// heading (hypot(BEHIND, HALF_WID)) or the robot cannot pick an opening move.
const START_CLEAR_RADIUS_CM = 24;

// Clearance tests carry no quantisation allowance, only a plain safety buffer.
// Two things earn that, and both must hold or the margin has to come back:
//
//   1. markFree clears a cell only when the whole cell was swept, so a clear
//      cell is empty everywhere, not just at its centre. rotationClear and
//      boxClear therefore test the footprint against blocked cell SQUARES
//      rather than blocked cell centres, which is exact — no allowance for
//      material sitting off-centre inside a cell.
//
//   2. The pivot is only ever tested where the robot can actually be. Spins
//      happen at cell centres: the start pose is cell-centred (World, and the
//      editor snaps user-placed starts), and driveToNextWaypoint drives exact
//      deltas onto (cx + 0.5) * SUBCELL_CM. Drives are axis-aligned, so the
//      cross-axis stays centred, and along the axis the footprint at any
//      intermediate point lies inside the union of the footprints at the two
//      bracketing cell centres — both of which the DP already requires clear.
const SAFETY_CM = 1;
const HALF_SUBCELL_CM = SUBCELL_CM / 2;

// Footprint measured from the PIVOT. The body is asymmetric about it — two
// thirds of the length trails behind — so the rear overhang dominates
// everything that follows.
const AHEAD_CM = PIVOT_TO_FRONT_CM + SAFETY_CM; // 8.33
const BEHIND_CM = PIVOT_FROM_REAR_CM + SAFETY_CM; // 15.67
const HALF_WID_CM = ROBOT_WIDTH_CM / 2 + SAFETY_CM; // 9.00
// Radius of the disc the body sweeps through a full spin about the pivot. Built
// from the box extents rather than the bare turning radius so it stays the
// circumscribing radius of both drive boxes — that is what makes turn ⊆ up ∩
// east hold, which the DP leans on when it seeds the start cell.
const ROBOT_RADIUS_CM = Math.hypot(BEHIND_CM, HALF_WID_CM); // 18.07

// The footprint as a rectangle about its own centre, which sits behind the
// pivot because the body trails. Used by the swept-mask builder.
const FOOTPRINT_MID_CM = (AHEAD_CM - BEHIND_CM) / 2; // -3.67
const FOOTPRINT_HALF_LEN_CM = (AHEAD_CM + BEHIND_CM) / 2; // 12.00

/**
 * Rotation step used when tracing a swept region. Sized so the outermost corner
 * advances a quarter of a sub-cell per step: sample any coarser and a thin
 * obstacle can hide between two orientations, never get tested, and report the
 * turn as clear. Scales with map resolution, so it stays correct if
 * GRID_RESOLUTION_PER_CELL changes.
 */
const SWEEP_TRACE_DEG =
  ((SUBCELL_CM / (4 * ROBOT_RADIUS_CM)) * 180) / Math.PI;

/**
 * Sub-cell offsets the body covers while rotating in place between the two
 * headings, relative to the pivot's own sub-cell. The shape depends only on the
 * robot's geometry, never on the map, so both masks are built once here and the
 * hot path is then a flat walk over the result. In the C++ port these become
 * const tables in flash rather than RAM.
 *
 * QUARTER_TURN_MASK covers a 90 degree turn between north and east. Both turn
 * directions sweep the same set — turnTo takes the shortest arc — so one mask
 * serves N->E and E->N alike.
 *
 * SPIN_MASK covers a full revolution. That one needs no tracing: the pivot lies
 * inside the body, so every bearing is covered and the swept region is exactly
 * the disc of the circumscribing radius.
 */
const QUARTER_TURN_MASK = traceSweptOffsets(0, 90);
const SPIN_MASK = discOffsets();

// A physical threshold, deliberately not tied to the map resolution: a finer
// map should not make ever-shorter exploration steps look worthwhile.
const MIN_STEP_CM = CELL_CM;
const MAX_STEPS = 500; // hard cap so a stuck run can't loop forever

export async function run(r: Robot): Promise<void> {
  // Everything is a potential obstacle until proven clear; clear the start bubble.
  r.grid.fill(true);
  clearStartZone(r);
  // scan() consults the clearance grids to size its sweep, so they must exist
  // before the first one. The start bubble is wider than ROBOT_RADIUS_CM, so
  // this makes the opening sweep a full revolution.
  computeCells(r);

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
    // A straight run may pass the goal's row or column, but only as far as the
    // goal disc still covers this line — monotone travel cannot come back, so
    // any further and the goal is lost. Stopping dead at the row throws away
    // room that is often exactly what clears an obstacle.
    const northRun = driveRun(r, p, 0, 1,
      Math.max(0, goal.y + goalChord(goal, Math.abs(p.x - goal.x)) - p.y));
    const eastRun = driveRun(r, p, 1, 0,
      Math.max(0, goal.x + goalChord(goal, Math.abs(p.y - goal.y)) - p.x));
    if (northRun < MIN_STEP_CM && eastRun < MIN_STEP_CM) {
      r.log("stuck — no clear north or east step");
      return;
    }
    // Driven in full rather than capped: both runs already end on the furthest
    // spin-capable cell, and truncating would stop somewhere we can't scan.
    if (northRun >= eastRun) {
      await turnScanning(r, 0); // face north
      await driveScanning(r, northRun);
    } else {
      await turnScanning(r, 90); // face east
      await driveScanning(r, eastRun);
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
    cx: Math.max(0, Math.min(r.grid.cols - 1, Math.floor(g.x / SUBCELL_CM))),
    cy: Math.max(0, Math.min(r.grid.rows - 1, Math.floor(g.y / SUBCELL_CM))),
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
  // Cornering needs only a quarter turn, not a full spin. Gating this on
  // reachable was what stranded paths whose one viable corner sat in a corridor
  // wide enough to turn in but not to spin in.
  const turn = (i: number, j: number) => r.turn90.get(s.cx + i, s.cy + j);

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

/**
 * Half-chord of the goal disc on a line passing `offAxisCm` from its centre —
 * how far past the goal's row or column a straight run along that line still
 * ends inside the goal. Zero when the line misses the disc entirely.
 */
function goalChord(goal: { radius: number }, offAxisCm: number): number {
  const d2 = goal.radius * goal.radius - offAxisCm * offAxisCm;
  return d2 > 0 ? Math.sqrt(d2) : 0;
}

/** True if this cell's centre lies inside the goal, which ends the run. */
function inGoal(r: Robot, cx: number, cy: number): boolean {
  const g = r.goal();
  const dx = (cx + 0.5) * SUBCELL_CM - g.x;
  const dy = (cy + 0.5) * SUBCELL_CM - g.y;
  return dx * dx + dy * dy <= g.radius * g.radius;
}

/**
 * DP box from the robot to the far edge of the goal DISC, not its centre.
 * The extra radius matters: rounding an obstacle often needs a little room past
 * the goal's own row or column, and any cell inside the disc ends the run just
 * as well. Monotone travel can never come back for that room, so a box stopping
 * at the centre gives it up permanently. Null when the goal is not up and to
 * the right, which monotone travel cannot reach at all.
 */
function goalBox(r: Robot, s: Cell): { W: number; H: number } | null {
  const g = goalCell(r);
  const pad = Math.ceil(r.goal().radius / SUBCELL_CM);
  const W = Math.min(r.grid.cols - 1, g.cx + pad) - s.cx;
  const H = Math.min(r.grid.rows - 1, g.cy + pad) - s.cy;
  return W < 0 || H < 0 ? null : { W, H };
}

/** Minimal-turn N/E path from the robot's cell into the goal, or null. */
function findPath(r: Robot): Cell[] | null {
  const s = r.cell();
  const box = goalBox(r, s);
  if (!box) return null;
  // Must be able to pick an initial heading at the start.
  if (!r.turn90.get(s.cx, s.cy)) return null;

  const { W, H } = box;
  const cost = buildCostDP(r, s, W, H);
  let best: { i: number; j: number; d: number; turns: number } | null = null;
  for (let i = 0; i <= W; i++) {
    for (let j = 0; j <= H; j++) {
      if (!inGoal(r, s.cx + i, s.cy + j)) continue;
      const [e, n] = cost[i][j];
      const turns = Math.min(e, n);
      if (!Number.isFinite(turns)) continue;
      if (best === null || turns < best.turns) {
        best = { i, j, d: e <= n ? 0 : 1, turns };
      }
    }
  }
  if (!best) return null;
  return backtrack(cost, s, best.i, best.j, best.d);
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
  const box = goalBox(r, s);
  if (!box) return null;
  if (!r.turn90.get(s.cx, s.cy)) return null;

  const gc = goalCell(r);
  const { W, H } = box;
  const cost = buildCostDP(r, s, W, H);

  const goal = r.goal();
  const distToGoal = (cx: number, cy: number) =>
    Math.hypot(goal.x - (cx + 0.5) * SUBCELL_CM, goal.y - (cy + 0.5) * SUBCELL_CM);
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
      // Any cell we can sweep from is a candidate vantage point, not just the
      // spin-capable ones — scan() sizes its arc to whatever the cell allows.
      if (!r.turn90.get(s.cx + i, s.cy + j)) continue;
      // The box reaches past the goal so paths can round an obstacle there, but
      // STOPPING past it strands us: monotone travel has no way back. Only cells
      // that end the run outright may sit beyond the goal's row or column.
      if (
        (s.cx + i > gc.cx || s.cy + j > gc.cy) &&
        !inGoal(r, s.cx + i, s.cy + j)
      ) {
        continue;
      }
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
 * Drive along the path, corner by corner, until reaching somewhere the caller
 * can safely spin — because its next act is a full 360° scan, and only a
 * reachable cell survives that.
 *
 * Corners that are merely turn90 are driven straight through without stopping.
 * That is the whole point of the quarter-turn mask: a group of spin-capable
 * cells that no single straight run can reach is now reachable across several
 * legs. Stopping at the FIRST spin-capable cell rather than the path's end
 * keeps the re-plan cadence as tight as the geometry allows, so a shorter route
 * can still appear as the map fills in.
 */
async function driveToNextWaypoint(r: Robot, cells: Cell[]): Promise<void> {
  const corners = cornerCells(cells);
  // Path is a single cell (already on the goal cell) — settle onto the goal.
  if (corners.length < 2) {
    await nudgeToGoal(r);
    return;
  }

  for (let k = 1; k < corners.length; k++) {
    const from = corners[k - 1];
    const to = corners[k];
    if (to.cx !== from.cx) {
      await turnScanning(r, 90); // east segment
      await driveScanning(r, (to.cx + 0.5) * SUBCELL_CM - r.position().x);
    } else {
      await turnScanning(r, 0); // north segment
      await driveScanning(r, (to.cy + 0.5) * SUBCELL_CM - r.position().y);
    }
    if (inGoal(r, to.cx, to.cy)) {
      await nudgeToGoal(r);
      return;
    }
    // Stop early only where a full revolution is available, since that buys the
    // most map. Merely cornerable corners are driven straight through; the
    // path's own end is a vantage the planner already vetted.
    if (r.reachable.get(to.cx, to.cy)) return;
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
  const cx = (gc.cx + 0.5) * SUBCELL_CM;
  const cy = (gc.cy + 0.5) * SUBCELL_CM;
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
 * Sweep the sensor at a waypoint, carving known-clear cells into r.grid, and
 * turn as far as the clearance here allows. We want the widest sweep we can
 * get — not just the quadrant we travel into — because the clearance grids
 * depend on obstacles on every side of a candidate cell, so learning what is
 * behind and beside us unlocks cells ahead.
 *
 * Where the robot can spin, that means a full revolution. Where it can only
 * corner, a quarter turn still yields about 114° of bearings once the 24° cone
 * is counted, which is enough to plan a further leg from. Insisting on the full
 * revolution everywhere is what stranded the robot beside an obstacle sitting a
 * shade inside ROBOT_RADIUS_CM, with open ground on every side.
 */
async function scan(r: Robot): Promise<void> {
  markFree(r);
  const c = r.cell();
  if (r.reachable.get(c.cx, c.cy)) {
    for (let swept = 0; swept < SWEEP_TOTAL_DEG; swept += SWEEP_STEP_DEG) {
      await r.turn(SWEEP_STEP_DEG);
      markFree(r);
    }
    return;
  }
  // Not spin-capable: take the quarter turn if it fits, else the fixed reading
  // we already have. Never rotate further than the masks vouch for.
  if (!r.turn90.get(c.cx, c.cy)) return;
  await turnScanning(r, 0);
  for (let swept = 0; swept < QUARTER_SWEEP_DEG; swept += SWEEP_STEP_DEG) {
    await r.turn(SWEEP_STEP_DEG);
    markFree(r);
  }
}

/**
 * Rebuild the four clearance grids from the occupancy map. A cell must first be
 * known-clear itself, then:
 *   - turn90 (teal): the body can rotate a quarter turn between north and east
 *     here — enough to take a corner, not enough to spin.
 *   - reachable (orange): nothing known-blocked within the full rotation disc,
 *     so the robot can spin on the spot. Only these are safe to scan from.
 *   - driveUp (blue): the north-facing footprint is clear — safe to drive
 *     north through, whether or not the robot could turn here.
 *   - driveEast (pink): likewise facing east.
 * Out-of-bounds neighbours are ignored (board edges don't collide).
 */
function computeCells(r: Robot): void {
  r.reachable.fill(false);
  r.turn90.fill(false);
  r.driveUp.fill(false);
  r.driveEast.fill(false);
  for (let cy = 0; cy < r.grid.rows; cy++) {
    for (let cx = 0; cx < r.grid.cols; cx++) {
      if (r.grid.get(cx, cy)) continue; // the cell itself must be known-clear
      if (maskClear(r, cx, cy, QUARTER_TURN_MASK)) {
        r.turn90.set(cx, cy, true);
        // The quarter turn passes through both fixed headings, so its clearance
        // implies each drive box: turn90 ⊆ driveUp ∩ driveEast.
        r.driveUp.set(cx, cy, true);
        r.driveEast.set(cx, cy, true);
        // reachable ⊆ turn90, so the expensive disc is only worth testing where
        // a quarter turn already fits.
        if (maskClear(r, cx, cy, SPIN_MASK)) r.reachable.set(cx, cy, true);
        continue;
      }
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

/** True if no known-blocked cell sits at any of the mask's offsets from (cx, cy). */
function maskClear(r: Robot, cx: number, cy: number, mask: Int16Array): boolean {
  for (let i = 0; i < mask.length; i += 2) {
    if (r.grid.get(cx + mask[i], cy + mask[i + 1])) return false;
  }
  return true;
}

/** Pack a window of marked flags into a flat (dx, dy) offset list. */
function packOffsets(hit: Uint8Array, reach: number): Int16Array {
  const span = 2 * reach + 1;
  const out: number[] = [];
  for (let dy = -reach; dy <= reach; dy++) {
    for (let dx = -reach; dx <= reach; dx++) {
      if (hit[(dy + reach) * span + (dx + reach)]) out.push(dx, dy);
    }
  }
  return Int16Array.from(out);
}

/**
 * Offsets whose sub-cell square overlaps the disc of ROBOT_RADIUS_CM. Closest
 * point of the square to the centre is found by clamping per axis, so a cell
 * whose centre sits outside the radius but whose corner reaches inside counts.
 */
function discOffsets(): Int16Array {
  const reach = Math.ceil(ROBOT_RADIUS_CM / SUBCELL_CM);
  const span = 2 * reach + 1;
  const r2 = ROBOT_RADIUS_CM * ROBOT_RADIUS_CM;
  const hit = new Uint8Array(span * span);
  for (let dy = -reach; dy <= reach; dy++) {
    for (let dx = -reach; dx <= reach; dx++) {
      const nearX = Math.max(0, Math.abs(dx * SUBCELL_CM) - HALF_SUBCELL_CM);
      const nearY = Math.max(0, Math.abs(dy * SUBCELL_CM) - HALF_SUBCELL_CM);
      if (nearX * nearX + nearY * nearY < r2) {
        hit[(dy + reach) * span + (dx + reach)] = 1;
      }
    }
  }
  return packOffsets(hit, reach);
}

/**
 * Trace the region the body sweeps rotating from `fromDeg` to `toDeg`, by
 * stepping the heading and unioning the footprint at each step. Cells already
 * marked are skipped, which is what keeps this affordable despite the fine
 * step — most of the region is claimed in the first few degrees.
 */
function traceSweptOffsets(fromDeg: number, toDeg: number): Int16Array {
  const reach = Math.ceil(ROBOT_RADIUS_CM / SUBCELL_CM);
  const span = 2 * reach + 1;
  const hit = new Uint8Array(span * span);
  for (let deg = fromDeg; deg <= toDeg + 1e-9; deg += SWEEP_TRACE_DEG) {
    const rad = (deg * Math.PI) / 180;
    const sin = Math.sin(rad);
    const cos = Math.cos(rad);
    for (let dy = -reach; dy <= reach; dy++) {
      for (let dx = -reach; dx <= reach; dx++) {
        const idx = (dy + reach) * span + (dx + reach);
        if (hit[idx]) continue;
        if (footprintCoversSquare(dx * SUBCELL_CM, dy * SUBCELL_CM, sin, cos)) {
          hit[idx] = 1;
        }
      }
    }
  }
  return packOffsets(hit, reach);
}

/**
 * Separating-axis test: does the footprint at this heading overlap the
 * axis-aligned sub-cell square centred at (px, py) relative to the pivot? Two
 * convex rectangles need only four axes — the square's two, and the
 * footprint's two.
 */
function footprintCoversSquare(
  px: number,
  py: number,
  sin: number,
  cos: number,
): boolean {
  const half = HALF_SUBCELL_CM;
  const absSin = Math.abs(sin);
  const absCos = Math.abs(cos);
  // Offset from the footprint's centre, which trails the pivot.
  const dx = px - sin * FOOTPRINT_MID_CM;
  const dy = py - cos * FOOTPRINT_MID_CM;

  const along = dx * sin + dy * cos;
  if (Math.abs(along) > FOOTPRINT_HALF_LEN_CM + half * (absSin + absCos)) {
    return false;
  }
  const across = dx * cos - dy * sin;
  if (Math.abs(across) > HALF_WID_CM + half * (absCos + absSin)) return false;

  const spanX = FOOTPRINT_HALF_LEN_CM * absSin + HALF_WID_CM * absCos;
  if (Math.abs(dx) > spanX + half) return false;
  const spanY = FOOTPRINT_HALF_LEN_CM * absCos + HALF_WID_CM * absSin;
  return Math.abs(dy) <= spanY + half;
}

/**
 * True if the axis-aligned box spanning [minX, maxX] x [minY, maxY] cm about
 * the pivot at cell (cx, cy) overlaps no known-blocked cell. The extents are
 * signed because the robot's body is not centred on its pivot. Overlap is
 * box-against-cell-square, so a blocked cell only has to reach the box to
 * count, not to have its centre inside it.
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
  const loX = Math.floor(minX / SUBCELL_CM);
  const hiX = Math.ceil(maxX / SUBCELL_CM);
  const loY = Math.floor(minY / SUBCELL_CM);
  const hiY = Math.ceil(maxY / SUBCELL_CM);
  for (let dy = loY; dy <= hiY; dy++) {
    for (let dx = loX; dx <= hiX; dx++) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || ny < 0 || nx >= r.grid.cols || ny >= r.grid.rows) continue;
      if (!r.grid.get(nx, ny)) continue; // clear neighbour — fine
      const wx = dx * SUBCELL_CM;
      const wy = dy * SUBCELL_CM;
      if (wx + HALF_SUBCELL_CM <= minX || wx - HALF_SUBCELL_CM >= maxX) continue;
      if (wy + HALF_SUBCELL_CM <= minY || wy - HALF_SUBCELL_CM >= maxY) continue;
      return false;
    }
  }
  return true;
}

/**
 * March from `origin` along an axis while the footprint keeps fitting, and
 * return the furthest distance that is also spin-capable — because the caller
 * stops there and immediately scans, and a 360° sweep needs full rotation
 * clearance.
 *
 * The corridor itself only needs the matching drive mask. Requiring spin
 * clearance for every cell along the way is what used to strand the robot in
 * open terrain: an obstacle a shade under ROBOT_RADIUS_CM to the side kills
 * every orange cell in a corridor the body fits through with room to spare.
 *
 * Returns 0 when nothing along the run can be spun at, which the caller reads
 * as having nowhere useful to go.
 */
function driveRun(
  r: Robot,
  origin: Vec2,
  dirX: number,
  dirY: number,
  maxCm: number,
): number {
  const inc = SUBCELL_CM / 2;
  const fits = dirY !== 0 ? r.driveUp : r.driveEast;
  let stop = 0;
  for (let d = 0; d <= maxCm + 1e-9; d += inc) {
    const cx = Math.floor((origin.x + dirX * d) / SUBCELL_CM);
    const cy = Math.floor((origin.y + dirY * d) / SUBCELL_CM);
    if (cx < 0 || cy < 0 || cx >= r.grid.cols || cy >= r.grid.rows) break;
    if (!fits.get(cx, cy)) break;
    if (r.turn90.get(cx, cy)) stop = d; // furthest point we could sweep from
  }
  return stop;
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

// Two rows of the corner lattice, reused across calls. Every lattice point is
// shared by up to four cells, so sweeping row-pairs tests each point once
// instead of once per adjoining cell. Two rows is all the history needed, so
// this stays O(cols) rather than O(cols * rows).
let cornerRowBelow = new Uint8Array(0);
let cornerRowAbove = new Uint8Array(0);

/**
 * True if the offset (dx, dy) from the sensor origin lies inside the scan
 * wedge: within `reach`, and within half the cone angle of the heading.
 * Squared magnitudes only — no atan2, no sqrt — since this runs on every
 * corner in range on every frame.
 */
function inWedge(
  dx: number,
  dy: number,
  fx: number,
  fy: number,
  reach2: number,
  cosHalfConeSq: number,
): boolean {
  const d2 = dx * dx + dy * dy;
  if (d2 > reach2) return false;
  const along = dx * fx + dy * fy; // projection onto the heading
  if (along <= 0) return d2 === 0; // the apex is inside; behind it is not
  return along * along >= cosHalfConeSq * d2; // cos^2(angle) vs cos^2(halfCone)
}

/**
 * Inverse sensor model for one measurement. A cell is cleared only when ALL
 * FOUR of its corners lie inside the wedge, not merely its centre: the
 * clearance grids read a clear cell as a full SUBCELL_CM square of emptiness, so
 * clearing on the centre alone lets a cell straddling the beam edge or the hit
 * arc be treated as wholly open. Under-clearing is safe — unknown is already
 * treated as blocked — whereas over-clearing is what drives the robot into
 * things.
 *
 * Four corners is not a sampling approximation: a sector below 180 degrees is
 * the intersection of a disc with two half-planes, hence convex, so a square
 * lies inside it exactly when its corners do.
 */
function markFree(r: Robot): void {
  const o = sensorOrigin(r);
  const d = r.distance();
  // How far we can trust "free" along the sensor: up to the hit (minus a
  // margin so we don't erase the obstacle face), or full range if nothing hit.
  const reach = Number.isFinite(d) ? d - CLEAR_MARGIN_CM : SENSOR_MAX_CM;
  if (reach <= 0) return;

  const rad = (r.heading() * Math.PI) / 180;
  const fx = Math.sin(rad); // heading as a unit vector (0 deg = +y)
  const fy = Math.cos(rad);
  const reach2 = reach * reach;
  const cosHalf = Math.cos((SENSOR_HALF_CONE_DEG * Math.PI) / 180);
  const cosHalfSq = cosHalf * cosHalf;

  // Square bounding box around the reachable area; the wedge test trims it. A
  // cell inside the reach disc has its centre inside it too, so nothing is
  // missed by bounding on centres.
  const lo = clampCell(o.x - reach, o.y - reach, r);
  const hi = clampCell(o.x + reach, o.y + reach, r);

  // Corner columns span [lo.cx, hi.cx + 1] — one more than the cell columns.
  const width = hi.cx - lo.cx + 2;
  if (cornerRowBelow.length < width) {
    cornerRowBelow = new Uint8Array(width);
    cornerRowAbove = new Uint8Array(width);
  }

  const fillRow = (row: Uint8Array, cy: number): void => {
    const dy = cy * SUBCELL_CM - o.y;
    for (let i = 0; i < width; i++) {
      row[i] = inWedge(
        (lo.cx + i) * SUBCELL_CM - o.x,
        dy,
        fx,
        fy,
        reach2,
        cosHalfSq,
      )
        ? 1
        : 0;
    }
  };

  let below = cornerRowBelow;
  let above = cornerRowAbove;
  fillRow(below, lo.cy);
  for (let cy = lo.cy; cy <= hi.cy; cy++) {
    fillRow(above, cy + 1);
    for (let i = 0; i + 1 < width; i++) {
      if (below[i] && below[i + 1] && above[i] && above[i + 1]) {
        r.grid.set(lo.cx + i, cy, false); // whole cell provably empty
      }
    }
    // The row we just filled becomes the lower edge of the next cell row.
    const spent = below;
    below = above;
    above = spent;
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
      const dx = (cx + 0.5) * SUBCELL_CM - p.x;
      const dy = (cy + 0.5) * SUBCELL_CM - p.y;
      if (dx * dx + dy * dy <= rad2) r.grid.set(cx, cy, false);
    }
  }
}

/** Clamp a world point (cm) to valid cell indices. */
function clampCell(x: number, y: number, r: Robot): { cx: number; cy: number } {
  return {
    cx: Math.max(0, Math.min(r.grid.cols - 1, Math.floor(x / SUBCELL_CM))),
    cy: Math.max(0, Math.min(r.grid.rows - 1, Math.floor(y / SUBCELL_CM))),
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
