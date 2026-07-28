// Replays a board snapshot exported by the UI's "Copy board JSON" button
// against the real Simulation and program, with no renderer involved.

import { CELL_CM, GRID_RESOLUTION_PER_CELL } from "../src/config";
import { run } from "../src/robot/program";
import { Simulation } from "../src/sim/Simulation";
import { World } from "../src/sim/World";

export interface BoardSnapshot {
  grid: { cols: number; rows: number; cellCm: number; resolutionPerCell: number };
  start: { x: number; y: number; heading: number };
  goal: { x: number; y: number; radius: number };
  obstacles: { cx: number; cy: number; w: number; h: number; angle: number }[];
}

export interface BoardResult {
  status: string;
  reason: string;
  travelledCm: number;
  ticks: number;
  log: string[];
  distanceToGoalCm: number;
}

const TICK_SECONDS = 1 / 60;
const TICK_LIMIT = 400_000;

/**
 * A snapshot records the grid settings it was captured under, because the
 * clearance grids are resolution dependent — replaying one against different
 * settings would not be the same test.
 */
function assertSnapshotMatchesConfig(board: BoardSnapshot): void {
  if (board.grid.cellCm !== CELL_CM) {
    throw new Error(`board cellCm ${board.grid.cellCm}, config ${CELL_CM}`);
  }
  if (board.grid.resolutionPerCell !== GRID_RESOLUTION_PER_CELL) {
    throw new Error(
      `board resolutionPerCell ${board.grid.resolutionPerCell}, ` +
        `config ${GRID_RESOLUTION_PER_CELL}`,
    );
  }
}

export async function runBoard(board: BoardSnapshot): Promise<BoardResult> {
  assertSnapshotMatchesConfig(board);

  const world = new World(board.grid.cols, board.grid.rows);
  world.obstacles = board.obstacles.map((o) => ({ ...o }));
  world.start = { ...board.start };
  world.goal = { ...board.goal };

  const sim = new Simulation(world);
  const log: string[] = [];
  sim.onLog = (line) => log.push(line);
  sim.run(run);

  let travelledCm = 0;
  let previous = { ...sim.pivot };
  let ticks = 0;
  while (ticks < TICK_LIMIT && sim.status === "running") {
    sim.tick(TICK_SECONDS);
    travelledCm += Math.hypot(sim.pivot.x - previous.x, sim.pivot.y - previous.y);
    previous = { ...sim.pivot };
    ticks++;
    // The program advances through awaits, so the tick loop has to yield a
    // macrotask or it starves its own continuations and nothing moves.
    await new Promise((resolve) => setImmediate(resolve));
  }

  return {
    status: sim.status,
    reason: sim.statusReason,
    travelledCm,
    ticks,
    log,
    distanceToGoalCm: Math.hypot(
      world.goal.x - sim.pivot.x,
      world.goal.y - sim.pivot.y,
    ),
  };
}
