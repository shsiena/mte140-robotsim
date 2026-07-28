// The editable environment: grid dimensions, obstacles, robot start pose, goal,
// and the live boolean overlay grid the program writes to.

import { CELL_CM, GRID_RESOLUTION_PER_CELL } from "../config";
import type { BoolGrid } from "../robot/types";
import { snapToSubCellCenter } from "./coords";
import type { Box } from "./geometry";

export interface Pose {
  x: number; // cm
  y: number; // cm
  heading: number; // deg
}

export interface Goal {
  x: number; // cm
  y: number; // cm
  radius: number; // cm
}

class MatrixGrid implements BoolGrid {
  raw: boolean[][];
  constructor(
    public readonly cols: number,
    public readonly rows: number,
  ) {
    this.raw = Array.from({ length: rows }, () =>
      new Array<boolean>(cols).fill(false),
    );
  }
  get(cx: number, cy: number): boolean {
    if (cx < 0 || cy < 0 || cx >= this.cols || cy >= this.rows) return false;
    return this.raw[cy][cx];
  }
  set(cx: number, cy: number, value: boolean): void {
    if (cx < 0 || cy < 0 || cx >= this.cols || cy >= this.rows) return;
    this.raw[cy][cx] = value;
  }
  fill(value: boolean): void {
    for (const row of this.raw) row.fill(value);
  }
}

export class World {
  /** Board width in drawn cells. The maps below are GRID_RESOLUTION_PER_CELL
   *  times finer along each edge; nothing else in the app is. */
  cols: number;
  rows: number;
  obstacles: Box[] = [];
  start: Pose;
  goal: Goal;
  // Assigned via allocGrids, which both the constructor and resize() call.
  overlay!: MatrixGrid;
  /** Sub-cells where the robot pivot has full-rotation clearance (orange). */
  reachable!: MatrixGrid;
  /** Sub-cells the robot can take a 90° corner on — quarter-turn clearance (teal). */
  turn90!: MatrixGrid;
  /** Sub-cells the robot fits in facing north — drive-north clearance (blue). */
  driveUp!: MatrixGrid;
  /** Sub-cells the robot fits in facing east — drive-east clearance (pink). */
  driveEast!: MatrixGrid;
  /** Display toggle: draw the obstacle boxes. */
  showObstacles = true;

  constructor(cols = 40, rows = 30) {
    this.cols = cols;
    this.rows = rows;
    this.allocGrids();
    // Sensible defaults: start near the bottom-left facing north, goal
    // top-right. Snapped to a sub-cell centre, since that is where the
    // clearance grids assume the pivot sits whenever the robot spins. Far
    // enough from the edges that the body — which trails PIVOT_FROM_REAR_CM
    // behind the pose point — starts on the board.
    const s = snapToSubCellCenter({ x: 4.5 * CELL_CM, y: 8.5 * CELL_CM });
    this.start = { x: s.x, y: s.y, heading: 0 };
    this.goal = {
      x: (cols - 3) * CELL_CM,
      y: (rows - 3) * CELL_CM,
      radius: 4,
    };
  }

  private allocGrids(): void {
    const gx = this.cols * GRID_RESOLUTION_PER_CELL;
    const gy = this.rows * GRID_RESOLUTION_PER_CELL;
    this.overlay = new MatrixGrid(gx, gy);
    this.reachable = new MatrixGrid(gx, gy);
    this.turn90 = new MatrixGrid(gx, gy);
    this.driveUp = new MatrixGrid(gx, gy);
    this.driveEast = new MatrixGrid(gx, gy);
  }

  get widthCm(): number {
    return this.cols * CELL_CM;
  }
  get heightCm(): number {
    return this.rows * CELL_CM;
  }

  /** Resize the board, rebuilding the maps and clamping features to bounds. */
  resize(cols: number, rows: number): void {
    this.cols = cols;
    this.rows = rows;
    this.allocGrids();
    this.start.x = Math.min(this.start.x, this.widthCm);
    this.start.y = Math.min(this.start.y, this.heightCm);
    this.goal.x = Math.min(this.goal.x, this.widthCm);
    this.goal.y = Math.min(this.goal.y, this.heightCm);
  }

  clearOverlay(): void {
    this.overlay.fill(false);
    this.reachable.fill(false);
    this.turn90.fill(false);
    this.driveUp.fill(false);
    this.driveEast.fill(false);
  }
}
