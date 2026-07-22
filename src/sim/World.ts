// The editable environment: grid dimensions, obstacles, robot start pose, goal,
// and the live boolean overlay grid the program writes to.

import { CELL_CM } from "../config";
import type { BoolGrid } from "../robot/types";
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
  cols: number;
  rows: number;
  obstacles: Box[] = [];
  start: Pose;
  goal: Goal;
  overlay: MatrixGrid;
  /** Cells where the robot centre has full-rotation clearance (drawn orange). */
  reachable: MatrixGrid;
  /** Cells the robot fits in facing north — drive-north clearance (blue). */
  driveUp: MatrixGrid;
  /** Cells the robot fits in facing east — drive-east clearance (pink). */
  driveEast: MatrixGrid;
  /** UI toggle passed to the program: hop to the closest reachable turn cell. */
  greedyTravel = false;

  constructor(cols = 40, rows = 30) {
    this.cols = cols;
    this.rows = rows;
    this.overlay = new MatrixGrid(cols, rows);
    this.reachable = new MatrixGrid(cols, rows);
    this.driveUp = new MatrixGrid(cols, rows);
    this.driveEast = new MatrixGrid(cols, rows);
    // Sensible defaults: start centred in a cell near the bottom-left facing
    // north, goal top-right. (Centre, not a grid corner, so the robot stays
    // tile-aligned as it drives.)
    this.start = { x: 3.5 * CELL_CM, y: 3.5 * CELL_CM, heading: 0 };
    this.goal = {
      x: (cols - 3) * CELL_CM,
      y: (rows - 3) * CELL_CM,
      radius: 4,
    };
  }

  get widthCm(): number {
    return this.cols * CELL_CM;
  }
  get heightCm(): number {
    return this.rows * CELL_CM;
  }

  /** Resize the grid, rebuilding the overlay and clamping features to bounds. */
  resize(cols: number, rows: number): void {
    this.cols = cols;
    this.rows = rows;
    this.overlay = new MatrixGrid(cols, rows);
    this.reachable = new MatrixGrid(cols, rows);
    this.driveUp = new MatrixGrid(cols, rows);
    this.driveEast = new MatrixGrid(cols, rows);
    this.start.x = Math.min(this.start.x, this.widthCm);
    this.start.y = Math.min(this.start.y, this.heightCm);
    this.goal.x = Math.min(this.goal.x, this.widthCm);
    this.goal.y = Math.min(this.goal.y, this.heightCm);
  }

  clearOverlay(): void {
    this.overlay.fill(false);
    this.reachable.fill(false);
    this.driveUp.fill(false);
    this.driveEast.fill(false);
  }
}
