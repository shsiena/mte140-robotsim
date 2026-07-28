// The public API your algorithm is written against. This is the contract that
// `run()` in program.ts receives. Keep it close to what you can express on the
// real VEX IQ so porting stays mechanical.

/** A boolean grid matching the environment's cell dimensions. */
export interface BoolGrid {
  readonly cols: number;
  readonly rows: number;
  get(cx: number, cy: number): boolean;
  /** Out-of-range writes are ignored. */
  set(cx: number, cy: number, value: boolean): void;
  fill(value: boolean): void;
  /** Direct row-major access: raw[cy][cx]. cy=0 is the bottom row. */
  readonly raw: boolean[][];
}

export interface Robot {
  // --- sensing (instant, synchronous) --------------------------------------
  /** Robot pivot point in world cm — the point it rotates about, two thirds of
   *  the way forward. Origin bottom-left, +x right, +y up. */
  position(): { x: number; y: number };
  /** Current grid cell of the robot pivot. cy=0 is the bottom row. */
  cell(): { cx: number; cy: number };
  /** Heading in degrees: 0=north(+y), clockwise positive. */
  heading(): number;
  /** IR reading in cm (1..80), or Infinity when nothing is in range. */
  distance(): number;
  /** The goal, known from the start. Success = centre within `radius`. */
  goal(): { x: number; y: number; radius: number };
  /** Environment dimensions. */
  gridSize(): { cols: number; rows: number; cellCm: number };

  // --- blocking motion (await; a collision ends the run) -------------------
  /** Drive along the current heading. +cm forward, -cm reverse. */
  driveFor(cm: number): Promise<void>;
  /** Rotate in place to an absolute heading, taking the shortest direction. */
  turnTo(headingDeg: number): Promise<void>;
  /** Rotate in place by a relative amount (+clockwise). */
  turn(deltaDeg: number): Promise<void>;

  // --- continuous motion (non-blocking) ------------------------------------
  /** Start driving at cm/s (sign = forward/reverse); runs until stop()/hit. */
  setDrive(cmPerSec: number): void;
  /** Start rotating at deg/s (+clockwise); runs until stop()/hit. */
  setTurn(degPerSec: number): void;
  /** Halt all continuous motion. */
  stop(): void;

  // --- yielding control ----------------------------------------------------
  /** Resolve after the next rendered frame (pose + sensor updated). */
  step(): Promise<void>;
  /** Resolve after `ms` of simulation time has elapsed. */
  wait(ms: number): Promise<void>;

  // --- your internal world model (rendered live as a cell overlay) ---------
  readonly grid: BoolGrid;
  /**
   * A second boolean grid, drawn as an orange overlay. Intended for cells the
   * robot centre can occupy with full-rotation clearance ("turn" cells), but
   * it's yours to use however you like.
   */
  readonly reachable: BoolGrid;
  /**
   * Overlay (teal): cells the robot can rotate a quarter turn on, between
   * facing north and facing east. Enough clearance to take a corner while
   * driving through, but not necessarily to spin on the spot.
   */
  readonly turn90: BoolGrid;
  /** Extra overlay (blue): cells the robot fits in facing north (drive-north). */
  readonly driveUp: BoolGrid;
  /** Extra overlay (pink): cells the robot fits in facing east (drive-east). */
  readonly driveEast: BoolGrid;

  /** Print to the telemetry log / console for debugging. */
  log(...args: unknown[]): void;
}

/** Signature every program.ts must export. */
export type RobotProgram = (robot: Robot) => Promise<void>;
