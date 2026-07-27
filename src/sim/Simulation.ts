// The simulation core. Owns the robot's live pose, integrates motion frame by
// frame (no kinematic model — speeds are cosmetic and the outcome is
// deterministic), detects collisions and goal capture, and exposes the async
// `Robot` API that programs are written against.

import {
  DEFAULT_DRIVE_CM_S,
  DEFAULT_TURN_DEG_S,
  MAX_SUBSTEP_CM,
  MAX_SUBSTEP_DEG,
} from "../config";
import type { BoolGrid, Robot } from "../robot/types";
import {
  type Vec2,
  add,
  angleDiff,
  dist,
  headingToVec,
  normalizeDeg,
  worldToCell,
} from "./coords";
import { measureCone, robotHitsAny, sensorOrigin } from "./geometry";
import { World } from "./World";

export type SimStatus = "idle" | "running" | "success" | "failed" | "stopped";

/** Thrown into the program's awaits when a run is cancelled/superseded. */
class Cancelled extends Error {
  constructor() {
    super("simulation run cancelled");
    this.name = "Cancelled";
  }
}

interface Deferred {
  resolve: () => void;
  reject: (e: unknown) => void;
}

type ActiveMove =
  | ({ kind: "drive"; remaining: number; sign: number } & Deferred)
  | ({ kind: "turn"; remaining: number; dir: number; target: number } & Deferred);

const EPS = 1e-6;

export class Simulation {
  pivot: Vec2 = { x: 0, y: 0 };
  heading = 0;
  status: SimStatus = "idle";
  statusReason = "";

  // Continuous velocities (set via setDrive/setTurn).
  private linVel = 0;
  private angVel = 0;

  // Default speeds for blocking moves (cosmetic).
  driveSpeed = DEFAULT_DRIVE_CM_S;
  turnSpeed = DEFAULT_TURN_DEG_S;

  private activeMove: ActiveMove | null = null;
  private stepWaiters: Deferred[] = [];
  private timeWaiters: { remaining: number; d: Deferred }[] = [];

  private generation = 0;
  trail: Vec2[] = [];
  lastSensor = Infinity;

  onStatus?: (status: SimStatus, reason: string) => void;
  onLog?: (line: string) => void;

  constructor(public world: World) {
    this.applyStartPose();
  }

  private applyStartPose(): void {
    this.pivot = { x: this.world.start.x, y: this.world.start.y };
    this.heading = normalizeDeg(this.world.start.heading);
    this.linVel = 0;
    this.angVel = 0;
    this.activeMove = null;
    this.trail = [{ ...this.pivot }];
    this.lastSensor = this.measure();
  }

  private setStatus(status: SimStatus, reason = ""): void {
    this.status = status;
    this.statusReason = reason;
    this.onStatus?.(status, reason);
  }

  // --- lifecycle -----------------------------------------------------------

  /** Cancel any running program and reset to the start pose. */
  reset(): void {
    this.cancelPending();
    this.generation++;
    this.world.clearOverlay();
    this.applyStartPose();
    this.setStatus("idle");
  }

  /** Reset, then launch a program against a fresh Robot API. */
  run(program: (r: Robot) => Promise<void>): void {
    this.reset();
    const gen = ++this.generation;
    this.setStatus("running");
    const api = new RobotApi(this, gen);
    Promise.resolve()
      .then(() => program(api))
      .then(() => {
        if (this.generation === gen && this.status === "running") {
          this.linVel = 0;
          this.angVel = 0;
          this.setStatus("stopped", "program finished");
        }
      })
      .catch((err) => {
        if (err instanceof Cancelled) return; // expected on reset/finish
        if (this.generation === gen) {
          this.setStatus("failed", `program error: ${String(err)}`);
        }
        // Surface real errors for debugging.
        if (!(err instanceof Cancelled)) console.error(err);
      });
  }

  private cancelPending(): void {
    const c = new Cancelled();
    this.activeMove?.reject(c);
    this.activeMove = null;
    for (const w of this.stepWaiters) w.reject(c);
    this.stepWaiters = [];
    for (const t of this.timeWaiters) t.d.reject(c);
    this.timeWaiters = [];
  }

  private fail(reason: string): void {
    this.linVel = 0;
    this.angVel = 0;
    this.setStatus("failed", reason);
    this.cancelPending();
  }

  private succeed(): void {
    this.linVel = 0;
    this.angVel = 0;
    this.setStatus("success", "reached goal");
    this.cancelPending();
  }

  // --- per-frame integration ----------------------------------------------

  /** Advance the simulation by `dt` seconds (called from the render loop). */
  tick(dt: number): void {
    if (this.status !== "running" || dt <= 0) return;

    this.integrate(dt);
    if (this.status !== "running") return;

    // Goal capture: robot pivot within the goal radius.
    const g = this.world.goal;
    if (dist(this.pivot, { x: g.x, y: g.y }) <= g.radius) {
      this.succeed();
      return;
    }

    this.lastSensor = this.measure();

    // One rendered frame has elapsed: release step() waiters.
    if (this.stepWaiters.length) {
      const waiters = this.stepWaiters;
      this.stepWaiters = [];
      for (const w of waiters) w.resolve();
    }

    // Advance timed waits.
    if (this.timeWaiters.length) {
      const ready: Deferred[] = [];
      this.timeWaiters = this.timeWaiters.filter((t) => {
        t.remaining -= dt * 1000;
        if (t.remaining <= 0) {
          ready.push(t.d);
          return false;
        }
        return true;
      });
      for (const d of ready) d.resolve();
    }
  }

  private integrate(dt: number): void {
    // Work out this frame's angular + forward deltas.
    let dHeading = 0;
    let dForward = 0;
    const move = this.activeMove;

    if (move?.kind === "turn") {
      const stepDeg = Math.min(this.turnSpeed * dt, move.remaining);
      dHeading = move.dir * stepDeg;
    } else if (move?.kind === "drive") {
      const stepCm = Math.min(this.driveSpeed * dt, move.remaining);
      dForward = move.sign * stepCm;
    } else {
      dHeading = this.angVel * dt;
      dForward = this.linVel * dt;
    }

    if (dHeading === 0 && dForward === 0) return;

    // Sub-step so we can't tunnel through thin obstacles and so we stop near
    // the contact point.
    const n = Math.max(
      1,
      Math.ceil(
        Math.max(
          Math.abs(dHeading) / MAX_SUBSTEP_DEG,
          Math.abs(dForward) / MAX_SUBSTEP_CM,
        ),
      ),
    );
    const stepH = dHeading / n;
    const stepF = dForward / n;

    for (let i = 0; i < n; i++) {
      const prevPivot = { ...this.pivot };
      const prevHeading = this.heading;
      this.heading = normalizeDeg(this.heading + stepH);
      this.pivot = add(this.pivot, headingToVec(this.heading), stepF);
      if (robotHitsAny(this.pivot, this.heading, this.world.obstacles)) {
        // Revert the offending sub-step and fail.
        this.pivot = prevPivot;
        this.heading = prevHeading;
        this.fail("collision with obstacle");
        return;
      }
      this.pushTrail();
    }

    // Consume the blocking move's remaining budget; resolve when spent.
    if (move?.kind === "turn") {
      move.remaining -= Math.abs(dHeading);
      if (move.remaining <= EPS) {
        this.heading = normalizeDeg(move.target);
        this.activeMove = null;
        move.resolve();
      }
    } else if (move?.kind === "drive") {
      move.remaining -= Math.abs(dForward);
      if (move.remaining <= EPS) {
        this.activeMove = null;
        move.resolve();
      }
    }
  }

  private pushTrail(): void {
    const last = this.trail[this.trail.length - 1];
    if (!last || dist(last, this.pivot) >= 0.5) {
      this.trail.push({ ...this.pivot });
    }
  }

  private measure(): number {
    const origin = sensorOrigin(this.pivot, this.heading);
    return measureCone(origin, this.heading, this.world.obstacles);
  }

  // --- API plumbing (called by RobotApi) -----------------------------------

  apiDistance(): number {
    return this.measure();
  }

  apiDriveFor(gen: number, cm: number): Promise<void> {
    return this.beginMove(gen, (d) => {
      if (cm === 0) return d.resolve();
      this.activeMove = {
        kind: "drive",
        remaining: Math.abs(cm),
        sign: Math.sign(cm),
        ...d,
      };
    });
  }

  apiTurnTo(gen: number, headingDeg: number): Promise<void> {
    return this.beginMove(gen, (d) => {
      const delta = angleDiff(this.heading, headingDeg); // (-180,180], + = CW
      if (Math.abs(delta) <= EPS) return d.resolve();
      this.activeMove = {
        kind: "turn",
        remaining: Math.abs(delta),
        dir: Math.sign(delta),
        target: normalizeDeg(headingDeg),
        ...d,
      };
    });
  }

  private beginMove(gen: number, start: (d: Deferred) => void): Promise<void> {
    return new Promise<void>((resolve, reject) => {
      if (gen !== this.generation || this.status !== "running") {
        return reject(new Cancelled());
      }
      // Blocking moves override any continuous velocity.
      this.linVel = 0;
      this.angVel = 0;
      this.activeMove?.reject(new Cancelled());
      start({ resolve, reject });
    });
  }

  apiSetDrive(gen: number, cmPerSec: number): void {
    if (gen !== this.generation || this.status !== "running") return;
    this.activeMove?.reject(new Cancelled());
    this.activeMove = null;
    this.linVel = cmPerSec;
  }

  apiSetTurn(gen: number, degPerSec: number): void {
    if (gen !== this.generation || this.status !== "running") return;
    this.activeMove?.reject(new Cancelled());
    this.activeMove = null;
    this.angVel = degPerSec;
  }

  apiStop(gen: number): void {
    if (gen !== this.generation) return;
    this.activeMove?.reject(new Cancelled());
    this.activeMove = null;
    this.linVel = 0;
    this.angVel = 0;
  }

  apiStep(gen: number): Promise<void> {
    return new Promise<void>((resolve, reject) => {
      if (gen !== this.generation || this.status !== "running") {
        return reject(new Cancelled());
      }
      this.stepWaiters.push({ resolve, reject });
    });
  }

  apiWait(gen: number, ms: number): Promise<void> {
    return new Promise<void>((resolve, reject) => {
      if (gen !== this.generation || this.status !== "running") {
        return reject(new Cancelled());
      }
      this.timeWaiters.push({ remaining: ms, d: { resolve, reject } });
    });
  }

  apiLog(line: string): void {
    this.onLog?.(line);
  }
}

/** Concrete Robot bound to one run generation. */
class RobotApi implements Robot {
  constructor(
    private sim: Simulation,
    private gen: number,
  ) {}

  position(): { x: number; y: number } {
    return { x: this.sim.pivot.x, y: this.sim.pivot.y };
  }
  cell(): { cx: number; cy: number } {
    return worldToCell(this.sim.pivot.x, this.sim.pivot.y);
  }
  heading(): number {
    return this.sim.heading;
  }
  distance(): number {
    return this.sim.apiDistance();
  }
  goal(): { x: number; y: number; radius: number } {
    const g = this.sim.world.goal;
    return { x: g.x, y: g.y, radius: g.radius };
  }
  gridSize(): { cols: number; rows: number; cellCm: number } {
    return { cols: this.sim.world.cols, rows: this.sim.world.rows, cellCm: 2 };
  }

  driveFor(cm: number): Promise<void> {
    return this.sim.apiDriveFor(this.gen, cm);
  }
  turnTo(headingDeg: number): Promise<void> {
    return this.sim.apiTurnTo(this.gen, headingDeg);
  }
  turn(deltaDeg: number): Promise<void> {
    return this.sim.apiTurnTo(this.gen, this.sim.heading + deltaDeg);
  }

  setDrive(cmPerSec: number): void {
    this.sim.apiSetDrive(this.gen, cmPerSec);
  }
  setTurn(degPerSec: number): void {
    this.sim.apiSetTurn(this.gen, degPerSec);
  }
  stop(): void {
    this.sim.apiStop(this.gen);
  }

  step(): Promise<void> {
    return this.sim.apiStep(this.gen);
  }
  wait(ms: number): Promise<void> {
    return this.sim.apiWait(this.gen, ms);
  }

  get grid(): BoolGrid {
    return this.sim.world.overlay;
  }

  get reachable(): BoolGrid {
    return this.sim.world.reachable;
  }

  get driveUp(): BoolGrid {
    return this.sim.world.driveUp;
  }

  get driveEast(): BoolGrid {
    return this.sim.world.driveEast;
  }

  log(...args: unknown[]): void {
    const line = args
      .map((a) => (typeof a === "string" ? a : JSON.stringify(a)))
      .join(" ");
    this.sim.apiLog(line);
  }
}
