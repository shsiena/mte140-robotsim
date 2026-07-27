// PixiJS v8 renderer. Everything is drawn in world space (cm, origin
// bottom-left, +y up) inside a single container whose Y axis is flipped, so the
// sim code never has to think about screen coordinates.

import { Application, Container, Graphics, type PointData } from "pixi.js";
import { CELL_CM, COLORS, OVERLAY_ALPHA, SENSOR_MAX_CM } from "../config";
import {
  // add,
  headingToVec,
  // rightVec,
  type Vec2,
} from "../sim/coords";
import { type Box, boxCorners, robotCorners, sensorOrigin } from "../sim/geometry";
import type { Simulation } from "../sim/Simulation";
import {
  // ROBOT_LENGTH_CM,
  SENSOR_HALF_CONE_DEG,
} from "../config";

const MARGIN_PX = 24;

export class Renderer {
  app = new Application();
  private world = new Container();

  private gridG = new Graphics();
  private overlayG = new Graphics();
  private driveG = new Graphics();
  private reachableG = new Graphics();
  private trailG = new Graphics();
  private goalG = new Graphics();
  private obstacleG = new Graphics();
  private coneG = new Graphics();
  private robotG = new Graphics();

  /** Optional draft obstacle being dragged out by the editor. */
  draftBox: { x: number; y: number; w: number; h: number } | null = null;
  /** Obstacle currently selected in the editor (highlighted). */
  selectedBox: Box | null = null;

  pxPerCm = 1;
  private gridKey = "";

  async init(host: HTMLElement): Promise<void> {
    await this.app.init({
      resizeTo: host,
      background: COLORS.background,
      antialias: true,
      autoDensity: true,
      resolution: window.devicePixelRatio || 1,
    });
    host.appendChild(this.app.canvas);

    this.world.addChild(
      this.gridG,
      this.overlayG,
      this.driveG,
      this.reachableG,
      this.trailG,
      this.goalG,
      this.obstacleG,
      this.coneG,
      this.robotG,
    );
    this.app.stage.addChild(this.world);
  }

  /** Convert a global (screen) point to world cm. */
  toWorld(global: PointData): Vec2 {
    const p = this.world.toLocal(global);
    return { x: p.x, y: p.y };
  }

  private layout(sim: Simulation): void {
    const { width: W, height: H } = this.app.screen;
    const wCm = sim.world.widthCm;
    const hCm = sim.world.heightCm;
    this.pxPerCm = Math.min(
      (W - 2 * MARGIN_PX) / wCm,
      (H - 2 * MARGIN_PX) / hCm,
    );
    const boardW = wCm * this.pxPerCm;
    const boardH = hCm * this.pxPerCm;
    const originX = (W - boardW) / 2;
    const originY = (H + boardH) / 2; // world y=0 sits at the board's bottom
    this.world.position.set(originX, originY);
    this.world.scale.set(this.pxPerCm, -this.pxPerCm);
  }

  private px(n: number): number {
    return n / this.pxPerCm; // px width -> world units at current zoom
  }

  render(sim: Simulation): void {
    this.layout(sim);
    this.drawGrid(sim);
    this.drawOverlay(sim);
    this.drawDriveCells(sim);
    this.drawReachable(sim);
    this.drawTrail(sim);
    this.drawGoal(sim);
    this.drawObstacles(sim);
    this.drawCone(sim);
    this.drawRobot(sim);
  }

  private drawGrid(sim: Simulation): void {
    const key = `${sim.world.cols}x${sim.world.rows}@${this.pxPerCm.toFixed(3)}`;
    if (key === this.gridKey) return; // static unless size/zoom changed
    this.gridKey = key;

    const g = this.gridG.clear();
    const w = sim.world.widthCm;
    const h = sim.world.heightCm;
    const lw = this.px(0.75);
    for (let cx = 0; cx <= sim.world.cols; cx++) {
      const x = cx * CELL_CM;
      g.moveTo(x, 0).lineTo(x, h);
    }
    for (let cy = 0; cy <= sim.world.rows; cy++) {
      const y = cy * CELL_CM;
      g.moveTo(0, y).lineTo(w, y);
    }
    g.stroke({ width: lw, color: COLORS.grid, alpha: 0.6 });
    g.rect(0, 0, w, h).stroke({ width: this.px(2), color: COLORS.gridBorder });
  }

  private drawOverlay(sim: Simulation): void {
    const g = this.overlayG.clear();
    const raw = sim.world.overlay.raw;
    for (let cy = 0; cy < sim.world.rows; cy++) {
      for (let cx = 0; cx < sim.world.cols; cx++) {
        if (raw[cy][cx]) {
          g.rect(cx * CELL_CM, cy * CELL_CM, CELL_CM, CELL_CM);
        }
      }
    }
    g.fill({ color: COLORS.overlayTrue, alpha: OVERLAY_ALPHA });
  }

  private drawDriveCells(sim: Simulation): void {
    const g = this.driveG.clear();
    const up = sim.world.driveUp.raw;
    const east = sim.world.driveEast.raw;
    const reach = sim.world.reachable.raw;
    // Only draw the "extra" gap cells (drivable but not a turn cell), so orange
    // turn cells stay distinct.
    for (let cy = 0; cy < sim.world.rows; cy++) {
      for (let cx = 0; cx < sim.world.cols; cx++) {
        if (east[cy][cx] && !reach[cy][cx]) {
          g.rect(cx * CELL_CM, cy * CELL_CM, CELL_CM, CELL_CM);
        }
      }
    }
    g.fill({ color: COLORS.overlayDriveEast, alpha: OVERLAY_ALPHA });
    for (let cy = 0; cy < sim.world.rows; cy++) {
      for (let cx = 0; cx < sim.world.cols; cx++) {
        if (up[cy][cx] && !reach[cy][cx]) {
          g.rect(cx * CELL_CM, cy * CELL_CM, CELL_CM, CELL_CM);
        }
      }
    }
    g.fill({ color: COLORS.overlayDriveUp, alpha: OVERLAY_ALPHA });
  }

  private drawReachable(sim: Simulation): void {
    const g = this.reachableG.clear();
    const raw = sim.world.reachable.raw;
    for (let cy = 0; cy < sim.world.rows; cy++) {
      for (let cx = 0; cx < sim.world.cols; cx++) {
        if (raw[cy][cx]) {
          g.rect(cx * CELL_CM, cy * CELL_CM, CELL_CM, CELL_CM);
        }
      }
    }
    g.fill({ color: COLORS.overlayReachable, alpha: OVERLAY_ALPHA });
  }

  private drawTrail(sim: Simulation): void {
    const g = this.trailG.clear();
    if (sim.trail.length < 2) return;
    g.moveTo(sim.trail[0].x, sim.trail[0].y);
    for (let i = 1; i < sim.trail.length; i++) {
      g.lineTo(sim.trail[i].x, sim.trail[i].y);
    }
    g.stroke({ width: this.px(1.5), color: COLORS.trail, alpha: 0.9 });
  }

  private drawGoal(sim: Simulation): void {
    const g = this.goalG.clear();
    const goal = sim.world.goal;
    g.circle(goal.x, goal.y, goal.radius)
      .fill({ color: COLORS.goal, alpha: 0.18 })
      .stroke({ width: this.px(2), color: COLORS.goal });
    g.circle(goal.x, goal.y, this.px(3)).fill({ color: COLORS.goal });
  }

  private drawObstacles(sim: Simulation): void {
    const g = this.obstacleG.clear();
    // Hiding them is display-only: they still collide, and the editor still
    // picks them up. The draft box stays visible so dragging one out has
    // feedback either way.
    if (sim.world.showObstacles) {
      for (const o of sim.world.obstacles) {
        g.poly(boxCorners(o).flatMap((c) => [c.x, c.y]));
      }
      g.fill({ color: COLORS.obstacle, alpha: 0.85 });

      if (this.selectedBox) {
        g.poly(boxCorners(this.selectedBox).flatMap((c) => [c.x, c.y])).stroke({
          width: this.px(2),
          color: COLORS.robotOutline,
        });
      }
    }
    if (this.draftBox) {
      const d = this.draftBox;
      g.rect(d.x, d.y, d.w, d.h).fill({
        color: COLORS.obstacleDraft,
        alpha: 0.6,
      });
    }
  }

  private drawCone(sim: Simulation): void {
    const g = this.coneG.clear();
    const origin = sensorOrigin(sim.pivot, sim.heading);
    const reading = sim.lastSensor;
    const range = Number.isFinite(reading)
      ? Math.min(reading, SENSOR_MAX_CM)
      : SENSOR_MAX_CM;

    // Filled wedge from the sensor origin out to the measured range.
    const segs = 10;
    g.moveTo(origin.x, origin.y);
    for (let i = 0; i <= segs; i++) {
      const a =
        sim.heading - SENSOR_HALF_CONE_DEG +
        (i / segs) * 2 * SENSOR_HALF_CONE_DEG;
      const dir = headingToVec(a);
      g.lineTo(origin.x + dir.x * range, origin.y + dir.y * range);
    }
    g.lineTo(origin.x, origin.y);
    g.fill({
      color: COLORS.sensorFill,
      alpha: Number.isFinite(reading) ? 0.22 : 0.1,
    });
    g.stroke({ width: this.px(1), color: COLORS.sensorFill, alpha: 0.7 });

    // Hit marker along the centreline at the measured distance.
    if (Number.isFinite(reading)) {
      const c = headingToVec(sim.heading);
      g.circle(
        origin.x + c.x * reading,
        origin.y + c.y * reading,
        this.px(3.5),
      ).fill({ color: COLORS.sensorHit });
    }
    // Sensor mount point.
    g.circle(origin.x, origin.y, this.px(2.5)).fill({ color: COLORS.sensorHit });
  }

  private drawRobot(sim: Simulation): void {
    const g = this.robotG.clear();
    const corners = robotCorners(sim.pivot, sim.heading);
    g.poly(corners.flatMap((c) => [c.x, c.y]))
      .fill({ color: COLORS.robot, alpha: 0.9 })
      .stroke({ width: this.px(1.5), color: COLORS.robotOutline });

    // Heading marker: centre to the middle of the front edge.
    // const fwd = headingToVec(sim.heading);
    // const nose = add(sim.pivot, fwd, ROBOT_LENGTH_CM / 2);
    // g.moveTo(sim.pivot.x, sim.pivot.y)
      // .lineTo(nose.x, nose.y)
      // .stroke({ width: this.px(2), color: COLORS.heading });

    // Small tick marking the robot's right side (so orientation is unambiguous).
    // const right = rightVec(sim.heading);
    // const rp = add(sim.pivot, right, 5);
    // g.circle(rp.x, rp.y, this.px(2)).fill({ color: COLORS.heading });
  }
}
