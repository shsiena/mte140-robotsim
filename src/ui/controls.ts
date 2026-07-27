// Builds the side panel: run controls, the map editor (draw obstacles, place
// start pose and goal, resize the grid), display toggles, telemetry and a log.

import type { FederatedPointerEvent } from "pixi.js";
import { CELL_CM } from "../config";
import type { Renderer } from "../render/Renderer";
import { worldToCell } from "../sim/coords";
import { type Box, type Rect, pointInBox } from "../sim/geometry";
import type { SimStatus, Simulation } from "../sim/Simulation";

type EditMode = "draw" | "erase" | "rotate" | "start" | "goal";

const FIXED_STEP = 1 / 60;

export class EditorUI {
  simSpeed = 1;
  paused = false;
  private stepRequested = false;
  private mode: EditMode = "draw";

  private dragStart: { x: number; y: number } | null = null;
  private rotateTarget: Box | null = null;
  private grabOffset = 0; // radians: pointer angle - box angle at grab

  constructor(
    private sim: Simulation,
    private renderer: Renderer,
    private panel: HTMLElement,
    private onRun: () => void,
  ) {
    this.build();
    this.wirePointer();
    sim.onStatus = (s, r) => this.setStatus(s, r);
    sim.onLog = (line) => this.appendLog(line);
  }

  consumeStep(): boolean {
    if (this.stepRequested) {
      this.stepRequested = false;
      return true;
    }
    return false;
  }

  // --- DOM ------------------------------------------------------------------

  private $(sel: string): HTMLElement {
    return this.panel.querySelector(sel) as HTMLElement;
  }

  private build(): void {
    this.panel.innerHTML = `
      <h1>MTE 140 Robot Sim</h1>
      <div class="hint">Edit <code>src/robot/program.ts</code>, then Run.</div>

      <h2>Run</h2>
      <div class="row">
        <button id="run" class="primary">▶ Run</button>
        <button id="reset">↺ Reset</button>
        <button id="pause">⏸ Pause</button>
        <button id="step">⏭ Step</button>
      </div>
      <div class="row">
        <label>Speed
          <input id="speed" type="range" min="0.1" max="4" step="0.1" value="1" />
        </label>
        <span id="speedVal">1.0×</span>
      </div>

      <h2>Status</h2>
      <div class="row"><span id="status" class="status idle">idle</span></div>
      <div class="telemetry" id="telemetry"></div>

      <h2>Display</h2>
      <div class="row">
        <label><input id="showObstacles" type="checkbox" /> Obstacles</label>
      </div>

      <h2>Environment</h2>
      <div class="row">
        <label>Cols <input id="cols" type="number" min="4" max="200" /></label>
        <label>Rows <input id="rows" type="number" min="4" max="200" /></label>
        <button id="applySize">Apply</button>
      </div>
      <div class="row">
        <label>Start heading
          <input id="startHeading" type="number" min="0" max="359" step="5" />
        </label>
      </div>
      <div class="row">
        <label>Goal radius
          <input id="goalRadius" type="number" min="1" max="40" step="1" />
        </label>
      </div>

      <h2>Edit mode</h2>
      <div class="row">
        <button data-mode="draw" class="mode active">Draw box</button>
        <button data-mode="rotate" class="mode">Rotate</button>
        <button data-mode="erase" class="mode">Erase</button>
        <button data-mode="start" class="mode">Start</button>
        <button data-mode="goal" class="mode">Goal</button>
      </div>
      <div class="row"><button id="clearBoxes">Clear all boxes</button></div>
      <div class="hint" id="modeHint"></div>

      <h2>Log</h2>
      <div id="log"></div>
    `;

    const w = this.sim.world;
    (this.$("#cols") as HTMLInputElement).value = String(w.cols);
    (this.$("#rows") as HTMLInputElement).value = String(w.rows);
    (this.$("#startHeading") as HTMLInputElement).value = String(w.start.heading);
    (this.$("#goalRadius") as HTMLInputElement).value = String(w.goal.radius);

    this.$("#run").onclick = () => this.onRun();
    this.$("#reset").onclick = () => this.sim.reset();
    this.$("#pause").onclick = () => this.togglePause();
    this.$("#step").onclick = () => {
      this.stepRequested = true;
    };

    const speed = this.$("#speed") as HTMLInputElement;
    speed.oninput = () => {
      this.simSpeed = Number(speed.value);
      this.$("#speedVal").textContent = `${this.simSpeed.toFixed(1)}×`;
    };

    const showObstacles = this.$("#showObstacles") as HTMLInputElement;
    showObstacles.checked = w.showObstacles;
    showObstacles.onchange = () => {
      w.showObstacles = showObstacles.checked;
    };

    this.$("#applySize").onclick = () => this.applySize();
    (this.$("#startHeading") as HTMLInputElement).onchange = (e) => {
      w.start.heading = Number((e.target as HTMLInputElement).value) || 0;
      if (this.sim.status === "idle") this.sim.reset();
    };
    (this.$("#goalRadius") as HTMLInputElement).onchange = (e) => {
      w.goal.radius = Number((e.target as HTMLInputElement).value) || 4;
    };

    for (const btn of this.panel.querySelectorAll<HTMLElement>(".mode")) {
      btn.onclick = () => this.setMode(btn.dataset.mode as EditMode);
    }
    this.$("#clearBoxes").onclick = () => {
      w.obstacles = [];
      this.renderer.selectedBox = null;
      this.rotateTarget = null;
    };
    this.updateModeHint();
  }

  private togglePause(): void {
    this.paused = !this.paused;
    this.$("#pause").textContent = this.paused ? "▶ Resume" : "⏸ Pause";
    this.$("#pause").classList.toggle("active", this.paused);
  }

  private applySize(): void {
    const cols = Number((this.$("#cols") as HTMLInputElement).value);
    const rows = Number((this.$("#rows") as HTMLInputElement).value);
    if (cols >= 4 && rows >= 4) {
      this.sim.world.resize(cols, rows);
      this.sim.reset();
    }
  }

  private setMode(mode: EditMode): void {
    this.mode = mode;
    if (mode !== "rotate") {
      this.renderer.selectedBox = null;
      this.rotateTarget = null;
    }
    for (const btn of this.panel.querySelectorAll<HTMLElement>(".mode")) {
      btn.classList.toggle("active", btn.dataset.mode === mode);
    }
    this.updateModeHint();
  }

  private updateModeHint(): void {
    const hints: Record<EditMode, string> = {
      draw: "Drag to draw an obstacle box (any position, fractional cells).",
      erase: "Click an obstacle to remove it.",
      rotate: "Click an obstacle and drag to rotate it about its centre.",
      start: "Click to move the robot's start position.",
      goal: "Click to move the goal.",
    };
    this.$("#modeHint").textContent = hints[this.mode];
  }

  // --- canvas editing -------------------------------------------------------

  private wirePointer(): void {
    const stage = this.renderer.app.stage;
    stage.eventMode = "static";
    stage.hitArea = this.renderer.app.screen;
    stage.on("pointerdown", (e: FederatedPointerEvent) => this.onDown(e));
    stage.on("pointermove", (e: FederatedPointerEvent) => this.onMove(e));
    stage.on("pointerup", (e: FederatedPointerEvent) => this.onUp(e));
    stage.on("pointerupoutside", (e: FederatedPointerEvent) => this.onUp(e));
  }

  private onDown(e: FederatedPointerEvent): void {
    const p = this.renderer.toWorld(e.global);
    const w = this.sim.world;
    switch (this.mode) {
      case "draw":
        this.dragStart = { x: p.x, y: p.y };
        break;
      case "erase": {
        // Remove the topmost obstacle under the cursor.
        for (let i = w.obstacles.length - 1; i >= 0; i--) {
          if (pointInBox(p, w.obstacles[i])) {
            if (w.obstacles[i] === this.renderer.selectedBox) {
              this.renderer.selectedBox = null;
            }
            w.obstacles.splice(i, 1);
            break;
          }
        }
        break;
      }
      case "rotate": {
        for (let i = w.obstacles.length - 1; i >= 0; i--) {
          const b = w.obstacles[i];
          if (pointInBox(p, b)) {
            this.rotateTarget = b;
            this.renderer.selectedBox = b;
            this.grabOffset =
              Math.atan2(p.y - b.cy, p.x - b.cx) - (b.angle * Math.PI) / 180;
            break;
          }
        }
        break;
      }
      case "start": {
        const c = worldToCell(p.x, p.y);
        w.start.x = (c.cx + 0.5) * CELL_CM;
        w.start.y = (c.cy + 0.5) * CELL_CM;
        if (this.sim.status !== "running") this.sim.reset();
        break;
      }
      case "goal":
        w.goal.x = p.x;
        w.goal.y = p.y;
        break;
    }
  }

  private onMove(e: FederatedPointerEvent): void {
    const p = this.renderer.toWorld(e.global);
    if (this.mode === "draw" && this.dragStart) {
      this.renderer.draftBox = this.rectFrom(this.dragStart, p);
    } else if (this.mode === "rotate" && this.rotateTarget) {
      const b = this.rotateTarget;
      const ang = Math.atan2(p.y - b.cy, p.x - b.cx) - this.grabOffset;
      b.angle = (ang * 180) / Math.PI;
    }
  }

  private onUp(e: FederatedPointerEvent): void {
    if (this.mode === "draw" && this.dragStart) {
      const box = this.rectFrom(this.dragStart, this.renderer.toWorld(e.global));
      if (box.w >= 1 && box.h >= 1) {
        this.sim.world.obstacles.push({
          cx: box.x + box.w / 2,
          cy: box.y + box.h / 2,
          w: box.w,
          h: box.h,
          angle: 0,
        });
      }
      this.dragStart = null;
      this.renderer.draftBox = null;
    }
    this.rotateTarget = null;
  }

  private rectFrom(a: { x: number; y: number }, b: { x: number; y: number }): Rect {
    return {
      x: Math.min(a.x, b.x),
      y: Math.min(a.y, b.y),
      w: Math.abs(a.x - b.x),
      h: Math.abs(a.y - b.y),
    };
  }

  // --- live readouts --------------------------------------------------------

  private setStatus(status: SimStatus, reason: string): void {
    const el = this.$("#status");
    el.className = `status ${status}`;
    el.textContent = reason ? `${status} — ${reason}` : status;
  }

  private appendLog(line: string): void {
    const log = this.$("#log");
    log.textContent += line + "\n";
    log.scrollTop = log.scrollHeight;
  }

  refresh(): void {
    const s = this.sim;
    const c = worldToCell(s.pivot.x, s.pivot.y);
    const d = s.lastSensor;
    this.$("#telemetry").innerHTML =
      `<div><span>pos</span><span>(${s.pivot.x.toFixed(1)}, ${s.pivot.y.toFixed(1)}) cm</span></div>` +
      `<div><span>cell</span><span>(${c.cx}, ${c.cy})</span></div>` +
      `<div><span>heading</span><span>${s.heading.toFixed(1)}°</span></div>` +
      `<div><span>IR</span><span>${Number.isFinite(d) ? d.toFixed(1) + " cm" : "∞"}</span></div>`;
  }

  get fixedStep(): number {
    return FIXED_STEP;
  }
}
