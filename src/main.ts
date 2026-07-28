// App entry point: wires the renderer, world, simulation, and UI together and
// runs the render loop.

import "./style.css";
import { Renderer } from "./render/Renderer";
import { run as initialProgram } from "./robot/program";
import type { Robot } from "./robot/types";
import { Simulation } from "./sim/Simulation";
import { World } from "./sim/World";
import { EditorUI } from "./ui/controls";

async function main(): Promise<void> {
  const stage = document.getElementById("stage")!;
  const panel = document.getElementById("panel")!;

  const renderer = new Renderer();
  await renderer.init(stage);

  const world = new World();
  const sim = new Simulation(world);

  // The active program. Vite HMR swaps it in place without a full reload.
  let program: (r: Robot) => Promise<void> = initialProgram;
  if (import.meta.hot) {
    import.meta.hot.accept("./robot/program", (mod) => {
      if (mod) {
        program = (mod as unknown as { run: typeof initialProgram }).run;
        sim.reset();
      }
    });
  }

  const ui = new EditorUI(sim, renderer, panel, () => sim.run(program));

  let last = performance.now();
  const loop = (now: number): void => {
    let dt = Math.min((now - last) / 1000, 0.05) * ui.simSpeed;
    last = now;

    if (ui.paused) {
      if (ui.consumeStep()) sim.tick(ui.fixedStep * ui.simSpeed);
    } else {
      sim.tick(dt);
    }

    renderer.render(sim);
    ui.refresh();
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}

main().catch((e) => console.error(e));
