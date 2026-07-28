// Runs the compiled C++ algorithm against the same board and the same
// simulator the TypeScript one is tested with. Anything that passes here is
// the translation unit the robot runs, not a re-derivation of it.

import { expect, test } from "bun:test";
import { spawnSync } from "node:child_process";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import course from "./boards/course.json";
import { bridgeProgram } from "./bridgeProgram";
import { type BoardSnapshot, runBoard } from "./runBoard";

const CPP_DIR = join(dirname(fileURLToPath(import.meta.url)), "../cpp");
const BRIDGE = join(CPP_DIR, "build/bridge");

const build = spawnSync("make", ["-C", CPP_DIR, "build/bridge"], { encoding: "utf8" });
if (build.status !== 0) throw new Error(`building the bridge failed\n${build.stderr ?? ""}`);

test("the C++ port reaches the goal on the competition course", async () => {
  const result = await runBoard(course as BoardSnapshot, bridgeProgram(BRIDGE));

  expect(result.reason).not.toBe("collision with obstacle");
  expect(result.status).toBe("success");
  expect(result.distanceToGoalCm).toBeLessThanOrEqual(course.goal.radius);
}, 120_000);
