// Checks that the firmware's compile-time footprint traces cover exactly the
// sub-cells the simulator's runtime traces do.
//
// The two derive the same geometry through different arithmetic: the simulator
// through the platform's Math.sin/cos, the firmware through the hand-rolled
// series in cpp/include/constmath.h. Body edges land exactly on sub-cell
// boundaries at the quarter-turn headings, so a final-bit disagreement between
// them changes which sub-cells the robot believes its body occupies. Comparing
// cell by cell also proves the firmware's row-span encoding is lossless: a span
// cannot represent a gap, so a mask row with a hole in it would show up here as
// extra cells on the firmware side.

import { expect, test } from "bun:test";
import { spawnSync } from "node:child_process";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import {
  FACING_EAST,
  FACING_NORTH,
  FULL_TURN,
  QUARTER_TURN,
  START_CLEAR_RADIUS_CM,
} from "../src/robot/program";

const CPP_DIR = join(dirname(fileURLToPath(import.meta.url)), "../cpp");
const MASK_NAMES = ["FACING_NORTH", "FACING_EAST", "QUARTER_TURN", "FULL_TURN"] as const;

type MaskName = (typeof MASK_NAMES)[number];

interface Tables {
  scalars: Map<string, number>;
  masks: Map<MaskName, Set<string>>;
}

const SIMULATOR_MASKS: Record<MaskName, Int16Array> = {
  FACING_NORTH,
  FACING_EAST,
  QUARTER_TURN,
  FULL_TURN,
};

function run(command: string, args: string[]): string {
  const result = spawnSync(command, args, { encoding: "utf8" });
  if (result.status !== 0) {
    throw new Error(
      `${command} ${args.join(" ")} exited ${result.status}\n${result.stderr ?? ""}`,
    );
  }
  return result.stdout;
}

/** Expand each row span back into the individual sub-cells it stands for. */
function readFirmwareTables(): Tables {
  run("make", ["-C", CPP_DIR, "build/masks_parity"]);
  const lines = run(join(CPP_DIR, "build/masks_parity"), []).trim().split("\n");

  const scalars = new Map<string, number>();
  const masks = new Map<MaskName, Set<string>>();

  for (let i = 0; i < lines.length; ) {
    const [name, value] = lines[i].split(" ");
    i++;
    if (!MASK_NAMES.includes(name as MaskName)) {
      scalars.set(name, Number(value));
      continue;
    }
    const cells = new Set<string>();
    for (let row = 0; row < Number(value); row++, i++) {
      const [dy, xMin, xMax] = lines[i].split(" ").map(Number);
      for (let dx = xMin; dx <= xMax; dx++) cells.add(`${dx},${dy}`);
    }
    masks.set(name as MaskName, cells);
  }
  return { scalars, masks };
}

function simulatorCells(packed: Int16Array): Set<string> {
  const cells = new Set<string>();
  for (let k = 0; k < packed.length; k += 2) cells.add(`${packed[k]},${packed[k + 1]}`);
  return cells;
}

function difference(a: Set<string>, b: Set<string>): string[] {
  return [...a].filter((cell) => !b.has(cell)).sort();
}

const firmware = readFirmwareTables();

test.each(MASK_NAMES)("%s covers the same sub-cells as the simulator", (name) => {
  const expected = simulatorCells(SIMULATOR_MASKS[name]);
  const actual = firmware.masks.get(name);
  if (!actual) throw new Error(`${name} missing from the firmware dump`);

  expect({
    missing: difference(expected, actual),
    extra: difference(actual, expected),
  }).toEqual({ missing: [], extra: [] });
});

test("start clearance radius matches the simulator to the bit", () => {
  expect(firmware.scalars.get("START_CLEAR_RADIUS_CM")).toBe(START_CLEAR_RADIUS_CM);
});
