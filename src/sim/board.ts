// Serialising the editable board to JSON. The snapshot is self-describing on
// purpose: it carries the config the robot's maps and footprint depend on, not
// just the obstacle layout, so a run reproduced from it behaves identically
// even if the defaults in config.ts have moved on since.

import {
  CELL_CM,
  GRID_RESOLUTION_PER_CELL,
  PIVOT_FROM_REAR_CM,
  ROBOT_LENGTH_CM,
  ROBOT_WIDTH_CM,
  SENSOR_CONE_DEG,
  SENSOR_FORWARD_CM,
  SENSOR_MAX_CM,
  SENSOR_MIN_CM,
} from "../config";
import type { World } from "./World";

export const BOARD_SNAPSHOT_VERSION = 1;

/** Trim float noise so hand-editing a snapshot stays pleasant. */
function round(n: number): number {
  return Math.round(n * 1000) / 1000;
}

export function boardToJson(world: World): string {
  return JSON.stringify(
    {
      version: BOARD_SNAPSHOT_VERSION,
      grid: {
        cols: world.cols,
        rows: world.rows,
        cellCm: CELL_CM,
        resolutionPerCell: GRID_RESOLUTION_PER_CELL,
      },
      start: {
        x: round(world.start.x),
        y: round(world.start.y),
        heading: round(world.start.heading),
      },
      goal: {
        x: round(world.goal.x),
        y: round(world.goal.y),
        radius: round(world.goal.radius),
      },
      obstacles: world.obstacles.map((o) => ({
        cx: round(o.cx),
        cy: round(o.cy),
        w: round(o.w),
        h: round(o.h),
        angle: round(o.angle),
      })),
      robot: {
        lengthCm: ROBOT_LENGTH_CM,
        widthCm: ROBOT_WIDTH_CM,
        pivotFromRearCm: round(PIVOT_FROM_REAR_CM),
      },
      sensor: {
        forwardCm: round(SENSOR_FORWARD_CM),
        coneDeg: SENSOR_CONE_DEG,
        minCm: SENSOR_MIN_CM,
        maxCm: SENSOR_MAX_CM,
      },
    },
    null,
    2,
  );
}
