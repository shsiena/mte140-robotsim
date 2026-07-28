import { expect, test } from "bun:test";
import course from "./boards/course.json";
import { type BoardSnapshot, runBoard } from "./runBoard";

test("reaches the goal on the competition course", async () => {
  const result = await runBoard(course as BoardSnapshot);

  expect(result.reason).not.toBe("collision with obstacle");
  expect(result.status).toBe("success");
  expect(result.distanceToGoalCm).toBeLessThanOrEqual(course.goal.radius);
}, 60_000);
