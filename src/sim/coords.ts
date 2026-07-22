// Coordinate + heading conventions for the whole simulation.
//
//   World: origin at bottom-left, +x right, +y up, units = cm.
//   Heading: 0 deg = north (+y / up), CLOCKWISE positive.
//            90 = east (+x), 180 = south (-y), 270 = west (-x).
//   Cells:  cx = floor(x / CELL_CM), cy = floor(y / CELL_CM).
//           cx increases left->right, cy increases bottom->top.
//
// Because heading 0 points at +y and grows clockwise, the forward unit vector
// is (sin h, cos h): h=0 -> (0,1) up, h=90 -> (1,0) right. The robot's "right"
// is that vector rotated -90 clockwise: (cos h, -sin h).

import { CELL_CM } from "../config";

export interface Vec2 {
  x: number;
  y: number;
}

const DEG = Math.PI / 180;

/** Unit vector pointing along a heading (0=+y, clockwise positive). */
export function headingToVec(headingDeg: number): Vec2 {
  const r = headingDeg * DEG;
  return { x: Math.sin(r), y: Math.cos(r) };
}

/** Unit vector pointing to the robot's right for a given heading. */
export function rightVec(headingDeg: number): Vec2 {
  const r = headingDeg * DEG;
  return { x: Math.cos(r), y: -Math.sin(r) };
}

/** Compass bearing (deg, 0=+y, clockwise) of the vector (dx, dy). */
export function vecToHeading(dx: number, dy: number): number {
  return normalizeDeg((Math.atan2(dx, dy) * 180) / Math.PI);
}

/** Normalise degrees into [0, 360). */
export function normalizeDeg(deg: number): number {
  let d = deg % 360;
  if (d < 0) d += 360;
  return d;
}

/** Smallest signed difference a->b, in (-180, 180]. Positive = clockwise. */
export function angleDiff(a: number, b: number): number {
  let d = normalizeDeg(b - a);
  if (d > 180) d -= 360;
  return d;
}

export function worldToCell(x: number, y: number): { cx: number; cy: number } {
  return { cx: Math.floor(x / CELL_CM), cy: Math.floor(y / CELL_CM) };
}

export function cellCenter(cx: number, cy: number): Vec2 {
  return { x: (cx + 0.5) * CELL_CM, y: (cy + 0.5) * CELL_CM };
}

export function dist(a: Vec2, b: Vec2): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

export function add(a: Vec2, b: Vec2, s = 1): Vec2 {
  return { x: a.x + b.x * s, y: a.y + b.y * s };
}
