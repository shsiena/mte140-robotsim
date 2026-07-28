// Coordinate + heading conventions for the whole simulation.
//
//   World: origin at bottom-left, +x right, +y up, units = cm.
//   Heading: 0 deg = north (+y / up), CLOCKWISE positive.
//            90 = east (+x), 180 = south (-y), 270 = west (-x).
//   Cells:  the board is drawn in CELL_CM cells, but every map index is a
//           SUBCELL_CM sub-cell: cx = floor(x / SUBCELL_CM), likewise cy.
//           cx increases left->right, cy increases bottom->top.
//
// Because heading 0 points at +y and grows clockwise, the forward unit vector
// is (sin h, cos h): h=0 -> (0,1) up, h=90 -> (1,0) right. The robot's "right"
// is that vector rotated -90 clockwise: (cos h, -sin h).

import { SUBCELL_CM } from "../config";

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

export function worldToSubCell(x: number, y: number): { cx: number; cy: number } {
  return { cx: Math.floor(x / SUBCELL_CM), cy: Math.floor(y / SUBCELL_CM) };
}

export function subCellCenter(cx: number, cy: number): Vec2 {
  return { x: (cx + 0.5) * SUBCELL_CM, y: (cy + 0.5) * SUBCELL_CM };
}

/**
 * Snap a world point onto the centre of the sub-cell containing it. Poses the
 * robot may spin at must land here: the clearance grids test the footprint at
 * sub-cell centres and carry no allowance for the pivot sitting off one.
 */
export function snapToSubCellCenter(p: Vec2): Vec2 {
  const c = worldToSubCell(p.x, p.y);
  return subCellCenter(c.cx, c.cy);
}

export function dist(a: Vec2, b: Vec2): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

export function add(a: Vec2, b: Vec2, s = 1): Vec2 {
  return { x: a.x + b.x * s, y: a.y + b.y * s };
}
