// Geometry helpers: robot footprint, polygon collision (SAT), and the exact
// IR cone distance measurement.

import {
  type Vec2,
  add,
  angleDiff,
  dist,
  headingToVec,
  rightVec,
  vecToHeading,
} from "./coords";
import {
  PIVOT_FROM_REAR_CM,
  PIVOT_TO_FRONT_CM,
  ROBOT_WIDTH_CM,
  SENSOR_FORWARD_CM,
  SENSOR_RIGHT_CM,
  SENSOR_HALF_CONE_DEG,
  SENSOR_MAX_CM,
  SENSOR_MIN_CM,
} from "../config";

/** Axis-aligned obstacle box in world cm. */
export interface Rect {
  x: number; // min x
  y: number; // min y
  w: number;
  h: number;
}

export function rectCorners(r: Rect): Vec2[] {
  return [
    { x: r.x, y: r.y },
    { x: r.x + r.w, y: r.y },
    { x: r.x + r.w, y: r.y + r.h },
    { x: r.x, y: r.y + r.h },
  ];
}

/** Oriented obstacle box: centre, full extents, and rotation in world cm/deg. */
export interface Box {
  cx: number; // centre x
  cy: number; // centre y
  w: number; // full width
  h: number; // full height
  angle: number; // degrees, counter-clockwise (x-right, y-up)
}

/** The four world-space corners of an oriented box (consistent winding). */
export function boxCorners(b: Box): Vec2[] {
  const r = (b.angle * Math.PI) / 180;
  const c = Math.cos(r);
  const s = Math.sin(r);
  const ux: Vec2 = { x: c, y: s }; // local +x axis
  const uy: Vec2 = { x: -s, y: c }; // local +y axis
  const hw = b.w / 2;
  const hh = b.h / 2;
  return [
    { x: b.cx + ux.x * hw + uy.x * hh, y: b.cy + ux.y * hw + uy.y * hh },
    { x: b.cx + ux.x * hw - uy.x * hh, y: b.cy + ux.y * hw - uy.y * hh },
    { x: b.cx - ux.x * hw - uy.x * hh, y: b.cy - ux.y * hw - uy.y * hh },
    { x: b.cx - ux.x * hw + uy.x * hh, y: b.cy - ux.y * hw + uy.y * hh },
  ];
}

/** Point-in-convex-polygon test (works for any consistent winding). */
export function pointInConvexPoly(p: Vec2, poly: Vec2[]): boolean {
  let sign = 0;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % poly.length];
    const cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    if (Math.abs(cross) < 1e-9) continue; // on the edge
    const s = cross > 0 ? 1 : -1;
    if (sign === 0) sign = s;
    else if (s !== sign) return false;
  }
  return true;
}

/** Whether a world point lies inside an oriented box. */
export function pointInBox(p: Vec2, b: Box): boolean {
  return pointInConvexPoly(p, boxCorners(b));
}

/**
 * The four world-space corners of the robot at a given pose. The body is
 * asymmetric about the pose point: it reaches PIVOT_TO_FRONT_CM ahead and
 * PIVOT_FROM_REAR_CM behind, because the pose we track is the turning pivot,
 * not the centre of the rectangle.
 */
export function robotCorners(pivot: Vec2, headingDeg: number): Vec2[] {
  const fwd = headingToVec(headingDeg);
  const right = rightVec(headingDeg);
  const hw = ROBOT_WIDTH_CM / 2;
  return [
    add(add(pivot, fwd, PIVOT_TO_FRONT_CM), right, hw),
    add(add(pivot, fwd, PIVOT_TO_FRONT_CM), right, -hw),
    add(add(pivot, fwd, -PIVOT_FROM_REAR_CM), right, -hw),
    add(add(pivot, fwd, -PIVOT_FROM_REAR_CM), right, hw),
  ];
}

/** World position of the IR sensor origin for a given pose. */
export function sensorOrigin(pivot: Vec2, headingDeg: number): Vec2 {
  const fwd = headingToVec(headingDeg);
  const right = rightVec(headingDeg);
  return add(add(pivot, fwd, SENSOR_FORWARD_CM), right, SENSOR_RIGHT_CM);
}

// ---------------------------------------------------------------------------
// Convex polygon overlap via the Separating Axis Theorem. Both the robot and
// obstacles are convex quads, so this is exact.
// ---------------------------------------------------------------------------

function project(poly: Vec2[], axis: Vec2): { min: number; max: number } {
  let min = Infinity;
  let max = -Infinity;
  for (const p of poly) {
    const d = p.x * axis.x + p.y * axis.y;
    if (d < min) min = d;
    if (d > max) max = d;
  }
  return { min, max };
}

function axesOf(poly: Vec2[]): Vec2[] {
  const axes: Vec2[] = [];
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % poly.length];
    // edge normal
    axes.push({ x: -(b.y - a.y), y: b.x - a.x });
  }
  return axes;
}

export function polysOverlap(a: Vec2[], b: Vec2[]): boolean {
  for (const axis of [...axesOf(a), ...axesOf(b)]) {
    const pa = project(a, axis);
    const pb = project(b, axis);
    if (pa.max < pb.min || pb.max < pa.min) return false; // separating axis
  }
  return true;
}

export function robotHitsAny(
  pivot: Vec2,
  headingDeg: number,
  obstacles: Box[],
): boolean {
  const rc = robotCorners(pivot, headingDeg);
  for (const o of obstacles) {
    if (polysOverlap(rc, boxCorners(o))) return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// IR cone measurement.
//
// Reading = shortest Euclidean distance from the sensor origin to any obstacle
// point whose bearing lies within +/- halfCone of the sensor heading. Exact for
// any convex polygon: the constrained minimum over each edge sits either at that
// edge's closest point (if inside the cone), at a corner inside the cone, or
// where a cone boundary ray crosses the edge.
// ---------------------------------------------------------------------------

function inCone(origin: Vec2, headingDeg: number, p: Vec2): boolean {
  const dx = p.x - origin.x;
  const dy = p.y - origin.y;
  if (dx === 0 && dy === 0) return true;
  const bearing = vecToHeading(dx, dy);
  return Math.abs(angleDiff(headingDeg, bearing)) <= SENSOR_HALF_CONE_DEG + 1e-9;
}

function closestPointOnSegment(p: Vec2, a: Vec2, b: Vec2): Vec2 {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const len2 = abx * abx + aby * aby;
  if (len2 === 0) return a;
  let t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2;
  t = Math.max(0, Math.min(1, t));
  return { x: a.x + abx * t, y: a.y + aby * t };
}

// Intersection point of ray (origin + t*dir, t>0) with segment [a,b], or null.
function raySegment(origin: Vec2, dir: Vec2, a: Vec2, b: Vec2): Vec2 | null {
  const v1x = origin.x - a.x;
  const v1y = origin.y - a.y;
  const v2x = b.x - a.x;
  const v2y = b.y - a.y;
  const denom = dir.x * v2y - dir.y * v2x;
  if (Math.abs(denom) < 1e-12) return null; // parallel
  const t = (v2x * v1y - v2y * v1x) / denom; // along ray
  const u = (dir.x * v1y - dir.y * v1x) / denom; // along segment
  if (t <= 0 || u < 0 || u > 1) return null;
  return { x: origin.x + dir.x * t, y: origin.y + dir.y * t };
}

function polyConeMinDist(
  origin: Vec2,
  headingDeg: number,
  corners: Vec2[],
): number {
  // Origin inside the polygon => touching; treat as zero distance.
  if (pointInConvexPoly(origin, corners)) return 0;

  const edges: [Vec2, Vec2][] = corners.map((c, i) => [
    c,
    corners[(i + 1) % corners.length],
  ]);

  let best = Infinity;

  // Closest point of each edge, if it falls inside the cone.
  for (const [a, b] of edges) {
    const cp = closestPointOnSegment(origin, a, b);
    if (inCone(origin, headingDeg, cp)) best = Math.min(best, dist(origin, cp));
  }

  // Corners inside the cone.
  for (const c of corners) {
    if (inCone(origin, headingDeg, c)) best = Math.min(best, dist(origin, c));
  }

  // Where the two cone boundary rays cross the polygon edges.
  for (const boundary of [
    headingDeg - SENSOR_HALF_CONE_DEG,
    headingDeg + SENSOR_HALF_CONE_DEG,
  ]) {
    const dir = headingToVec(boundary);
    for (const [a, b] of edges) {
      const hit = raySegment(origin, dir, a, b);
      if (hit) best = Math.min(best, dist(origin, hit));
    }
  }

  return best;
}

/**
 * Measure the IR sensor. Returns a distance in [SENSOR_MIN_CM, SENSOR_MAX_CM],
 * or Infinity when nothing lies within the cone and range. Board edges are
 * invisible to the sensor (only drawn obstacles register).
 */
export function measureCone(
  origin: Vec2,
  headingDeg: number,
  obstacles: Box[],
): number {
  let best = Infinity;
  for (const o of obstacles) {
    best = Math.min(best, polyConeMinDist(origin, headingDeg, boxCorners(o)));
  }
  if (best > SENSOR_MAX_CM) return Infinity;
  return Math.max(best, SENSOR_MIN_CM);
}
