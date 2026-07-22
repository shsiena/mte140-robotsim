// Central tunables for the simulation. Everything measured in centimetres /
// degrees so it maps 1:1 onto the real VEX IQ robot and its IR sensor.

export const CELL_CM = 2; // each grid cell is 2cm x 2cm

// Robot body: a rectangle with a heading. 11 x 8 cells => 22cm x 16cm.
// "length" runs along the heading (forward), "width" is side-to-side.
export const ROBOT_LENGTH_CM = 22;
export const ROBOT_WIDTH_CM = 16;

// IR sensor mounting, expressed in the robot's local frame:
//   forward  = +cm along the heading   (front edge is +LENGTH/2)
//   right    = +cm to the robot's right
// Sensor sits on the front edge, 3/4 of the way across the width from the left,
// i.e. 0.25 * width to the right of centre.
export const SENSOR_FORWARD_CM = ROBOT_LENGTH_CM / 2; // 11
export const SENSOR_RIGHT_CM = 0 * ROBOT_WIDTH_CM; // 4

// IR detection cone: 24.19 deg total, so +/- 12.095 deg about the centreline.
export const SENSOR_CONE_DEG = 24.19;
export const SENSOR_HALF_CONE_DEG = SENSOR_CONE_DEG / 2;

// Valid measurement window. Beyond max (or nothing in the cone) reads Infinity.
export const SENSOR_MIN_CM = 1;
export const SENSOR_MAX_CM = 120;

// Default animation speeds. These are purely cosmetic: the simulation is
// deterministic and the outcome does not depend on them. The sim-speed slider
// scales both.
export const DEFAULT_DRIVE_CM_S = 20;
export const DEFAULT_TURN_DEG_S = 120;

// Motion is integrated in small sub-steps to prevent tunnelling through thin
// obstacles and to stop close to the contact point.
export const MAX_SUBSTEP_CM = 0.5;
export const MAX_SUBSTEP_DEG = 1.0;

// Colours (0xRRGGBB) for the renderer.
export const COLORS = {
  background: 0x12141a,
  grid: 0x262b36,
  gridBorder: 0x3a4150,
  obstacle: 0xd94f4f,
  obstacleDraft: 0x8a3535,
  robot: 0x4a90d9,
  robotOutline: 0xbcd7f5,
  heading: 0xffffff,
  sensor: 0x2b3340,
  sensorFill: 0x4fd98a,
  sensorHit: 0xffe14f,
  goal: 0x4fd98a,
  trail: 0x7a8aa8,
  overlayTrue: 0x9b6bff, // internal-matrix cell = true
  overlayReachable: 0xff9b3d, // rotation-clear ("turn") cell — orange
  overlayDriveUp: 0x5b8cff, // fits facing north (drive-north gap) — blue
  overlayDriveEast: 0xff6ec7, // fits facing east (drive-east gap) — pink
} as const;

// Alpha for the boolean-matrix overlay tint.
export const OVERLAY_ALPHA = 0.38;
