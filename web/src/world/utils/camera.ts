type Vec3 = [number, number, number];

export const ART_CAMERA = {
  position: [0, 3.692496, -3.983431] as Vec3,
  target: [0, 0.35, 0] as Vec3,
  focal: 2850,
  origin: [0, 140.854] as [number, number],
};
export const ART_VIEWBOX: [number, number, number, number] = [-313, -464, 626, 703];
export const ART_SCALE = 0.5;
export const CAR_CENTRE_Z = -0.7 * ART_SCALE;

const CAR_AT = 0.76;
const CAR_AT_UPRIGHT = 0.68;
const CAR_AT_SHORT = 0.62;
const SHORT_PX = 500;
const FOCAL_PER_HEIGHT = 1.0;
const FOCAL_PER_WIDTH = 1.5;
const NEAR_DEPTH = 0.2;

export interface Camera {
  position: Vec3;
  target: Vec3;
  forward: Vec3;
  right: Vec3;
  up: Vec3;
  focal: number;
  centre: [number, number];
  car: [number, number];
  artScale: number;
}

function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function dot(a: Vec3, b: Vec3) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function unit(a: Vec3): Vec3 {
  const length = Math.hypot(...a);
  return [a[0] / length, a[1] / length, a[2] / length];
}

function axes(position: Vec3, target: Vec3) {
  const forward = unit(sub(target, position));
  const right = unit(cross([0, 1, 0], forward));
  return { forward, right, up: cross(forward, right) };
}

function carHeightShare(width: number, height: number) {
  if (height > width) return CAR_AT_UPRIGHT;
  if (height <= SHORT_PX) return CAR_AT_SHORT;
  return CAR_AT;
}

export function chaseCamera(width: number, height: number): Camera {
  const toEgo = ([x, y, z]: Vec3): Vec3 => [
    x * ART_SCALE,
    y * ART_SCALE,
    z * ART_SCALE + CAR_CENTRE_Z,
  ];
  const position = toEgo(ART_CAMERA.position);
  const target = toEgo(ART_CAMERA.target);
  const focal = Math.min(height * FOCAL_PER_HEIGHT, width * FOCAL_PER_WIDTH);
  const artScale = focal / ART_CAMERA.focal;
  const car: [number, number] = [width / 2, height * carHeightShare(width, height)];
  const centre: [number, number] = [
    car[0] - ART_CAMERA.origin[0] * artScale,
    car[1] - ART_CAMERA.origin[1] * artScale,
  ];
  return { position, target, ...axes(position, target), focal, centre, car, artScale };
}

export function project(camera: Camera, x: number, z: number, up = 0) {
  const d = sub([x, up, z], camera.position);
  const depth = dot(d, camera.forward);
  if (depth <= NEAR_DEPTH) return null;
  const scale = camera.focal / depth;
  return {
    x: camera.centre[0] + scale * dot(d, camera.right),
    y: camera.centre[1] - scale * dot(d, camera.up),
    scale,
  };
}

export function projectArtPoint(x: number, y: number, z: number) {
  const { position, focal, origin } = ART_CAMERA;
  const { forward, right, up } = axes(position, ART_CAMERA.target);
  const d = sub([x, y, z], position);
  const depth = dot(d, forward);
  if (depth < 0.1) return null;
  const k = focal / depth;
  return [k * dot(d, right) - origin[0], -k * dot(d, up) - origin[1]];
}
