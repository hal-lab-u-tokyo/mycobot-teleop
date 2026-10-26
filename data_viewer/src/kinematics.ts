import type { Vec3 } from "./types";

type Matrix4 = number[];

const IDENTITY: Matrix4 = [
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1, 0,
  0, 0, 0, 1,
];

// Joint origins and axes are taken from Dobot-Arm's Nova 5 URDF/Xacro model.
// URDF convention: parent_T_child = origin * revolute(joint-axis, q).
const JOINT_ORIGINS: { xyz: Vec3; rpy: Vec3 }[] = [
  { xyz: [0, 0, 0.24], rpy: [0, 0, 0] },
  { xyz: [0, 0, 0], rpy: [-1.57080287682252, 1.53586622836832, Math.PI] },
  { xyz: [-0.399756009268664, -0.0139690033141953, 0], rpy: [0, 0, 0] },
  { xyz: [-0.329798707647103, -0.0115244277211674, 0.134999532858734], rpy: [0, 0, -1.53586622840712] },
  { xyz: [0, -0.12, 0], rpy: [1.5708, 0, 0] },
  { xyz: [0, 0.088328, 0], rpy: [-1.5708, 0, 0] },
];

export const NOVA5_JOINT_LIMITS_RAD: [number, number][] = [
  [-6.28, 6.28], [-3.14, 3.14], [-2.793, 2.793],
  [-6.28, 6.28], [-6.28, 6.28], [-6.28, 6.28],
];

export interface Nova5JointFrame {
  position: Vec3;
  axis: Vec3;
  limit: [number, number];
}

export interface Nova5Pose {
  points: Vec3[];
  joints: Nova5JointFrame[];
  linkTransforms: Matrix4[];
}

function multiply(a: Matrix4, b: Matrix4): Matrix4 {
  const out = new Array<number>(16).fill(0);
  for (let row = 0; row < 4; row += 1) {
    for (let column = 0; column < 4; column += 1) {
      for (let k = 0; k < 4; k += 1) out[row * 4 + column] += a[row * 4 + k] * b[k * 4 + column];
    }
  }
  return out;
}

function translation([x, y, z]: Vec3): Matrix4 {
  return [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
}

function rotationX(angle: number): Matrix4 {
  const c = Math.cos(angle); const s = Math.sin(angle);
  return [1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1];
}

function rotationY(angle: number): Matrix4 {
  const c = Math.cos(angle); const s = Math.sin(angle);
  return [c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1];
}

function rotationZ(angle: number): Matrix4 {
  const c = Math.cos(angle); const s = Math.sin(angle);
  return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

function fromRpy(rpy: Vec3): Matrix4 {
  return multiply(multiply(rotationZ(rpy[2]), rotationY(rpy[1])), rotationX(rpy[0]));
}

function originPosition(matrix: Matrix4): Vec3 {
  return [matrix[3], matrix[7], matrix[11]];
}

/** Frame positions, revolute axes, and limits following Dobot's Nova 5 xacro. */
export function nova5KinematicPose(jointsDeg: number[]): Nova5Pose {
  let transform = [...IDENTITY];
  const points: Vec3[] = [[0, 0, 0]];
  const joints: Nova5JointFrame[] = [];
  const linkTransforms: Matrix4[] = [[...IDENTITY]];
  for (let index = 0; index < 6; index += 1) {
    const origin = JOINT_ORIGINS[index];
    transform = multiply(transform, multiply(translation(origin.xyz), fromRpy(origin.rpy)));
    const position = originPosition(transform);
    const axis: Vec3 = [transform[2], transform[6], transform[10]];
    joints.push({ position, axis, limit: NOVA5_JOINT_LIMITS_RAD[index] });
    points.push(position);
    transform = multiply(transform, rotationZ((jointsDeg[index] ?? 0) * Math.PI / 180));
    linkTransforms.push([...transform]);
  }
  return { points, joints, linkTransforms };
}

/** Returns points from base through each joint origin to the Link6 flange. */
export function nova5ForwardKinematics(jointsDeg: number[]): Vec3[] {
  return nova5KinematicPose(jointsDeg).points;
}

export function pathLength(points: Vec3[]): number {
  let length = 0;
  for (let index = 1; index < points.length; index += 1) {
    const previous = points[index - 1]; const current = points[index];
    length += Math.hypot(current[0] - previous[0], current[1] - previous[1], current[2] - previous[2]);
  }
  return length;
}

export function interpolateJoints(
  samples: { timeNs: number; joints: number[] }[],
  timeNs: number,
): number[] | null {
  if (!samples.length) return null;
  if (timeNs <= samples[0].timeNs) return [...samples[0].joints];
  const last = samples[samples.length - 1];
  if (timeNs >= last.timeNs) return [...last.joints];
  let low = 0; let high = samples.length - 1;
  while (low + 1 < high) {
    const middle = (low + high) >>> 1;
    if (samples[middle].timeNs <= timeNs) low = middle;
    else high = middle;
  }
  const left = samples[low]; const right = samples[high];
  const span = Math.max(1, right.timeNs - left.timeNs);
  const t = (timeNs - left.timeNs) / span;
  return left.joints.map((value, index) => value + ((right.joints[index] ?? value) - value) * t);
}
