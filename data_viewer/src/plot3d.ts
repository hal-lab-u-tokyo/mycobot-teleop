import { nova5KinematicPose } from "./kinematics";
import type { TimedJoints, Vec3 } from "./types";

type ViewMode = "3d" | "top" | "side";
type Matrix4 = number[];
type Triangle = [Vec3, Vec3, Vec3];

const COLORS = {
  grid: "#d9d4ca",
  axisX: "#b97161",
  axisY: "#668b78",
  axisZ: "#718ba2",
  actual: "#527b73",
  command: "#bd7850",
  arm: "#a9aa9f",
  armEdge: "#73796f",
  joint: "#888c82",
  range: "#c4926f",
};

export class TrajectoryPlot {
  private readonly canvas: HTMLCanvasElement;
  private readonly context: CanvasRenderingContext2D;
  private actual: TimedJoints[] = [];
  private command: TimedJoints[] = [];
  private cursorNs = 0;
  private yaw = -0.82;
  private pitch = 0.48;
  private zoom = 1;
  private mode: ViewMode = "3d";
  private armVisible = true;
  private drag: { x: number; y: number; yaw: number; pitch: number } | null = null;
  private meshes: Triangle[][] | null = null;

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    const context = canvas.getContext("2d");
    if (!context) throw new Error("Canvas 2D is not available");
    this.context = context;
    this.canvas.addEventListener("pointerdown", (event) => this.onPointerDown(event));
    this.canvas.addEventListener("pointermove", (event) => this.onPointerMove(event));
    this.canvas.addEventListener("pointerup", () => this.onPointerUp());
    this.canvas.addEventListener("pointercancel", () => this.onPointerUp());
    this.canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      this.zoom = Math.max(0.55, Math.min(2.5, this.zoom * (event.deltaY < 0 ? 1.08 : 0.92)));
      this.draw();
    }, { passive: false });
    new ResizeObserver(() => this.draw()).observe(canvas);
    void this.loadRobotMeshes();
  }

  setSamples(actual: TimedJoints[], command: TimedJoints[]): void {
    this.actual = actual;
    this.command = command;
    this.draw();
  }

  setCursor(timeNs: number): void {
    this.cursorNs = timeNs;
    this.draw();
  }

  setArmVisible(visible: boolean): void {
    this.armVisible = visible;
    this.draw();
  }

  setView(mode: ViewMode): void {
    this.mode = mode;
    if (mode === "3d") { this.yaw = -0.82; this.pitch = 0.48; }
    if (mode === "top") { this.yaw = -0.82; this.pitch = 1.52; }
    if (mode === "side") { this.yaw = -1.56; this.pitch = 0.16; }
    this.draw();
  }

  reset(): void {
    this.zoom = 1;
    this.setView("3d");
  }

  private cameraBasis(): { right: Vec3; up: Vec3; target: Vec3 } {
    const target: Vec3 = [-0.03, 0.03, 0.38];
    const cp = Math.cos(this.pitch); const sp = Math.sin(this.pitch);
    const dir: Vec3 = [cp * Math.cos(this.yaw), cp * Math.sin(this.yaw), sp];
    const right: Vec3 = [-Math.sin(this.yaw), Math.cos(this.yaw), 0];
    const up: Vec3 = [-sp * Math.cos(this.yaw), -sp * Math.sin(this.yaw), cp];
    return { right, up, target: [target[0] + dir[0] * 2, target[1] + dir[1] * 2, target[2] + dir[2] * 2] };
  }

  private project(point: Vec3, width: number, height: number): [number, number] {
    const basis = this.cameraBasis();
    const aim: Vec3 = [basis.target[0] - (Math.cos(this.pitch) * Math.cos(this.yaw)) * 2,
      basis.target[1] - (Math.cos(this.pitch) * Math.sin(this.yaw)) * 2,
      basis.target[2] - Math.sin(this.pitch) * 2];
    const relative: Vec3 = [point[0] - aim[0], point[1] - aim[1], point[2] - aim[2]];
    const x = relative[0] * basis.right[0] + relative[1] * basis.right[1] + relative[2] * basis.right[2];
    const y = relative[0] * basis.up[0] + relative[1] * basis.up[1] + relative[2] * basis.up[2];
    const scale = Math.min(width, height) * 0.70 * this.zoom;
    return [width * 0.52 + x * scale, height * 0.53 - y * scale];
  }

  private draw(): void {
    const rect = this.canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.round(rect.width * dpr); const height = Math.round(rect.height * dpr);
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width; this.canvas.height = height;
    }
    const ctx = this.context;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const logicalW = rect.width; const logicalH = rect.height;
    ctx.clearRect(0, 0, logicalW, logicalH);
    this.drawGrid(logicalW, logicalH);
    this.drawTrajectory(this.command, COLORS.command, logicalW, logicalH, [5, 5], 1.5, 0.75);
    this.drawTrajectory(this.actual, COLORS.actual, logicalW, logicalH, [], 2.1, 0.9);
    const actualJoints = this.sampleAt(this.actual, this.cursorNs);
    const commandJoints = this.sampleAt(this.command, this.cursorNs);
    if (this.armVisible) {
      if (commandJoints) this.drawRobot(nova5KinematicPose(commandJoints).points, true, logicalW, logicalH);
      if (actualJoints) {
        const pose = nova5KinematicPose(actualJoints);
        if (this.meshes) this.drawStlModel(pose.linkTransforms, logicalW, logicalH);
        else this.drawRobot(pose.points, false, logicalW, logicalH);
        this.drawJointRanges(pose.joints, logicalW, logicalH);
      }
    }
    this.drawAxes(logicalW, logicalH);
  }

  private async loadRobotMeshes(): Promise<void> {
    const names = ["base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"];
    try {
      const meshes = await Promise.all(names.map(async (name) => {
        const response = await fetch(`/models/nova5/${name}.STL`);
        if (!response.ok) throw new Error(`Missing Nova5 mesh: ${name}`);
        return this.parseStl(await response.arrayBuffer());
      }));
      if (meshes.some((mesh) => mesh.length === 0)) throw new Error("Empty Nova5 mesh");
      this.meshes = meshes;
      this.draw();
    } catch {
      // Keep the built-in, width-aware schematic when the optional STL files are absent.
    }
  }

  private parseStl(buffer: ArrayBuffer): Triangle[] {
    const bytes = new Uint8Array(buffer);
    const prefix = new TextDecoder().decode(bytes.subarray(0, Math.min(bytes.length, 256))).trimStart();
    if (prefix.startsWith("solid") && new TextDecoder().decode(bytes.subarray(Math.max(0, bytes.length - 64))).includes("endsolid")) {
      const text = new TextDecoder().decode(bytes);
      const vertexPattern = /vertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)/g;
      const vertices: Vec3[] = [];
      for (const match of text.matchAll(vertexPattern)) vertices.push([Number(match[1]), Number(match[2]), Number(match[3])]);
      const triangles: Triangle[] = [];
      for (let index = 0; index + 2 < vertices.length; index += 3) triangles.push([vertices[index], vertices[index + 1], vertices[index + 2]]);
      return triangles;
    }
    if (buffer.byteLength < 84) return [];
    const view = new DataView(buffer);
    const count = view.getUint32(80, true);
    if (count === 0 || 84 + count * 50 > buffer.byteLength) return [];
    const triangles: Triangle[] = [];
    for (let index = 0; index < count; index += 1) {
      const offset = 84 + index * 50 + 12;
      const vertex = (point: number): Vec3 => {
        const vertexOffset = offset + point * 12;
        return [view.getFloat32(vertexOffset, true), view.getFloat32(vertexOffset + 4, true), view.getFloat32(vertexOffset + 8, true)];
      };
      triangles.push([vertex(0), vertex(1), vertex(2)]);
    }
    return triangles;
  }

  private drawStlModel(linkTransforms: Matrix4[], width: number, height: number): void {
    if (!this.meshes) return;
    const triangles: { points: [number, number][]; depth: number; color: string }[] = [];
    const eye: Vec3 = [Math.cos(this.pitch) * Math.cos(this.yaw), Math.cos(this.pitch) * Math.sin(this.yaw), Math.sin(this.pitch)];
    const light = this.normalize([0.35, -0.42, 0.84]);
    this.meshes.forEach((mesh, linkIndex) => {
      const transform = linkTransforms[linkIndex];
      if (!transform) return;
      const baseColor: Vec3 = linkIndex === 0 ? [121, 128, 122] : linkIndex === 6 ? [166, 166, 159] : [157, 169, 181];
      for (const triangle of mesh) {
        const world = triangle.map((vertex) => this.transformPoint(transform, vertex)) as Triangle;
        const normal = this.normalize(this.cross(
          [world[1][0] - world[0][0], world[1][1] - world[0][1], world[1][2] - world[0][2]],
          [world[2][0] - world[0][0], world[2][1] - world[0][1], world[2][2] - world[0][2]],
        ));
        const shade = 0.32 + 0.68 * Math.max(0, this.dot(normal, light));
        const color = `rgb(${Math.round(baseColor[0] * shade)}, ${Math.round(baseColor[1] * shade)}, ${Math.round(baseColor[2] * shade)})`;
        const points = world.map((vertex) => this.project(vertex, width, height));
        const depth = world.reduce((sum, vertex) => sum + this.dot(vertex, eye), 0) / 3;
        triangles.push({ points, depth, color });
      }
    });
    triangles.sort((a, b) => a.depth - b.depth);
    const ctx = this.context;
    ctx.save(); ctx.lineJoin = "round";
    for (const triangle of triangles) {
      ctx.beginPath(); ctx.moveTo(...triangle.points[0]); ctx.lineTo(...triangle.points[1]); ctx.lineTo(...triangle.points[2]); ctx.closePath();
      ctx.fillStyle = triangle.color; ctx.fill();
    }
    ctx.restore();
  }

  private transformPoint(matrix: Matrix4, point: Vec3): Vec3 {
    return [
      matrix[0] * point[0] + matrix[1] * point[1] + matrix[2] * point[2] + matrix[3],
      matrix[4] * point[0] + matrix[5] * point[1] + matrix[6] * point[2] + matrix[7],
      matrix[8] * point[0] + matrix[9] * point[1] + matrix[10] * point[2] + matrix[11],
    ];
  }

  private dot(a: Vec3, b: Vec3): number {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  private sampleAt(samples: TimedJoints[], timeNs: number): number[] | null {
    if (!samples.length) return null;
    if (timeNs <= samples[0].timeNs) return samples[0].joints;
    if (timeNs >= samples[samples.length - 1].timeNs) return samples[samples.length - 1].joints;
    let left = 0; let right = samples.length - 1;
    while (left + 1 < right) {
      const mid = (left + right) >>> 1;
      if (samples[mid].timeNs <= timeNs) left = mid; else right = mid;
    }
    const a = samples[left]; const b = samples[right];
    const ratio = (timeNs - a.timeNs) / (b.timeNs - a.timeNs);
    return a.joints.map((angle, index) => angle + ((b.joints[index] ?? angle) - angle) * ratio);
  }

  private drawGrid(width: number, height: number): void {
    const ctx = this.context;
    ctx.lineWidth = 1;
    for (let x = -1; x <= 1.0001; x += 0.1) {
      const a = this.project([x, -1, 0], width, height); const b = this.project([x, 1, 0], width, height);
      ctx.strokeStyle = Math.abs(x) < 0.001 ? "#aaa497" : COLORS.grid;
      ctx.globalAlpha = Math.abs(x) < 0.001 ? 0.9 : 0.75;
      ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
    }
    for (let y = -1; y <= 1.0001; y += 0.1) {
      const a = this.project([-1, y, 0], width, height); const b = this.project([1, y, 0], width, height);
      ctx.strokeStyle = Math.abs(y) < 0.001 ? "#aaa497" : COLORS.grid;
      ctx.globalAlpha = Math.abs(y) < 0.001 ? 0.9 : 0.75;
      ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
    }
    ctx.globalAlpha = 1;
  }

  private drawTrajectory(samples: TimedJoints[], color: string, width: number, height: number, dash: number[], lineWidth: number, alpha: number): void {
    if (samples.length < 2) return;
    const ctx = this.context;
    ctx.save(); ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.lineJoin = "round"; ctx.setLineDash(dash);
    ctx.beginPath();
    samples.forEach((sample, index) => {
      const pos = sample.position;
      const point = this.project(pos, width, height);
      if (index === 0) ctx.moveTo(point[0], point[1]); else ctx.lineTo(point[0], point[1]);
    });
    ctx.stroke(); ctx.restore();
  }

  private drawRobot(points: Vec3[], ghost: boolean, width: number, height: number): void {
    this.drawBase(width, height, ghost);
    const radii = [0.078, 0, 0.058, 0.048, 0.038, 0.029];
    for (let index = 0; index < points.length - 1; index += 1) {
      if (radii[index] <= 0) continue;
      this.drawLink(points[index], points[index + 1], radii[index], ghost, width, height);
    }
    if (ghost) return;
    const jointRadii = [0.084, 0.070, 0.065, 0.055, 0.043, 0.034, 0.027];
    points.forEach((position, index) => this.drawJoint(position, jointRadii[index], index === 0, width, height));
  }

  private drawBase(width: number, height: number, ghost: boolean): void {
    const ctx = this.context;
    const rings = (z: number, rx: number, ry: number) => Array.from({ length: 32 }, (_, index) => {
      const angle = (index / 32) * Math.PI * 2;
      return this.project([rx * Math.cos(angle), ry * Math.sin(angle), z], width, height);
    });
    const lower = rings(0.015, 0.115, 0.105);
    const upper = rings(0.075, 0.102, 0.095);
    ctx.save();
    ctx.globalAlpha = ghost ? 0.3 : 1;
    ctx.fillStyle = ghost ? COLORS.command : "#777d73";
    ctx.strokeStyle = ghost ? COLORS.command : COLORS.armEdge;
    ctx.lineWidth = ghost ? 1.5 : 1;
    if (ghost) ctx.setLineDash([3, 4]);
    ctx.beginPath();
    ctx.moveTo(...lower[0]);
    for (let i = 1; i < lower.length; i += 1) ctx.lineTo(...lower[i]);
    for (let i = upper.length - 1; i >= 0; i -= 1) ctx.lineTo(...upper[i]);
    ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(...upper[0]);
    for (let i = 1; i < upper.length; i += 1) ctx.lineTo(...upper[i]);
    ctx.closePath();
    ctx.fillStyle = ghost ? COLORS.command : "#a3a59a";
    ctx.fill(); ctx.stroke();
    ctx.restore();
  }

  private drawLink(start: Vec3, end: Vec3, radius: number, ghost: boolean, width: number, height: number): void {
    const ctx = this.context;
    const a = this.project(start, width, height); const b = this.project(end, width, height);
    const dx = b[0] - a[0]; const dy = b[1] - a[1]; const length = Math.hypot(dx, dy);
    if (length < 1) return;
    const nx = -dy / length; const ny = dx / length;
    const scale = Math.min(width, height) * 0.70 * this.zoom;
    const r1 = Math.max(3, radius * scale); const r2 = Math.max(3, radius * scale * 0.78);
    ctx.save();
    ctx.globalAlpha = ghost ? 0.46 : 1;
    ctx.lineCap = "round"; ctx.lineJoin = "round";
    ctx.strokeStyle = ghost ? COLORS.command : COLORS.armEdge;
    ctx.lineWidth = ghost ? 2 : 1;
    ctx.setLineDash(ghost ? [4, 4] : []);
    ctx.beginPath();
    ctx.moveTo(a[0] + nx * r1, a[1] + ny * r1);
    ctx.lineTo(b[0] + nx * r2, b[1] + ny * r2);
    ctx.lineTo(b[0] - nx * r2, b[1] - ny * r2);
    ctx.lineTo(a[0] - nx * r1, a[1] - ny * r1);
    ctx.closePath();
    ctx.fillStyle = ghost ? COLORS.command : COLORS.arm; ctx.fill(); ctx.stroke();
    if (!ghost) {
      ctx.setLineDash([]); ctx.strokeStyle = "#e4e1d6"; ctx.lineWidth = Math.max(1, r1 * 0.22);
      ctx.beginPath(); ctx.moveTo(a[0] + nx * r1 * 0.16, a[1] + ny * r1 * 0.16); ctx.lineTo(b[0] + nx * r2 * 0.16, b[1] + ny * r2 * 0.16); ctx.stroke();
    }
    ctx.restore();
  }

  private drawJoint(position: Vec3, radius: number, base: boolean, width: number, height: number): void {
    const ctx = this.context; const [x, y] = this.project(position, width, height);
    const r = Math.max(4, radius * Math.min(width, height) * 0.70 * this.zoom);
    ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fillStyle = base ? "#858a7f" : COLORS.joint;
    ctx.strokeStyle = COLORS.armEdge; ctx.lineWidth = 1.2;
    ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.arc(x - r * 0.18, y - r * 0.22, r * 0.48, Math.PI * 1.05, Math.PI * 1.85);
    ctx.strokeStyle = "#e6e3d8"; ctx.lineWidth = Math.max(1, r * 0.09); ctx.stroke();
  }

  private drawJointRanges(joints: ReturnType<typeof nova5KinematicPose>["joints"], width: number, height: number): void {
    const ctx = this.context; const scale = Math.min(width, height) * 0.70 * this.zoom;
    joints.forEach((joint, index) => {
      const [lower, upper] = joint.limit;
      const span = Math.min(Math.PI * 2, upper - lower);
      const fallback: Vec3 = Math.abs(joint.axis[2]) > 0.92 ? [0, 1, 0] : [0, 0, 1];
      const u = this.normalize(this.cross(joint.axis, fallback));
      const v = this.normalize(this.cross(joint.axis, u));
      const radius = [0.112, 0.09, 0.078, 0.065, 0.052, 0.041][index];
      const steps = Math.max(20, Math.ceil(span / (Math.PI / 24)));
      const points: [number, number][] = [];
      for (let step = 0; step <= steps; step += 1) {
        const angle = lower + span * (step / steps);
        const point: Vec3 = [
          joint.position[0] + radius * (u[0] * Math.cos(angle) + v[0] * Math.sin(angle)),
          joint.position[1] + radius * (u[1] * Math.cos(angle) + v[1] * Math.sin(angle)),
          joint.position[2] + radius * (u[2] * Math.cos(angle) + v[2] * Math.sin(angle)),
        ];
        points.push(this.project(point, width, height));
      }
      ctx.save(); ctx.globalAlpha = 0.52; ctx.strokeStyle = COLORS.range; ctx.lineWidth = Math.max(1, scale * 0.0025);
      ctx.beginPath(); points.forEach(([x, y], pointIndex) => pointIndex === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)); ctx.stroke();
      for (const endpoint of [points[0], points[points.length - 1]]) {
        ctx.beginPath(); ctx.arc(endpoint[0], endpoint[1], 2.1, 0, Math.PI * 2); ctx.fillStyle = COLORS.range; ctx.fill();
      }
      ctx.restore();
    });
  }

  private cross(a: Vec3, b: Vec3): Vec3 {
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  }

  private normalize(a: Vec3): Vec3 {
    const length = Math.hypot(...a) || 1;
    return [a[0] / length, a[1] / length, a[2] / length];
  }

  private drawAxes(width: number, height: number): void {
    const ctx = this.context; const start: Vec3 = [0, 0, 0.01]; const length = 0.25;
    const axes: [Vec3, string, string][] = [
      [[length, 0, 0], COLORS.axisX, "X"],
      [[0, length, 0], COLORS.axisY, "Y"],
      [[0, 0, length], COLORS.axisZ, "Z"],
    ];
    const s = this.project(start, width, height);
    for (const [direction, color, label] of axes) {
      const end = this.project(direction, width, height);
      ctx.strokeStyle = color; ctx.lineWidth = 1.4; ctx.beginPath(); ctx.moveTo(...s); ctx.lineTo(...end); ctx.stroke();
      ctx.fillStyle = color; ctx.font = "9px 'DM Mono', monospace"; ctx.fillText(label, end[0] + 4, end[1] - 4);
    }
  }

  private onPointerDown(event: PointerEvent): void {
    this.canvas.setPointerCapture(event.pointerId);
    this.canvas.classList.add("dragging");
    this.drag = { x: event.clientX, y: event.clientY, yaw: this.yaw, pitch: this.pitch };
  }

  private onPointerMove(event: PointerEvent): void {
    if (!this.drag) return;
    this.yaw = this.drag.yaw + (event.clientX - this.drag.x) * 0.008;
    this.pitch = Math.max(-0.15, Math.min(1.55, this.drag.pitch + (event.clientY - this.drag.y) * 0.008));
    this.draw();
  }

  private onPointerUp(): void {
    this.drag = null;
    this.canvas.classList.remove("dragging");
  }
}
