import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { nova5KinematicPose } from "./kinematics";
import type { TimedJoints, Vec3 } from "./types";

type ViewMode = "3d" | "top" | "side";

const COLORS = {
  background: "#f0ede5",
  grid: "#d9d4ca",
  axisX: "#b97161",
  axisY: "#668b78",
  axisZ: "#718ba2",
  actual: "#527b73",
  command: "#bd7850",
  arm: "#a9aa9f",
  gripper: "#777d73",
};

const GRIPPER = {
  maxOpeningM: 0.095,
  mainWidthM: 0.093,
  mainHeightM: 0.0523,
  mainDepthM: 0.1295,
  mainOffsetM: 0.07325,
  flangeWidthM: 0.093,
  flangeHeightM: 0.067,
  flangeDepthM: 0.0182,
  // STEP flange plane to tips is 188.041 mm; UI target remains user's 190 mm.
  eefToTipM: 0.190,
  fingertipCenterX: [-0.054600017, 0.054399983] as const,
};

export class TrajectoryPlot {
  private readonly host: HTMLElement;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(36, 1, 0.01, 30);
  private readonly controls: OrbitControls;
  private readonly trajectories = new THREE.Group();
  private readonly robot = new THREE.Group();
  private readonly commandedRobot = new THREE.Group();
  private readonly gripper = new THREE.Group();
  private readonly gripperFallback = new THREE.Group();
  private readonly fingers = new THREE.Group();
  private readonly fingertipGroups: THREE.Group[] = [];
  private actual: TimedJoints[] = [];
  private command: TimedJoints[] = [];
  private cursorNs = 0;
  private armVisible = true;
  private openingM = GRIPPER.maxOpeningM;
  private meshes: THREE.BufferGeometry[] | null = null;
  private readonly loadedMeshes: THREE.Mesh[] = [];
  private readonly fallbackLinks: THREE.Mesh[] = [];
  private readonly fallbackJoints: THREE.Mesh[] = [];
  private readonly commandBones: THREE.Line[] = [];
  private readonly actualMaterial = new THREE.MeshStandardMaterial({
    color: COLORS.arm, roughness: 0.74, metalness: 0.08,
  });
  private readonly ghostMaterial = new THREE.MeshStandardMaterial({
    color: COLORS.command, roughness: 0.8, transparent: true, opacity: 0.24,
  });

  constructor(host: HTMLElement) {
    this.host = host;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(COLORS.background);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.15;
    this.renderer.domElement.setAttribute("aria-hidden", "true");
    this.renderer.domElement.className = "trajectory-webgl";
    this.host.append(this.renderer.domElement);

    this.scene.background = new THREE.Color(COLORS.background);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xb9b3a7, 2.0));
    const keyLight = new THREE.DirectionalLight(0xffffff, 2.6);
    keyLight.position.set(-2.5, -3, 4.5);
    this.scene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(0xffffff, 1.0);
    fillLight.position.set(2, 2, 2.5);
    this.scene.add(fillLight);
    this.scene.add(this.makeGrid());
    this.scene.add(this.makeAxes());
    this.scene.add(this.trajectories, this.commandedRobot, this.robot);
    this.setupGripper();
    this.makeFallbackRobot();
    void this.loadGripperMeshes();

    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0, 0.37);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.minDistance = 0.45;
    this.controls.maxDistance = 6;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.setView("3d");

    new ResizeObserver(() => this.resize()).observe(this.host);
    this.resize();
    this.renderer.setAnimationLoop(() => {
      this.controls.update();
      this.render();
    });
    this.render();
    void this.loadRobotMeshes();
  }

  setSamples(actual: TimedJoints[], command: TimedJoints[]): void {
    this.actual = actual;
    this.command = command;
    this.replaceChildren(this.trajectories, this.makeTrajectory(actual, COLORS.actual, false), this.makeTrajectory(command, COLORS.command, true));
    this.updatePose();
    this.render();
  }

  setCursor(timeNs: number): void {
    this.cursorNs = timeNs;
    this.updatePose();
    this.render();
  }

  setArmVisible(visible: boolean): void {
    this.armVisible = visible;
    this.robot.visible = visible;
    this.commandedRobot.visible = visible;
    this.gripper.visible = visible;
    this.render();
  }

  setGripperOpeningMm(opening: number): void {
    this.openingM = THREE.MathUtils.clamp(opening / 1000, 0, GRIPPER.maxOpeningM);
    const closingTravelM = (GRIPPER.maxOpeningM - this.openingM) / 2;
    this.fingertipGroups.forEach((finger, index) => {
      finger.position.set(
        GRIPPER.fingertipCenterX[index] + (index === 0 ? 1 : -1) * closingTravelM,
        0,
        0,
      );
    });
    this.render();
  }

  setView(mode: ViewMode): void {
    const target = new THREE.Vector3(0, 0, 0.37);
    const positions: Record<ViewMode, THREE.Vector3> = {
      "3d": new THREE.Vector3(1.75, -2.2, 1.55),
      top: new THREE.Vector3(0.001, -0.001, 3.4),
      side: new THREE.Vector3(0.001, -3.4, 0.42),
    };
    this.controls.target.copy(target);
    this.camera.position.copy(target).add(positions[mode]);
    this.camera.up.set(0, mode === "top" ? 1 : 0, mode === "top" ? 0 : 1);
    this.camera.lookAt(target);
    this.controls.update();
  }

  reset(): void {
    this.setView("3d");
  }

  private resize(): void {
    const width = Math.max(1, this.host.clientWidth);
    const height = Math.max(1, this.host.clientHeight);
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.render();
  }

  private render(): void {
    this.renderer.render(this.scene, this.camera);
  }

  private makeGrid(): THREE.Group {
    const group = new THREE.Group();
    const points: number[] = [];
    for (let offset = -1; offset <= 1.0001; offset += 0.1) {
      points.push(offset, -1, 0, offset, 1, 0, -1, offset, 0, 1, offset, 0);
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(points, 3));
    group.add(new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color: COLORS.grid, transparent: true, opacity: 0.82 })));
    const axisPoints = [
      new THREE.Vector3(-1, 0, 0.001), new THREE.Vector3(1, 0, 0.001),
      new THREE.Vector3(0, -1, 0.001), new THREE.Vector3(0, 1, 0.001),
    ];
    group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(axisPoints.slice(0, 2)), new THREE.LineBasicMaterial({ color: "#aaa497" })));
    group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(axisPoints.slice(2)), new THREE.LineBasicMaterial({ color: "#aaa497" })));
    return group;
  }

  private makeAxes(): THREE.Group {
    const group = new THREE.Group();
    const axis = (end: Vec3, color: string) => {
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0.012), new THREE.Vector3(...end)]),
        new THREE.LineBasicMaterial({ color }),
      );
      group.add(line);
      const cone = new THREE.Mesh(new THREE.ConeGeometry(0.008, 0.025, 10), new THREE.MeshBasicMaterial({ color }));
      cone.position.set(...end);
      cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(...end).normalize());
      group.add(cone);
    };
    axis([0.25, 0, 0.012], COLORS.axisX);
    axis([0, 0.25, 0.012], COLORS.axisY);
    axis([0, 0, 0.262], COLORS.axisZ);
    return group;
  }

  private makeTrajectory(samples: TimedJoints[], color: string, dashed: boolean): THREE.Line | null {
    if (samples.length < 2) return null;
    const positions = samples.flatMap((sample) => sample.position);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    if (dashed) {
      const line = new THREE.Line(geometry, new THREE.LineDashedMaterial({ color, dashSize: 0.018, gapSize: 0.012, transparent: true, opacity: 0.85 }));
      line.computeLineDistances();
      return line;
    }
    return new THREE.Line(geometry, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.92 }));
  }

  private setupGripper(): void {
    const metal = new THREE.MeshStandardMaterial({ color: COLORS.gripper, roughness: 0.72, metalness: 0.12 });
    const dark = new THREE.MeshStandardMaterial({ color: "#5e625d", roughness: 0.78, metalness: 0.06 });
    // Fallback only: the viewer loads tessellated A.STEP and FLANGE.STEP meshes.
    const mainBody = new THREE.Mesh(
      new THREE.BoxGeometry(GRIPPER.mainWidthM, GRIPPER.mainHeightM, GRIPPER.mainDepthM), metal,
    );
    mainBody.position.set(0, 0, GRIPPER.mainOffsetM);
    this.gripperFallback.add(mainBody);
    const flangeBody = new THREE.Mesh(
      new THREE.BoxGeometry(GRIPPER.flangeWidthM, GRIPPER.flangeHeightM, GRIPPER.flangeDepthM), metal,
    );
    flangeBody.position.set(0, 0, GRIPPER.flangeDepthM / 2);
    this.gripperFallback.add(flangeBody);
    this.gripper.add(this.gripperFallback);
    const flange = new THREE.Mesh(new THREE.CylinderGeometry(0.0315, 0.0315, 0.008, 32), dark);
    flange.rotation.x = Math.PI / 2;
    flange.position.set(0, 0, -0.008);
    this.gripperFallback.add(flange);
    const flangeFace = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.0015, 32), metal);
    flangeFace.rotation.x = Math.PI / 2;
    flangeFace.position.set(0, 0, -0.0035);
    this.gripperFallback.add(flangeFace);

    // Moving fingertip geometry is loaded from FINGER_FINGERTIP.STL below.
    for (let index = 0; index < 2; index += 1) {
      const finger = new THREE.Group();
      this.fingers.add(finger);
      this.fingertipGroups.push(finger);
    }
    this.gripper.add(this.fingers);
    this.setGripperOpeningMm(95);
    this.scene.add(this.gripper);
  }

  private makeFallbackRobot(): void {
    const linkGeometry = new THREE.CylinderGeometry(0.035, 0.05, 1, 18);
    const jointGeometry = new THREE.SphereGeometry(0.044, 18, 12);
    for (let index = 0; index < 6; index += 1) {
      const link = new THREE.Mesh(linkGeometry, this.actualMaterial);
      this.fallbackLinks.push(link);
      this.robot.add(link);
      const bone = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: COLORS.command, transparent: true, opacity: 0.45 }));
      this.commandBones.push(bone);
      this.commandedRobot.add(bone);
    }
    for (let index = 0; index < 7; index += 1) {
      const joint = new THREE.Mesh(jointGeometry, this.actualMaterial);
      this.fallbackJoints.push(joint);
      this.robot.add(joint);
    }
  }

  private async loadGripperMeshes(): Promise<void> {
    const loader = new STLLoader();
    const cadToLink6 = new THREE.Matrix4().set(
      0.001, 0, 0, -0.020346,
      0, 0, -0.001, 0.093,
      0, 0.001, 0, 0.025724,
      0, 0, 0, 1,
    );
    try {
      const [mainBody, flange, fingertip] = await Promise.all([
        loader.loadAsync("/models/ag160/A.STL"),
        loader.loadAsync("/models/ag160/FLANGE.STL"),
        loader.loadAsync("/models/ag160/FINGER_FINGERTIP.STL"),
      ]);
      const components = [
        { geometry: mainBody, color: "#777d73" },
        { geometry: flange, color: "#a9aa9f" },
      ];
      for (const { geometry, color } of components) {
        geometry.applyMatrix4(cadToLink6);
        geometry.computeVertexNormals();
        geometry.computeBoundingSphere();
        const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color, roughness: 0.76, metalness: 0.08 }));
        this.gripper.add(mesh);
      }
      const fingertipMaterial = new THREE.MeshStandardMaterial({
        color: "#8f948d",
        roughness: 0.86,
        metalness: 0.02,
        transparent: false,
        opacity: 1,
        depthWrite: true,
        side: THREE.DoubleSide,
      });
      const instanceMatrices = [
        new THREE.Matrix4().set(
          0, 0, 0.001, 0,
          0.001, 0, 0, 0,
          0, 0.001, 0, GRIPPER.eefToTipM - 0.01725,
          0, 0, 0, 1,
        ),
        new THREE.Matrix4().set(
          0, 0, -0.001, 0,
          -0.001, 0, 0, 0,
          0, 0.001, 0, GRIPPER.eefToTipM - 0.01725,
          0, 0, 0, 1,
        ),
      ];
      this.fingertipGroups.forEach((group, index) => {
        const instanceGeometry = fingertip.clone().applyMatrix4(instanceMatrices[index]);
        instanceGeometry.computeVertexNormals();
        instanceGeometry.computeBoundingSphere();
        group.add(new THREE.Mesh(instanceGeometry, fingertipMaterial));
      });
      this.gripperFallback.visible = false;
      this.render();
    } catch {
      // The CAD-derived body blocks remain visible if the optional meshes are missing.
    }
  }

  private async loadRobotMeshes(): Promise<void> {
    const loader = new STLLoader();
    const names = ["base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"];
    try {
      const geometries = await Promise.all(names.map((name) => loader.loadAsync(`/models/nova5/${name}.STL`)));
      geometries.forEach((geometry) => {
        geometry.computeVertexNormals();
        geometry.computeBoundingSphere();
      });
      this.meshes = geometries;
      this.fallbackLinks.forEach((link) => { link.visible = false; });
      this.fallbackJoints.forEach((joint) => { joint.visible = false; });
      const materials = [
        new THREE.MeshStandardMaterial({ color: "#797f79", roughness: 0.8, metalness: 0.05 }),
        new THREE.MeshStandardMaterial({ color: "#a3aeb6", roughness: 0.72, metalness: 0.08 }),
      ];
      geometries.forEach((geometry, index) => {
        const material = index === 0 ? materials[0] : materials[1];
        const mesh = new THREE.Mesh(geometry, material);
        mesh.matrixAutoUpdate = false;
        this.loadedMeshes.push(mesh);
        this.robot.add(mesh);
      });
      this.updatePose();
      this.render();
    } catch {
      // The width-aware primitive arm remains available when optional STL files are absent.
    }
  }

  private updatePose(): void {
    const actualJoints = this.sampleAt(this.actual, this.cursorNs);
    const commandJoints = this.sampleAt(this.command, this.cursorNs);
    this.robot.visible = this.armVisible && Boolean(actualJoints);
    this.gripper.visible = this.armVisible && Boolean(actualJoints);
    if (actualJoints) {
      const pose = nova5KinematicPose(actualJoints);
      if (this.meshes) {
        this.meshes.forEach((_geometry, index) => {
          const mesh = this.loadedMeshes[index];
          if (!mesh || !pose.linkTransforms[index]) return;
          mesh.matrix.copy(this.toThreeMatrix(pose.linkTransforms[index]));
          mesh.matrixWorldNeedsUpdate = true;
        });
      } else {
        this.updatePrimitiveArm(pose.points);
      }
      this.gripper.matrixAutoUpdate = false;
      this.gripper.matrix.copy(this.toThreeMatrix(pose.linkTransforms[6]));
      this.gripper.matrixWorldNeedsUpdate = true;
    }
    if (commandJoints) {
      this.commandedRobot.visible = this.armVisible;
      const points = nova5KinematicPose(commandJoints).points;
      this.commandBones.forEach((bone, index) => {
        const start = new THREE.Vector3(...points[index]);
        const end = new THREE.Vector3(...points[index + 1]);
        const geometry = new THREE.BufferGeometry().setFromPoints([start, end]);
        bone.geometry.dispose();
        bone.geometry = geometry;
      });
    } else {
      this.commandedRobot.visible = false;
    }
  }

  private updatePrimitiveArm(points: Vec3[]): void {
    const axis = new THREE.Vector3(0, 1, 0);
    this.fallbackLinks.forEach((link, index) => {
      const start = new THREE.Vector3(...points[index]);
      const end = new THREE.Vector3(...points[index + 1]);
      const direction = end.clone().sub(start);
      link.position.copy(start).add(end).multiplyScalar(0.5);
      link.quaternion.setFromUnitVectors(axis, direction.clone().normalize());
      link.scale.y = direction.length();
    });
    this.fallbackJoints.forEach((joint, index) => joint.position.set(...points[index]));
  }

  private toThreeMatrix(values: number[]): THREE.Matrix4 {
    const matrix = new THREE.Matrix4();
    matrix.set(
      values[0], values[1], values[2], values[3],
      values[4], values[5], values[6], values[7],
      values[8], values[9], values[10], values[11],
      values[12], values[13], values[14], values[15],
    );
    return matrix;
  }

  private sampleAt(samples: TimedJoints[], timeNs: number): number[] | null {
    if (!samples.length) return null;
    if (timeNs <= samples[0].timeNs) return samples[0].joints;
    if (timeNs >= samples[samples.length - 1].timeNs) return samples[samples.length - 1].joints;
    let left = 0;
    let right = samples.length - 1;
    while (left + 1 < right) {
      const middle = (left + right) >>> 1;
      if (samples[middle].timeNs <= timeNs) left = middle;
      else right = middle;
    }
    const a = samples[left];
    const b = samples[right];
    const ratio = (timeNs - a.timeNs) / (b.timeNs - a.timeNs);
    return a.joints.map((angle, index) => angle + ((b.joints[index] ?? angle) - angle) * ratio);
  }

  private replaceChildren(parent: THREE.Group, ...children: (THREE.Object3D | null)[]): void {
    parent.clear();
    for (const child of children) if (child) parent.add(child);
  }
}
