import "./style.css";
import { interpolateJoints, nova5ForwardKinematics } from "./kinematics";
import { TrajectoryPlot } from "./plot3d";
import type { CameraData, DecodedFrame, EventRow, PrepareJob, SessionData, SidecarFrame, TimedJoints, Vec3 } from "./types";

interface SessionOption {
  sessionId: string;
  task: string;
  status: string;
  startedAt: string | null;
  error?: string;
  episodes: { directory: string; episode_id: string; status: string }[];
}

interface SignalSample {
  timeNs: number;
  values: number[];
}

const $ = <T extends Element = HTMLElement>(id: string): T => {
  const element = document.getElementById(id);
  if (!element) throw new Error(`Missing page element: ${id}`);
  return element as unknown as T;
};

const sessionSelect = $<HTMLSelectElement>("session-select");
const episodeSelect = $<HTMLSelectElement>("episode-select");
const slider = $<HTMLInputElement>("time-slider");
const canvas = new TrajectoryPlot($<HTMLCanvasElement>("trajectory-canvas"));

let sessionOptions: SessionOption[] = [];
let data: SessionData | null = null;
let actualSamples: TimedJoints[] = [];
let commandSamples: TimedJoints[] = [];
let actualAcceleration: SignalSample[] = [];
let commandAcceleration: SignalSample[] = [];
let startNs = 0;
let endNs = 1;
let currentNs = 0;
let playing = false;
let animationStartedAt = 0;
let animationStartNs = 0;
let playbackRate = 1;
let toastTimer = 0;
let lastImageIndexes: Record<string, number> = {};
let resizeFrame = 0;

async function api<T>(url: string, init?: RequestInit): Promise<T> {
  const response = await fetch(url, init);
  const payload = await response.json() as T & { error?: string };
  if (!response.ok) throw new Error(payload.error ?? `Request failed (${response.status})`);
  return payload;
}

function showToast(message: string): void {
  const toast = $("toast");
  toast.textContent = message;
  toast.classList.remove("hidden");
  window.clearTimeout(toastTimer);
  toastTimer = window.setTimeout(() => toast.classList.add("hidden"), 5200);
}

function setLoading(show: boolean, detail = "収録ディレクトリを検索しています"): void {
  $("loading-detail").textContent = detail;
  $("loading-overlay").classList.toggle("hidden", !show);
}

function sessionUrl(sessionId: string, episodeDir: string): string {
  return `/api/sessions/${encodeURIComponent(sessionId)}/episodes/${encodeURIComponent(episodeDir)}`;
}

function fmtDuration(seconds: number): string {
  const clamped = Math.max(0, seconds);
  const minutes = Math.floor(clamped / 60);
  const whole = Math.floor(clamped % 60);
  const millis = Math.floor((clamped - Math.floor(clamped)) * 1000);
  return `${String(minutes).padStart(2, "0")}:${String(whole).padStart(2, "0")}.${String(millis).padStart(3, "0")}`;
}

function fmtDate(value: unknown): string {
  if (typeof value !== "string") return "—";
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return value;
  return new Intl.DateTimeFormat("ja-JP", {
    year: "numeric", month: "2-digit", day: "2-digit", hour: "2-digit", minute: "2-digit", hour12: false,
  }).format(date);
}

function eventTime(event: EventRow): number | null {
  const value = event.host_monotonic_ns ?? event.host_send_start_ns ?? event.host_response_ns;
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function jointsTime(event: EventRow): number | null {
  const value = event.host_response_ns ?? event.host_send_start_ns ?? event.host_request_start_ns;
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function initializeData(next: SessionData): void {
  data = next;
  lastImageIndexes = {};
  const events = next.telemetry;
  const actualRows = events.filter((event) => event.event_type === "robot_state"
    && event.source === "nova" && Array.isArray(event.joint_position_deg));
  const commandRows = events.filter((event) => event.event_type === "action"
    && Array.isArray(event.joint_target_deg));

  actualSamples = actualRows.flatMap((event) => {
    const timeNs = jointsTime(event);
    const joints = event.joint_position_deg;
    if (timeNs === null || !joints) return [];
    return [{ timeNs, joints, chain: nova5ForwardKinematics(joints), position: nova5ForwardKinematics(joints).at(-1) as Vec3 }];
  }).sort((a, b) => a.timeNs - b.timeNs);
  commandSamples = commandRows.flatMap((event) => {
    const timeNs = jointsTime(event);
    const joints = event.joint_target_deg;
    if (timeNs === null || !joints) return [];
    const chain = nova5ForwardKinematics(joints);
    return [{ timeNs, joints, chain, position: chain[chain.length - 1] }];
  }).sort((a, b) => a.timeNs - b.timeNs);
  actualAcceleration = estimateAngularAcceleration(actualSamples);
  commandAcceleration = estimateAngularAcceleration(commandSamples);

  const sidecarTimes = next.cameras.flatMap((camera) => camera.frames.map((frame) => frame.host_receive_monotonic_ns));
  const sampleTimes = [...actualSamples, ...commandSamples].map((sample) => sample.timeNs);
  const allTimes = [...sidecarTimes, ...sampleTimes].filter(Number.isFinite);
  const startedEvent = events.find((event) => event.event_type === "episode_started");
  const recordedStart = typeof startedEvent?.host_monotonic_ns === "number" ? startedEvent.host_monotonic_ns : null;
  startNs = Math.min(...(recordedStart ? [recordedStart, ...allTimes] : allTimes));
  endNs = Math.max(...allTimes);
  if (!Number.isFinite(startNs) || !Number.isFinite(endNs) || endNs <= startNs) {
    startNs = 0; endNs = 1;
  }
  currentNs = startNs;
  canvas.setSamples(actualSamples, commandSamples);
  updateMetadata();
  renderSignalChart();
  renderTimelineMarkers();
  updateCursor();

}

function updateMetadata(): void {
  if (!data) return;
  const session = data.session;
  $("session-task").textContent = String(session.task ?? "タスク名なし");
  $("session-date").textContent = fmtDate(session.started_at_utc);
  $("session-id").textContent = data.sessionId;
  const duration = (endNs - startNs) / 1e9;

  for (const camera of data.cameras) {
    const serialId = `${camera.name}-serial`;
    const fpsId = `${camera.name}-fps`;
    const serial = document.getElementById(serialId);
    const fps = document.getElementById(fpsId);
    if (serial) serial.textContent = camera.serial;
    if (fps) fps.textContent = `${cameraFps(camera).toFixed(1)} FPS`;
  }

  $("footer-message").textContent = `${duration.toFixed(1)} 秒`;
  $("duration-time").textContent = fmtDuration(duration);
  $("start-label").textContent = fmtDuration(0);
  $("middle-label").textContent = fmtDuration(duration / 2);
  $("end-label").textContent = fmtDuration(duration);
}

function cameraFps(camera: CameraData): number {
  if (camera.frames.length < 2) return 0;
  const span = camera.frames[camera.frames.length - 1].host_receive_monotonic_ns - camera.frames[0].host_receive_monotonic_ns;
  return span > 0 ? (camera.frames.length - 1) * 1e9 / span : 0;
}

function estimateAngularAcceleration(samples: TimedJoints[]): SignalSample[] {
  if (samples.length < 3) return [];
  const unique = samples.filter((sample, index) => index === 0 || sample.timeNs > samples[index - 1].timeNs);
  if (unique.length < 3) return [];
  return unique.map((sample, index) => {
    const first = Math.max(0, index - 2);
    const last = Math.min(unique.length - 1, index + 2);
    const window = unique.slice(first, last + 1);
    const points = window.map((row) => ({ x: (row.timeNs - sample.timeNs) / 1e9, joints: row.joints }));
    const sums = Array.from({ length: 5 }, (_, power) => points.reduce((total, point) => total + point.x ** power, 0));
    const matrix = [
      [sums[0], sums[1], sums[2]],
      [sums[1], sums[2], sums[3]],
      [sums[2], sums[3], sums[4]],
    ];
    const values = Array.from({ length: 6 }, (_, joint) => {
      const rhs = [0, 1, 2].map((power) => points.reduce((total, point) => total + point.x ** power * (point.joints[joint] ?? 0), 0));
      const coefficients = solveThreeByThree(matrix, rhs);
      return coefficients ? 2 * coefficients[2] : 0;
    });
    return { timeNs: sample.timeNs, values };
  });
}

function solveThreeByThree(matrix: number[][], vector: number[]): number[] | null {
  const rows = matrix.map((row, index) => [...row, vector[index]]);
  for (let column = 0; column < 3; column += 1) {
    let pivot = column;
    for (let row = column + 1; row < 3; row += 1) {
      if (Math.abs(rows[row][column]) > Math.abs(rows[pivot][column])) pivot = row;
    }
    if (Math.abs(rows[pivot][column]) < 1e-12) return null;
    [rows[column], rows[pivot]] = [rows[pivot], rows[column]];
    const divisor = rows[column][column];
    for (let cell = column; cell < 4; cell += 1) rows[column][cell] /= divisor;
    for (let row = 0; row < 3; row += 1) {
      if (row === column) continue;
      const factor = rows[row][column];
      for (let cell = column; cell < 4; cell += 1) rows[row][cell] -= factor * rows[column][cell];
    }
  }
  return [rows[0][3], rows[1][3], rows[2][3]];
}

function renderSignalChart(): void {
  const colors = ["#527b73", "#bd7850", "#7186a0", "#a07867", "#758758", "#82708a"];
  const jointSignals = (samples: TimedJoints[]): SignalSample[] => samples.map((sample) => ({ timeNs: sample.timeNs, values: sample.joints }));
  const panes = [
    { id: "signal-chart-123", accelerationId: "accel-chart-123", joints: [0, 1, 2] },
    { id: "signal-chart-456", accelerationId: "accel-chart-456", joints: [3, 4, 5] },
  ];
  for (const pane of panes) {
    renderSignalPane($<SVGSVGElement>(pane.id), jointSignals(actualSamples), jointSignals(commandSamples), pane.joints, colors, "°", true);
    renderSignalPane($<SVGSVGElement>(pane.accelerationId), actualAcceleration, commandAcceleration, pane.joints, colors, "°/s²", false);
  }
}

function renderSignalPane(
  svg: SVGSVGElement,
  measured: SignalSample[],
  commanded: SignalSample[],
  joints: number[],
  colors: string[],
  unit: string,
  showJointLabels: boolean,
): void {
  const width = Math.max(320, svg.clientWidth || 900); const height = 137;
  svg.setAttribute("viewBox", `0 0 ${width} ${height}`);
  const left = 45; const right = width - 14; const top = 25; const bottom = height - 21;
  const values = [...measured, ...commanded].flatMap((sample) => joints.map((joint) => sample.values[joint]).filter(Number.isFinite));
  if (!values.length) {
    svg.innerHTML = `<text class="chart-label" x="${width / 2}" y="${height / 2}" text-anchor="middle">関節データがありません</text>`;
    return;
  }
  const minValue = Math.min(...values); const maxValue = Math.max(...values);
  const padding = Math.max(unit === "°" ? 4 : 10, (maxValue - minValue) * 0.1);
  const minY = minValue - padding; const maxY = maxValue + padding;
  const x = (timeNs: number) => left + ((timeNs - startNs) / Math.max(1, endNs - startNs)) * (right - left);
  const y = (value: number) => bottom - ((value - minY) / Math.max(1e-5, maxY - minY)) * (bottom - top);
  const makePath = (samples: SignalSample[], joint: number) => samples
    .filter((sample) => Number.isFinite(sample.values[joint]))
    .map((sample, index) => `${index === 0 ? "M" : "L"}${x(sample.timeNs).toFixed(2)},${y(sample.values[joint]).toFixed(2)}`).join(" ");
  const grid = Array.from({ length: 3 }, (_, index) => {
    const ratio = index / 2; const value = maxY - ratio * (maxY - minY); const yy = top + ratio * (bottom - top);
    return `<line class="grid-line" x1="${left}" y1="${yy}" x2="${right}" y2="${yy}"/><text class="chart-label" x="${left - 7}" y="${yy + 3}" text-anchor="end">${value.toFixed(0)}</text>`;
  }).join("");
  const ticks = Array.from({ length: 5 }, (_, index) => {
    const ratio = index / 4; const xx = left + ratio * (right - left);
    return `<text class="chart-label" x="${xx}" y="${height - 3}" text-anchor="${index === 0 ? "start" : index === 4 ? "end" : "middle"}">${fmtDuration(((endNs - startNs) / 1e9) * ratio)}</text>`;
  }).join("");
  const jointPaths = joints.map((joint) => `<path class="chart-line command" stroke="${colors[joint]}" d="${makePath(commanded, joint)}"/><path class="chart-line actual" stroke="${colors[joint]}" d="${makePath(measured, joint)}"/>`).join("");
  const legend = showJointLabels ? joints.map((joint, index) => `<text class="chart-joint-label" fill="${colors[joint]}" x="${right - (joints.length - index) * 37}" y="14">J${joint + 1}</text>`).join("") : "";
  const cursorX = x(currentNs);
  svg.innerHTML = `${grid}${ticks}${legend}${jointPaths}<line class="chart-cursor" x1="${cursorX}" y1="${top}" x2="${cursorX}" y2="${bottom}"/>`;
}

function renderTimelineMarkers(): void {
  const container = $("event-markers");
  container.innerHTML = "";
  if (!data) return;
  const markers = data.telemetry.filter((event) => ["episode_started", "episode_outcome", "camera_stopped"].includes(String(event.event_type)));
  for (const event of markers) {
    const time = eventTime(event);
    if (time === null) continue;
    const marker = document.createElement("span");
    marker.className = "event-marker";
    marker.style.left = `${Math.max(0, Math.min(100, ((time - startNs) / (endNs - startNs)) * 100))}%`;
    marker.title = String(event.event_type ?? "event");
    container.append(marker);
  }
}

function nearestSidecarFrame(camera: CameraData, timeNs: number): SidecarFrame | null {
  const frames = camera.frames;
  if (!frames.length) return null;
  let low = 0; let high = frames.length - 1;
  while (low < high) {
    const mid = (low + high) >>> 1;
    if (frames[mid].host_receive_monotonic_ns < timeNs) low = mid + 1;
    else high = mid;
  }
  if (low > 0) {
    const before = frames[low - 1]; const after = frames[low];
    if (timeNs - before.host_receive_monotonic_ns <= after.host_receive_monotonic_ns - timeNs) return before;
  }
  return frames[low];
}

function updateCameraImage(camera: CameraData): void {
  const img = $<HTMLImageElement>(`${camera.name}-image`);
  const placeholder = $(`${camera.name}-placeholder`);
  if (!camera.prepared || !camera.decodedFrames.length) {
    placeholder.classList.remove("hidden");
    img.classList.remove("ready");
    return;
  }
  const frame = nearestSidecarFrame(camera, currentNs);
  if (!frame) return;
  let index = camera.decodedFrames.findIndex((item) => item.frameNumber === frame.frame_number);
  if (index < 0) {
    let minDifference = Number.POSITIVE_INFINITY;
    camera.decodedFrames.forEach((item, currentIndex) => {
      const difference = Math.abs(item.timestampMs - frame.timestamp_ms);
      if (difference < minDifference) { minDifference = difference; index = currentIndex; }
    });
  }
  if (index < 0) return;
  if (lastImageIndexes[camera.name] !== index) {
    lastImageIndexes[camera.name] = index;
    img.src = `/api/sessions/${encodeURIComponent(data?.sessionId ?? "")}/episodes/${encodeURIComponent(data?.episodeDir ?? "")}/cameras/${encodeURIComponent(camera.name)}/frames/${index}`;
    img.onload = () => { img.classList.add("ready"); placeholder.classList.add("hidden"); };
  }
  $(`${camera.name}-frame-label`).textContent = `FRAME ${String(frame.frame_number).padStart(4, "0")}`;
  $(`${camera.name}-timecode`).textContent = fmtDuration((frame.host_receive_monotonic_ns - startNs) / 1e9);
}

function updateCursor(): void {
  if (!data) return;
  const span = Math.max(1, endNs - startNs);
  const ratio = Math.max(0, Math.min(1, (currentNs - startNs) / span));
  slider.value = String(Math.round(ratio * 1000));
  $("timeline-progress").style.width = `${ratio * 100}%`;
  $("current-time").textContent = fmtDuration((currentNs - startNs) / 1e9);
  canvas.setCursor(currentNs);
  renderSignalChart();

  const actualJoints = interpolateJoints(actualSamples, currentNs);
  if (actualJoints) {
    const position = nova5ForwardKinematics(actualJoints).at(-1) as Vec3;
    const mm = position.map((value) => Math.round(value * 1000));
    $("eef-coordinates").innerHTML = `X ${mm[0]} <b>Y ${mm[1]}</b> Z ${mm[2]} mm`;
    $("cursor-position").textContent = `X ${mm[0]} · Y ${mm[1]} · Z ${mm[2]} mm`;
  }
  for (const camera of data.cameras) updateCameraImage(camera);
  const front = data.cameras.find((camera) => camera.name === "front");
  const wrist = data.cameras.find((camera) => camera.name === "wrist");
  if (front && wrist) {
    const frontFrame = nearestSidecarFrame(front, currentNs);
    const wristFrame = nearestSidecarFrame(wrist, currentNs);
    const deltaMs = frontFrame && wristFrame
      ? Math.abs(frontFrame.host_receive_monotonic_ns - wristFrame.host_receive_monotonic_ns) / 1e6
      : Number.NaN;
    $("sync-delta").textContent = Number.isFinite(deltaMs) ? `frame Δ ${deltaMs.toFixed(1)} ms` : "frame Δ —";
  }
}

function animationFrame(now: number): void {
  if (!playing || !data) return;
  const next = animationStartNs + (now - animationStartedAt) * 1e6 * playbackRate;
  if (next >= endNs) {
    currentNs = endNs;
    playing = false;
    $("play-button").textContent = "▶";
    updateCursor();
    return;
  }
  currentNs = next;
  updateCursor();
  requestAnimationFrame(animationFrame);
}

function togglePlayback(): void {
  if (!data) return;
  if (playing) {
    playing = false;
    $("play-button").textContent = "▶";
    return;
  }
  if (currentNs >= endNs) currentNs = startNs;
  playing = true;
  animationStartedAt = performance.now();
  animationStartNs = currentNs;
  $("play-button").textContent = "Ⅱ";
  requestAnimationFrame(animationFrame);
}

function seekToRatio(value: number): void {
  playing = false;
  $("play-button").textContent = "▶";
  currentNs = startNs + (endNs - startNs) * value;
  updateCursor();
}

async function prepareCameras(next: SessionData): Promise<void> {
  if (!next.cameras.some((camera) => !camera.prepared)) return;
  const url = `/api/sessions/${encodeURIComponent(next.sessionId)}/episodes/${encodeURIComponent(next.episodeDir)}/prepare`;
  const initial = await api<PrepareJob>(url, { method: "POST" });
  if (initial.status === "ready") return;
  const loadingDetail = $("loading-detail");
  $("loading-overlay").classList.remove("hidden");
  for (;;) {
    const job = await api<PrepareJob>(`/api/jobs/${encodeURIComponent(initial.id)}`);
    loadingDetail.textContent = `${job.message} · ${job.progress}%`;
    if (job.status === "error") throw new Error(job.message);
    if (job.status === "ready") break;
    await new Promise((resolve) => window.setTimeout(resolve, 600));
  }
}

async function loadEpisode(sessionId: string, episodeDir: string): Promise<void> {
  setLoading(true, "収録メタデータを読み込んでいます");
  playing = false;
  $("play-button").textContent = "▶";
  try {
    const next = await api<SessionData>(sessionUrl(sessionId, episodeDir));
    initializeData(next);
    setLoading(next.cameras.some((camera) => !camera.prepared), "カメラ映像を展開しています");
    await prepareCameras(next);
    const refreshed = await api<SessionData>(sessionUrl(sessionId, episodeDir));
    initializeData(refreshed);
    setLoading(false);
    $("footer-message").textContent = `${((endNs - startNs) / 1e9).toFixed(1)} 秒`;
  } catch (error) {
    setLoading(false);
    showToast(error instanceof Error ? error.message : String(error));
    $("footer-message").textContent = "";
  }
}

async function loadSessions(preferredSession?: string): Promise<void> {
  setLoading(true, "収録ディレクトリを検索しています");
  try {
    sessionOptions = await api<SessionOption[]>("/api/sessions");
    sessionSelect.innerHTML = "";
    for (const session of sessionOptions) {
      const option = document.createElement("option");
      option.value = session.sessionId;
      option.textContent = session.error ? `${session.sessionId} · エラー` : `${session.task} · ${fmtDate(session.startedAt)}`;
      sessionSelect.append(option);
    }
    if (!sessionOptions.length) {
      setLoading(false);
      showToast("セッションがありません。MYCOBOT_RECORDINGS_DIR で収録先を指定できます。");
      return;
    }
    const selected = sessionOptions.find((session) => session.sessionId === preferredSession) ?? sessionOptions[0];
    sessionSelect.value = selected.sessionId;
    selectEpisodeOptions(selected);
    setLoading(false);
    const episodeDir = selected.episodes[0]?.directory;
    if (episodeDir) await loadEpisode(selected.sessionId, episodeDir);
    else showToast(selected.error ?? "このセッションにエピソードがありません");
  } catch (error) {
    setLoading(false);
    showToast(error instanceof Error ? error.message : String(error));
  }
}

function selectEpisodeOptions(session: SessionOption, preferredEpisode?: string): void {
  episodeSelect.innerHTML = "";
  for (const episode of session.episodes) {
    const option = document.createElement("option");
    option.value = episode.directory;
    option.textContent = `episode ${episode.episode_id} · ${episode.status}`;
    episodeSelect.append(option);
  }
  if (preferredEpisode && session.episodes.some((episode) => episode.directory === preferredEpisode)) {
    episodeSelect.value = preferredEpisode;
  }
}

sessionSelect.addEventListener("change", () => {
  const session = sessionOptions.find((item) => item.sessionId === sessionSelect.value);
  if (!session) return;
  selectEpisodeOptions(session);
  const episodeDir = session.episodes[0]?.directory;
  if (episodeDir) void loadEpisode(session.sessionId, episodeDir);
});

episodeSelect.addEventListener("change", () => {
  if (sessionSelect.value && episodeSelect.value) void loadEpisode(sessionSelect.value, episodeSelect.value);
});
$("refresh-button").addEventListener("click", () => { void loadSessions(sessionSelect.value); });
$("play-button").addEventListener("click", togglePlayback);
slider.addEventListener("input", () => seekToRatio(Number(slider.value) / 1000));
$("step-back").addEventListener("click", () => seekToRatio(Math.max(0, (currentNs - startNs) / (endNs - startNs) - 1 / 30 / ((endNs - startNs) / 1e9))));
$("step-forward").addEventListener("click", () => seekToRatio(Math.min(1, (currentNs - startNs) / (endNs - startNs) + 1 / 30 / ((endNs - startNs) / 1e9))));
$("playback-speed").addEventListener("change", (event) => {
  playbackRate = Number((event.currentTarget as HTMLSelectElement).value) || 1;
});
document.querySelectorAll<HTMLButtonElement>(".view-toggle[data-view]").forEach((button) => {
  button.addEventListener("click", () => {
    document.querySelectorAll(".view-toggle[data-view]").forEach((item) => item.classList.remove("active"));
    button.classList.add("active");
    canvas.setView(button.dataset.view as "3d" | "top" | "side");
  });
});
const armToggle = $<HTMLButtonElement>("toggle-arm-model");
armToggle.addEventListener("click", () => {
  const visible = armToggle.getAttribute("aria-pressed") !== "true";
  armToggle.setAttribute("aria-pressed", String(visible));
  armToggle.classList.toggle("active", visible);
  canvas.setArmVisible(visible);
});
$("reset-camera").addEventListener("click", () => {
  canvas.reset();
  document.querySelectorAll(".view-toggle[data-view]").forEach((item) => item.classList.toggle("active", (item as HTMLElement).dataset.view === "3d"));
});
window.addEventListener("resize", () => {
  window.cancelAnimationFrame(resizeFrame);
  resizeFrame = window.requestAnimationFrame(renderSignalChart);
});

void loadSessions();
