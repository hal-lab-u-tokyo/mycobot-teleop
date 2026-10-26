import { spawn } from "node:child_process";
import { createReadStream } from "node:fs";
import { access, mkdir, readFile, readdir, rm, stat, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import type { IncomingMessage, ServerResponse } from "node:http";
import type { Plugin, ViteDevServer, PreviewServer, UserConfig } from "vite";

const here = path.dirname(fileURLToPath(import.meta.url));
const defaultRecordingsRoots = [
  process.env.MYCOBOT_RECORDINGS_DIR,
  path.resolve(here, "../data/recordings"),
  path.resolve(process.cwd(), "data/recordings"),
  path.resolve(process.cwd(), "../data/recordings"),
].filter((value): value is string => Boolean(value)).map((value) => path.resolve(value));
let recordingsRoot = defaultRecordingsRoots[0];
const cacheRoot = path.join(tmpdir(), "mycobot-data-viewer-cache");
const sessionIdPattern = /^session_[A-Za-z0-9_-]+$/;
const episodeDirPattern = /^episode-[A-Za-z0-9_-]+$/;
const jobs = new Map<string, PrepareJob>();

type PrepareJob = {
  id: string;
  status: "running" | "ready" | "error";
  progress: number;
  message: string;
};

type CameraFrame = {
  file: string;
  frameNumber: number;
  timestampMs: number;
};

type CameraManifest = {
  sourceSize: number;
  sourceMtimeMs: number;
  frames: CameraFrame[];
};

function sendJson(res: ServerResponse, status: number, value: unknown): void {
  res.statusCode = status;
  res.setHeader("Content-Type", "application/json; charset=utf-8");
  res.end(JSON.stringify(value));
}

function safeId(value: string | undefined, pattern: RegExp): string | null {
  return value && pattern.test(value) ? value : null;
}

function parseJsonLines<T>(text: string): T[] {
  return text.split(/\r?\n/).filter(Boolean).map((line) => JSON.parse(line) as T);
}

async function loadSession(sessionId: string): Promise<Record<string, unknown>> {
  recordingsRoot = await findRecordingsRoot(sessionId);
  const file = path.join(recordingsRoot, sessionId, "session.json");
  try {
    return JSON.parse(await readFile(file, "utf8")) as Record<string, unknown>;
  } catch (error) {
    throw new Error(`session.json を読み込めません: ${file} (${error instanceof Error ? error.message : String(error)})`);
  }
}

async function findRecordingsRoot(sessionId?: string): Promise<string> {
  for (const root of defaultRecordingsRoots) {
    try {
      const entries = await readdir(root, { withFileTypes: true });
      const matches = entries.some((entry) => entry.isDirectory()
        && sessionIdPattern.test(entry.name)
        && (!sessionId || entry.name === sessionId));
      if (matches) return root;
    } catch { /* try the next project-relative location */ }
  }
  throw new Error(`session_*/session.json が見つかりません。検索先: ${defaultRecordingsRoots.join(", ")}。必要なら MYCOBOT_RECORDINGS_DIR で data/recordings の場所を指定してください。`);
}

async function resolveEpisode(sessionId: string, episodeDir: string): Promise<string> {
  const session = await loadSession(sessionId);
  const episodes = Array.isArray(session.episodes) ? session.episodes : [];
  const found = episodes.find((item) => {
    if (!item || typeof item !== "object") return false;
    const record = item as Record<string, unknown>;
    return record.directory === episodeDir;
  });
  if (!found) throw new Error("Episode not found in session metadata");
  return path.join(recordingsRoot, sessionId, "episodes", episodeDir);
}

async function listSessions(): Promise<unknown[]> {
  recordingsRoot = await findRecordingsRoot();
  const dirs = await readdir(recordingsRoot, { withFileTypes: true });
  const sessions = await Promise.all(dirs
    .filter((entry) => entry.isDirectory() && sessionIdPattern.test(entry.name))
    .map(async (entry) => {
      try {
        const session = await loadSession(entry.name);
        return {
          sessionId: entry.name,
          task: session.task ?? "Untitled session",
          status: session.status ?? "unknown",
          startedAt: session.started_at_utc ?? null,
          episodes: Array.isArray(session.episodes) ? session.episodes : [],
        };
      } catch (error) {
        return {
          sessionId: entry.name,
          task: "session.json を読み込めません",
          status: "error",
          startedAt: null,
          episodes: [],
          error: error instanceof Error ? error.message : String(error),
        };
      }
    }));
  return sessions.sort((a, b) =>
    String((b as { startedAt?: string }).startedAt ?? "")
      .localeCompare(String((a as { startedAt?: string }).startedAt ?? "")));
}

function cameraCacheDir(sessionId: string, episodeDir: string, cameraName: string): string {
  return path.join(cacheRoot, sessionId, episodeDir, cameraName);
}

async function readCameraManifest(
  sessionId: string,
  episodeDir: string,
  cameraName: string,
): Promise<CameraManifest | null> {
  try {
    const file = path.join(cameraCacheDir(sessionId, episodeDir, cameraName), "manifest.json");
    const manifest = JSON.parse(await readFile(file, "utf8")) as CameraManifest;
    const bag = path.join(await resolveEpisode(sessionId, episodeDir), "cameras", `${cameraName}.bag`);
    const info = await stat(bag);
    if (manifest.sourceSize !== info.size || manifest.sourceMtimeMs !== info.mtimeMs) return null;
    return manifest;
  } catch {
    return null;
  }
}

function parseMetadata(text: string): { frameNumber: number; timestampMs: number } | null {
  const frameNumber = text.match(/^Frame Counter:\s*(\d+)/m);
  const timestamp = text.match(/^Frame Timestamp:\s*([\d.]+)/m);
  if (!frameNumber || !timestamp) return null;
  const timestampValue = Number(timestamp[1]) / 1000;
  if (!Number.isFinite(timestampValue)) return null;
  return { frameNumber: Number(frameNumber[1]), timestampMs: timestampValue };
}

async function getRsConvertPath(): Promise<string> {
  const candidates = [
    process.env.REALSENSE_RS_CONVERT,
    "/private/tmp/librealsense-2.57.6-install/bin/rs-convert",
    "/opt/homebrew/bin/rs-convert",
    "/usr/local/bin/rs-convert",
  ].filter((value): value is string => Boolean(value));
  for (const candidate of candidates) {
    try { await access(candidate); return candidate; } catch { /* try next SDK installation */ }
  }
  return "rs-convert";
}

function runConverter(args: string[], onProgress: (progress: number) => void): Promise<void> {
  const env = { ...process.env };
  const sdkLibs = [
    process.env.REALSENSE_LIB_DIR,
    "/private/tmp/librealsense-2.57.6-install/lib",
    "/opt/homebrew/opt/librealsense/lib",
  ].filter((value): value is string => Boolean(value));
  env.DYLD_LIBRARY_PATH = [env.DYLD_LIBRARY_PATH, ...sdkLibs].filter(Boolean).join(":");

  return new Promise((resolve, reject) => {
    void getRsConvertPath().then((executable) => {
      const child = spawn(executable, args, { env, stdio: ["ignore", "pipe", "pipe"] });
      let stderr = "";
      child.stdout.setEncoding("utf8");
      child.stdout.on("data", (chunk: string) => {
        const matches = chunk.match(/\d{1,3}%/g);
        if (matches?.length) onProgress(Number(matches[matches.length - 1].slice(0, -1)));
      });
      child.stderr.setEncoding("utf8");
      child.stderr.on("data", (chunk: string) => { stderr += chunk; });
      child.on("error", (error) => reject(new Error(`Could not launch rs-convert (${executable}): ${error.message}`)));
      child.on("close", (code) => {
        if (code === 0) resolve();
        else reject(new Error(`rs-convert exited with code ${code}: ${stderr.slice(-1200)}`));
      });
    }).catch(reject);
  });
}

async function prepareCamera(
  sessionId: string,
  episodeDir: string,
  cameraName: string,
  onProgress: (progress: number) => void,
): Promise<CameraManifest> {
  const existing = await readCameraManifest(sessionId, episodeDir, cameraName);
  if (existing) return existing;

  const episodePath = await resolveEpisode(sessionId, episodeDir);
  const bagPath = path.join(episodePath, "cameras", `${cameraName}.bag`);
  const bagInfo = await stat(bagPath);
  const outputDir = cameraCacheDir(sessionId, episodeDir, cameraName);
  await rm(outputDir, { recursive: true, force: true });
  await mkdir(outputDir, { recursive: true });
  const prefix = path.join(outputDir, "decoded");
  await runConverter(["-i", bagPath, "-c", "-p", prefix], onProgress);

  const files = await readdir(outputDir);
  const pngs = files.filter((name) => /^decoded_Color_.*\.png$/.test(name)).sort((a, b) => {
    const ta = Number(a.match(/_Color_([\d.]+)\.png$/)?.[1] ?? 0);
    const tb = Number(b.match(/_Color_([\d.]+)\.png$/)?.[1] ?? 0);
    return ta - tb;
  });
  const frames: CameraFrame[] = [];
  for (let index = 0; index < pngs.length; index += 1) {
    const png = pngs[index];
    const timestampMs = Number(png.match(/_Color_([\d.]+)\.png$/)?.[1]);
    const metadataName = png.replace("_Color_", "_Color_metadata_").replace(/\.png$/, ".txt");
    const metadataText = await readFile(path.join(outputDir, metadataName), "utf8");
    const metadata = parseMetadata(metadataText);
    const file = png;
    await rm(path.join(outputDir, metadataName), { force: true });
    if (metadata) frames.push({ file, ...metadata });
    onProgress(Math.min(99, 92 + Math.round(((index + 1) / Math.max(pngs.length, 1)) * 7)));
  }
  if (!frames.length) throw new Error(`No color frames decoded for camera ${cameraName}`);
  const manifest: CameraManifest = {
    sourceSize: bagInfo.size,
    sourceMtimeMs: bagInfo.mtimeMs,
    frames,
  };
  await writeFile(path.join(outputDir, "manifest.json"), JSON.stringify(manifest));
  onProgress(100);
  return manifest;
}

function queuePreparation(sessionId: string, episodeDir: string, cameraNames: string[]): PrepareJob {
  const id = `${sessionId}__${episodeDir}`;
  const previous = jobs.get(id);
  if (previous?.status === "running" || previous?.status === "ready") return previous;
  const job: PrepareJob = { id, status: "running", progress: 0, message: "Preparing camera playback" };
  jobs.set(id, job);
  void (async () => {
    try {
      for (let cameraIndex = 0; cameraIndex < cameraNames.length; cameraIndex += 1) {
        const cameraName = cameraNames[cameraIndex];
        job.message = `Decoding ${cameraName} camera`;
        await prepareCamera(sessionId, episodeDir, cameraName, (inner) => {
          job.progress = Math.round(((cameraIndex + inner / 100) / cameraNames.length) * 100);
        });
      }
      job.status = "ready";
      job.progress = 100;
      job.message = "Camera playback ready";
    } catch (error) {
      job.status = "error";
      job.message = error instanceof Error ? error.message : String(error);
    }
  })();
  return job;
}

async function handleApi(req: IncomingMessage, res: ServerResponse, next: (error?: unknown) => void): Promise<void> {
  const url = new URL(req.url ?? "/", "http://localhost");
  const pathname = url.pathname;
  try {
    if (req.method === "GET" && pathname === "/api/sessions") {
      sendJson(res, 200, await listSessions());
      return;
    }

    const episodeMatch = pathname.match(/^\/api\/sessions\/([^/]+)\/episodes\/([^/]+)$/);
    if (req.method === "GET" && episodeMatch) {
      const sessionId = safeId(episodeMatch[1], sessionIdPattern);
      const episodeDir = safeId(episodeMatch[2], episodeDirPattern);
      if (!sessionId || !episodeDir) return sendJson(res, 400, { error: "Invalid session or episode id" });
      const episodePath = await resolveEpisode(sessionId, episodeDir);
      const [session, episode, telemetryText] = await Promise.all([
        loadSession(sessionId),
        readFile(path.join(episodePath, "episode.json"), "utf8").then((text) => JSON.parse(text)),
        readFile(path.join(episodePath, "telemetry.jsonl"), "utf8"),
      ]);
      const cameraSpecs = Array.isArray(session.camera_specs) ? session.camera_specs : [];
      const cameras = await Promise.all(cameraSpecs.map(async (spec) => {
        const camera = spec as { name: string; serial: string };
        const manifest = await readCameraManifest(sessionId, episodeDir, camera.name);
        const sidecar = path.join(episodePath, "cameras", `${camera.name}.timestamps.jsonl`);
        const frames = parseJsonLines<Record<string, unknown>>(await readFile(sidecar, "utf8"));
        const bagPath = path.join(episodePath, "cameras", `${camera.name}.bag`);
        const bagInfo = await stat(bagPath);
        return {
          name: camera.name,
          serial: camera.serial,
          frameCount: frames.length,
          frames,
          bagBytes: bagInfo.size,
          prepared: Boolean(manifest),
          decodedFrames: manifest?.frames ?? [],
        };
      }));
      sendJson(res, 200, {
        sessionId,
        session,
        episodeDir,
        episode,
        telemetry: parseJsonLines<Record<string, unknown>>(telemetryText),
        cameras,
      });
      return;
    }

    const prepareMatch = pathname.match(/^\/api\/sessions\/([^/]+)\/episodes\/([^/]+)\/prepare$/);
    if (req.method === "POST" && prepareMatch) {
      const sessionId = safeId(prepareMatch[1], sessionIdPattern);
      const episodeDir = safeId(prepareMatch[2], episodeDirPattern);
      if (!sessionId || !episodeDir) return sendJson(res, 400, { error: "Invalid session or episode id" });
      const session = await loadSession(sessionId);
      const specs = Array.isArray(session.camera_specs) ? session.camera_specs as { name: string }[] : [];
      const job = queuePreparation(sessionId, episodeDir, specs.map((camera) => camera.name));
      sendJson(res, job.status === "error" ? 500 : 202, job);
      return;
    }

    const jobMatch = pathname.match(/^\/api\/jobs\/([^/]+)$/);
    if (req.method === "GET" && jobMatch) {
      const job = jobs.get(jobMatch[1]);
      sendJson(res, job ? 200 : 404, job ?? { error: "Job not found" });
      return;
    }

    const frameMatch = pathname.match(/^\/api\/sessions\/([^/]+)\/episodes\/([^/]+)\/cameras\/([^/]+)\/frames\/(\d+)$/);
    if (req.method === "GET" && frameMatch) {
      const sessionId = safeId(frameMatch[1], sessionIdPattern);
      const episodeDir = safeId(frameMatch[2], episodeDirPattern);
      const cameraName = /^[A-Za-z0-9_-]+$/.test(frameMatch[3]) ? frameMatch[3] : null;
      const frameIndex = Number(frameMatch[4]);
      if (!sessionId || !episodeDir || !cameraName) return sendJson(res, 400, { error: "Invalid frame path" });
      const manifest = await readCameraManifest(sessionId, episodeDir, cameraName);
      if (!manifest || !manifest.frames[frameIndex]) return sendJson(res, 404, { error: "Frame not prepared" });
      const file = path.join(cameraCacheDir(sessionId, episodeDir, cameraName), manifest.frames[frameIndex].file);
      res.statusCode = 200;
      res.setHeader("Content-Type", "image/png");
      res.setHeader("Cache-Control", "public, max-age=31536000, immutable");
      createReadStream(file).pipe(res);
      return;
    }

    if (pathname.startsWith("/api/")) {
      sendJson(res, 404, { error: "Unknown API endpoint" });
      return;
    }
    next();
  } catch (error) {
    sendJson(res, 500, { error: error instanceof Error ? error.message : String(error) });
  }
}

function installApi(server: ViteDevServer | PreviewServer): void {
  server.middlewares.use((req, res, next) => { void handleApi(req, res, next); });
}

function dataViewerPlugin(): Plugin {
  return {
    name: "mycobot-data-viewer-api",
    configureServer: installApi,
    configurePreviewServer: installApi,
  };
}

export default {
  root: here,
  plugins: [dataViewerPlugin()],
  server: { strictPort: true, port: 4173 },
  preview: { strictPort: true, port: 4173 },
} satisfies UserConfig;
