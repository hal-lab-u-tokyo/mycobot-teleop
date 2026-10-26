export type Vec3 = [number, number, number];

export interface EventRow {
  event_type?: string;
  host_monotonic_ns?: number;
  host_request_start_ns?: number;
  host_response_ns?: number;
  host_send_start_ns?: number;
  joint_position_deg?: number[];
  joint_target_deg?: number[];
  source?: string;
  action_name?: string;
  command_error?: string | null;
  camera_name?: string;
  frame_count?: number;
  dropped_frames?: number;
  outcome?: string;
  [key: string]: unknown;
}

export interface SidecarFrame {
  frame_number: number;
  timestamp_ms: number;
  timestamp_domain: string;
  host_receive_monotonic_ns: number;
}

export interface DecodedFrame {
  file: string;
  frameNumber: number;
  timestampMs: number;
}

export interface CameraData {
  name: string;
  serial: string;
  frameCount: number;
  frames: SidecarFrame[];
  bagBytes: number;
  prepared: boolean;
  decodedFrames: DecodedFrame[];
}

export interface EpisodeData {
  episode_id: string;
  status: string;
  outcome?: string;
  reason?: string;
  started_monotonic_ns?: number;
  finished_monotonic_ns?: number;
  event_count?: number;
  dropped_events?: number;
}

export interface SessionData {
  sessionId: string;
  episodeDir: string;
  session: Record<string, unknown>;
  episode: EpisodeData;
  telemetry: EventRow[];
  cameras: CameraData[];
}

export interface TimedJoints {
  timeNs: number;
  joints: number[];
  position: Vec3;
  chain: Vec3[];
}

export interface PrepareJob {
  id: string;
  status: "running" | "ready" | "error";
  progress: number;
  message: string;
}
