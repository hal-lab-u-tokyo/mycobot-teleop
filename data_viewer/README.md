# MyCobot Data Viewer

Local browser UI for reviewing a raw teleoperation session: Nova 5 measured and ServoJ-command EEF paths, joint-angle traces, and synchronized RealSense RGB frames.

## Run

From the repository root:

```sh
cd data_viewer
npm install
npm run model:download
npm run dev
```

Open the local URL printed by Vite (normally `http://127.0.0.1:4173`). The local API searches for `data/recordings` relative to this folder and the current working directory. If recordings live elsewhere, set `MYCOBOT_RECORDINGS_DIR` to the directory containing `session_*` folders. Select a session and episode in the header. The first view of an episode decodes each `.bag` with `rs-convert`, then stores PNG frames in the OS temporary directory under `mycobot-data-viewer-cache`. Source recordings are read-only. The cache can be removed when no viewer process is using it.

The viewer looks for `rs-convert` in this order:

1. `REALSENSE_RS_CONVERT`
2. `/private/tmp/librealsense-2.57.6-install/bin/rs-convert`
3. `/opt/homebrew/bin/rs-convert`
4. `/usr/local/bin/rs-convert`

If the selected binary needs a custom SDK library directory, set `REALSENSE_LIB_DIR` before starting Vite. For the temporary 2.57.6 build used for this session:

```sh
MYCOBOT_RECORDINGS_DIR=/path/to/data/recordings \
REALSENSE_RS_CONVERT=/private/tmp/librealsense-2.57.6-install/bin/rs-convert \
REALSENSE_LIB_DIR=/private/tmp/librealsense-2.57.6-install/lib \
npm run dev
```

## Time alignment

The common time axis is host monotonic nanoseconds:

- measured Nova joints: `robot_state.host_response_ns`
- ServoJ targets: `action.host_send_start_ns`
- camera frames: `*.timestamps.jsonl` `host_receive_monotonic_ns`

The viewer interpolates joint angles at the playback cursor and computes FK locally. It shows J1–J3 and J4–J6 together in separate angle plots. The `.bag` timestamps are retained as metadata but are not mixed with the host clock.

EEF is displayed at the Nova 5 `Link6` origin (flange frame). The FK joint origins, axes, and limits are based on Dobot's [Nova 5 Xacro](https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis/blob/feature/v4-optimization/dobot_gazebo/urdf/nova5_robot.xacro). `npm run model:download` fetches its seven STL meshes into `public/models/nova5/` and saves the upstream MIT license beside them. When those files are present the viewer renders the STL model; otherwise it falls back to a width-aware schematic. The measured controller TCP/tool offset is not recorded in this session, so the display does not claim calibrated TCP coordinates.

## Controls

- Drag the 3D canvas to orbit; scroll to zoom.
- Use `3D`, `上面`, and `側面` to switch the camera view.
- Play, pause, step, or scrub the shared time cursor to move both camera frames and the robot trace together.
- Compare the solid measured lines with dashed ServoJ targets; J1–J3 and J4–J6 are overlaid in separate panes.
- Angular acceleration is estimated from the joint-angle samples with a five-sample local quadratic fit and shown in °/s².

The viewer only reads recordings and writes decoded PNG cache files under the system temporary directory.
