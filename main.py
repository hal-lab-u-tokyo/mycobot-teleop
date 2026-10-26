"""Unified command line entry point for myCobot/Nova experiments."""

import argparse
import json
import math
import re
import uuid
from datetime import datetime
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(
        description="Control and synchronize the myCobot 280 Pi and Dobot Nova 5."
    )
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser(
        "sync",
        help="Move myCobot to the pose mapped from the current Nova pose.",
        description="Synchronize the myCobot pose to the current Nova pose.",
    )
    teleop_parser = commands.add_parser(
        "teleop",
        help="Teleoperate Nova from selected myCobot joints.",
        description="Teleoperate Nova from myCobot (dry run unless --execute is set).",
    )
    teleop_parser.add_argument(
        "--execute", action="store_true", help="Enable Nova and execute teleoperation."
    )
    teleop_parser.add_argument(
        "--joints", nargs="+", type=int, default=[1, 2, 3, 4, 5, 6],
        help="myCobot/Nova joints to map (1 through 6; default: 1 2 3 4 5 6).",
    )

    record_parser = commands.add_parser(
        "record",
        help="Record camera and robot data during executed teleoperation.",
        description=(
            "Record one teleoperation episode. --execute explicitly enables robot "
            "motion, then the existing ARM confirmation is still required. Without "
            "--execute this command follows the read-only dry-run path."
        ),
    )
    record_parser.add_argument(
        "--execute", action="store_true",
        help="Explicitly authorize robot motion for this recording session.",
    )
    record_parser.add_argument("--task", required=True, help="Natural-language task description.")
    record_parser.add_argument(
        "--output", type=Path, default=Path("data/recordings"),
        help="Parent directory for recording sessions (default: data/recordings).",
    )
    record_parser.add_argument(
        "--camera", action="append", required=True, metavar="NAME=SERIAL",
        help="RealSense camera name and serial; repeat for each camera (e.g. front=1234).",
    )
    record_parser.add_argument("--width", type=int, default=640, help="RGB width (default: 640).")
    record_parser.add_argument("--height", type=int, default=480, help="RGB height (default: 480).")
    record_parser.add_argument("--fps", type=int, default=30, help="RGB frame rate (default: 30).")
    record_parser.add_argument(
        "--joints", nargs="+", type=int, default=[1, 2, 3, 4, 5, 6],
        help="myCobot/Nova joints to map (1 through 6; default: 1 2 3 4 5 6).",
    )

    inspect_parser = commands.add_parser(
        "inspect", help="Inspect a raw recording session without modifying it."
    )
    inspect_parser.add_argument("--session", type=Path, required=True)

    quality_parser = commands.add_parser(
        "quality", help="Create a raw-session timing and motion quality report."
    )
    quality_parser.add_argument("--session", type=Path, required=True)
    quality_parser.add_argument("--output", type=Path)

    sync_parser = commands.add_parser(
        "synchronize", help="Create a synchronized intermediate dataset (not LeRobot)."
    )
    sync_parser.add_argument("--session", type=Path, required=True)
    sync_parser.add_argument("--output", type=Path, required=True)
    sync_parser.add_argument("--reference-camera", required=True)
    sync_parser.add_argument("--state-source", choices=("mycobot", "nova"))
    sync_parser.add_argument("--action-source")
    sync_parser.add_argument("--action-delay-ms", type=float, default=0.0)
    sync_parser.add_argument("--state-slop-ms", type=float, default=40.0)
    sync_parser.add_argument("--max-state-bracket-ms", type=float, default=150.0)
    sync_parser.add_argument("--action-slop-ms", type=float, default=40.0)
    sync_parser.add_argument("--camera-slop-ms", type=float, default=20.0)
    sync_parser.add_argument("--state-mode", choices=("interpolate", "nearest"), default="interpolate")

    export_parser = commands.add_parser(
        "export", help="Export a recording to a derived dataset format."
    )
    export_parser.add_argument("--session", type=Path, required=True)
    export_parser.add_argument("--output", type=Path, required=True)
    export_parser.add_argument("--format", choices=("synchronized", "lerobot"), default="synchronized")
    export_parser.add_argument("--reference-camera", required=True)
    export_parser.add_argument("--state-source", choices=("mycobot", "nova"))
    export_parser.add_argument("--action-source")
    export_parser.add_argument("--action-delay-ms", type=float, default=0.0)
    export_parser.add_argument("--state-slop-ms", type=float, default=40.0)
    export_parser.add_argument("--max-state-bracket-ms", type=float, default=150.0)
    export_parser.add_argument("--action-slop-ms", type=float, default=40.0)
    export_parser.add_argument("--camera-slop-ms", type=float, default=20.0)
    export_parser.add_argument("--state-mode", choices=("interpolate", "nearest"), default="interpolate")
    export_parser.add_argument("--fps", type=int, default=30, help="Nominal LeRobot FPS (default: 30).")
    export_parser.add_argument("--curation", type=Path, help="Optional curation decisions JSONL file.")
    export_parser.add_argument("--repo-id", help="Optional local LeRobot dataset identifier.")

    curate_parser = commands.add_parser(
        "curate", help="Append keep/filter/relabel decisions outside raw recordings."
    )
    curate_parser.add_argument("--session", type=Path, required=True)
    curate_parser.add_argument("--decisions", type=Path, required=True,
                               help="JSON file containing a list of episode decisions.")
    curate_parser.add_argument("--output", type=Path)

    args = parser.parse_args()

    if args.command == "sync":
        from sync_initial_pose import main as sync_main

        sync_main()
    elif args.command == "teleop":
        from teleop_mycobot_nova import run_teleop

        run_teleop(execute=args.execute, joints=args.joints)
    elif args.command == "record":
        from teleop_mycobot_nova import run_teleop

        if not args.task.strip():
            parser.error("--task must not be empty")

        cameras = []
        names = set()
        serials = set()
        for value in args.camera:
            if "=" not in value:
                parser.error("--camera must use NAME=SERIAL (got {!r})".format(value))
            name, serial = value.split("=", 1)
            if not re.fullmatch(r"[A-Za-z0-9_-]+", name) or not serial.strip():
                parser.error("Invalid --camera value {!r}; use NAME=SERIAL".format(value))
            if name in names:
                parser.error("Camera names must be unique: {!r}".format(name))
            if serial in serials:
                parser.error("Camera serials must be unique: {!r}".format(serial))
            names.add(name)
            serials.add(serial)
            cameras.append({"name": name, "serial": serial})
        if min(args.width, args.height, args.fps) <= 0:
            parser.error("Camera width, height, and fps must be positive")

        session_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = (
            args.output.expanduser().resolve()
            / "session_{}_{}".format(session_stamp, uuid.uuid4().hex[:8])
        )
        recording = None
        if args.execute:
            recording = {
                "task": args.task,
                "session_dir": session_dir,
                "cameras": cameras,
                "width": args.width,
                "height": args.height,
                "fps": args.fps,
            }
        else:
            print("DRY RUN: no robot commands or camera recordings will be started.")
            print("Add --execute to arm the recording flow; the ARM prompt still applies.")

        run_teleop(
            execute=args.execute,
            joints=args.joints,
            recording=recording,
        )
    elif args.command in {"inspect", "quality", "synchronize", "export", "curate"}:
        try:
            if args.command == "inspect":
                from src.mycobot_leader.datasets import inspect_session

                report = inspect_session(args.session).to_dict()
            elif args.command == "quality":
                from src.mycobot_leader.datasets.quality_report import generate_quality_report

                report = generate_quality_report(args.session, args.output)
            elif args.command in {"synchronize", "export"}:
                from src.mycobot_leader.datasets.synchronize import (
                    SynchronizationConfig,
                    write_synchronized_intermediate,
                )
                from src.mycobot_leader.datasets.lerobot_export import (
                    export_lerobot,
                )

                milliseconds = (
                    args.action_delay_ms,
                    args.state_slop_ms,
                    args.max_state_bracket_ms,
                    args.action_slop_ms,
                    args.camera_slop_ms,
                )
                if not all(math.isfinite(value) for value in milliseconds):
                    parser.error("Synchronization timing values must be finite")
                if not all(value >= 0 for value in milliseconds[1:]):
                    parser.error("Synchronization slop and bracket values must be non-negative")
                config = SynchronizationConfig(
                    reference_camera=args.reference_camera,
                    state_mode=args.state_mode,
                    state_source=args.state_source,
                    action_source=args.action_source,
                    state_slop_ns=int(args.state_slop_ms * 1_000_000),
                    max_state_bracket_ns=int(args.max_state_bracket_ms * 1_000_000),
                    action_slop_ns=int(args.action_slop_ms * 1_000_000),
                    camera_slop_ns=int(args.camera_slop_ms * 1_000_000),
                    action_delay_ns=int(args.action_delay_ms * 1_000_000),
                )
                if args.command == "export" and args.format == "lerobot":
                    report = export_lerobot(
                        args.session,
                        args.output,
                        config,
                        fps=args.fps,
                        curation_path=args.curation,
                        repo_id=args.repo_id,
                    )
                else:
                    report = write_synchronized_intermediate(args.session, args.output, config)
            else:
                from src.mycobot_leader.datasets.curation import write_curation_decisions

                with args.decisions.open("r", encoding="utf-8") as stream:
                    decisions = json.load(stream)
                if isinstance(decisions, dict):
                    decisions = decisions.get("decisions")
                if not isinstance(decisions, list):
                    parser.error("--decisions JSON must be a list or an object with a decisions list")
                output_path = write_curation_decisions(args.session, decisions, args.output)
                report = {"decisions_written": len(decisions), "output": str(output_path)}
            print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        except (OSError, TypeError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
            parser.error(str(exc))


if __name__ == "__main__":
    main()
