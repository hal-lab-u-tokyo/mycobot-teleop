"""Unified command line entry point for myCobot/Nova experiments."""

import argparse


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

    args = parser.parse_args()

    if args.command == "sync":
        from sync_initial_pose import main as sync_main

        sync_main()
    elif args.command == "teleop":
        from teleop_mycobot_nova import run_teleop

        run_teleop(execute=args.execute, joints=args.joints)


if __name__ == "__main__":
    main()
