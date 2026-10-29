"""Read or cautiously command a DH-Robotics AG-160-95 through a Nova 5.

Examples:
  uv run python test/check_ag160_gripper.py status
  uv run python test/check_ag160_gripper.py initialize --confirm-motion
  uv run python test/check_ag160_gripper.py move 400 --confirm-motion

The status command is read-only. Initialization and movement require an explicit
confirmation flag because they actuate the gripper.
"""

import argparse
import re
import sys

from dobot_sdk import DobotRobot


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.5.1", help="Nova 5 IP")
    parser.add_argument("--port", type=int, default=29999, help="Dashboard TCP port")
    parser.add_argument("--slave-id", type=int, default=1, help="AG Modbus ID")
    parser.add_argument("--baud", type=int, default=115200, help="Tool RS485 baud rate")
    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("status", help="Read AG initialization, grip state and position")
    initialize = subparsers.add_parser("initialize", help="Home the gripper")
    initialize.add_argument("--confirm-motion", action="store_true")
    move = subparsers.add_parser("move", help="Move to a position from 0 to 1000")
    move.add_argument("position", type=int)
    move.add_argument("--confirm-motion", action="store_true")
    args = parser.parse_args()
    if args.action == "move" and not 15 <= args.position <= 800:
        parser.error("position must be between 15 and 800 (validated operating range)")
    if args.action != "status" and not args.confirm_motion:
        parser.error("actuation requires --confirm-motion")
    return args


def check_response(response: str, command: str) -> str:
    match = re.match(r"\s*(-?\d+),\{(.*?)\}", response)
    if not match:
        raise RuntimeError(f"Unexpected response for {command}: {response!r}")
    code = int(match.group(1))
    if code != 0:
        raise RuntimeError(f"Nova rejected {command}: {response}")
    return match.group(2).strip()


def read_register(robot, index: int, address: int) -> int:
    command = f"GetHoldRegs({index},{address},1)"
    payload = check_response(robot.communication.GetHoldRegs(index, address, 1), command)
    try:
        return int(payload.split(",")[0])
    except (ValueError, IndexError) as exc:
        raise RuntimeError(f"Could not parse register {address:#06x}: {payload!r}") from exc


def write_register(robot, index: int, address: int, value: int) -> None:
    command = f"SetHoldRegs({index},{address},1,{{{value}}},U16)"
    response = robot.communication.SetHoldRegs(index, address, 1, [value])
    check_response(response, command)
    print(f"Wrote register {address:#06x} = {value}")


def main():
    args = parse_args()
    robot = None
    connected = False
    modbus_index = None
    try:
        robot = DobotRobot(args.host, dashboard_port=args.port)
        robot.Connect()
        connected = True
        set485 = robot.io.SetTool485(args.baud, parity="N", stopbit=1)
        check_response(set485, f"SetTool485({args.baud},\"N\",1)")

        response = robot.communication.ModbusCreate("127.0.0.1", 60000, args.slave_id, 1)
        payload = check_response(response, "ModbusCreate(127.0.0.1,60000,slave_id,1)")
        match = re.fullmatch(r"\s*(\d+)\s*", payload)
        if not match:
            raise RuntimeError(f"Could not parse Modbus master index: {response!r}")
        modbus_index = int(match.group(1))
        print(f"Connected to AG via Nova Tool RS485 (Modbus index {modbus_index}).")

        if args.action == "status":
            values = {
                "initialization": read_register(robot, modbus_index, 0x0200),
                "grip_state": read_register(robot, modbus_index, 0x0201),
                "position": read_register(robot, modbus_index, 0x0202),
            }
            print(f"AG status: {values}")
            if values["initialization"] != 1:
                print("AG is not initialized; use the initialize action when ready.")
        elif args.action == "initialize":
            write_register(robot, modbus_index, 0x0100, 1)
            print("Initialization command sent. Use 'status' to check the result.")
        else:
            initialized = read_register(robot, modbus_index, 0x0200)
            if initialized != 1:
                raise RuntimeError("AG is not initialized; run initialize first")
            write_register(robot, modbus_index, 0x0103, args.position)
            print("Position command sent. Use 'status' to read the resulting position.")
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if robot is not None and modbus_index is not None:
            try:
                check_response(robot.communication.ModbusClose(modbus_index), "ModbusClose")
            except Exception as exc:
                print(f"Warning: failed to close Modbus connection: {exc}", file=sys.stderr)
        if robot is not None and connected:
            robot.Disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
