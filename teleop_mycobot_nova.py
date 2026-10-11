"""myCobot 280 Pi -> Dobot Nova 5 joint-space teleoperation (J1-J6).

Nova initialization matches the known-working test_servoj.py sequence.

Mac requirements: uv add pynput
Raspberry Pi: use the fixed frame-aware mycobot TCP server on port 9000.

DRY RUN (no commands): uv run python main.py teleop
MOTION: uv run python main.py teleop --execute

This is an experiment, NOT collision avoidance or a certified safety function.
"""

import argparse
import math
import re
import threading
import time

from dobot_sdk import DobotRobot
from pymycobot import MyCobot280Socket
from pymycobot.common import ProtocolCode

from mycobot_control import (
    LED_ERROR,
    LED_SUCCESS,
    LED_STOPPING,
    LED_TELEOP,
    LED_WAITING,
    set_servo_enabled,
    set_status_led,
)


NOVA_IP = "192.168.5.1"
MYCOBOT_IP = "192.168.50.2"
MYCOBOT_PORT = 9000

# Map myCobot J1-J6 to Nova J1-J6.
DEFAULT_JOINTS = (1, 2, 3, 4, 5, 6)  # 1-based indices

# Keep these zero points and signs aligned with sync_initial_pose.py.
NOVA_ZERO = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
MYCOBOT_ZERO = [0.0, 0.0, 0.0, 0.0, -90.0, 90.0]
# Signs are provisional; verify each axis at small amplitude.
SIGNS = {1: +1.0, 2: +1.0, 3: +1.0, 4: +1.0, 5: +1.0, 6: +1.0}

PERIOD_S = 0.03  # nominal ~33 Hz as in working test_servoj.py; not a hard real-time guarantee
SERVO_T = 0.10
AHEADTIME = 50.0
GAIN = 500.0

MAX_FOLLOWER_SPEED_DEG_S = 6.0
MAX_LEADER_STEP_DEG = 10.0  # detect implausible jumps per sample
MAX_PASSIVE_LEADER_DRIFT_DEG = 3.0
MAX_FOLLOWER_UNSELECTED_DRIFT_DEG = 2.0
MAX_LEADER_READ_TIME_S = 0.35
MYCOBOT_HOLD_SPEED = 10

# Match the known-working test_servoj.py: DobotRobot uses its default
# 10-second receive timeout. Do not shorten it for EnableRobot.
DOBOT_RECEIVE_TIMEOUT_S = 10.0
ENABLE_TIMEOUT_S = 10.0


def safe_shutdown_nova(robot, motion_attempted):
    """Best effort shutdown, including a fresh connection after TCP receive timeout.

    This is NOT a guaranteed safety stop. Keep the hardware E-stop accessible.
    """
    def commands(control):
        if motion_attempted:
            try:
                result = control.Stop()
                print("Nova Stop:", result)
                require_ok(result, "Nova Stop")
            except Exception as exc:
                print("WARNING: Nova Stop request failed:", exc)
        # Always attempt DisableRobot, even if Stop failed.
        result = control.DisableRobot()
        require_ok(result, "Nova DisableRobot")
        return result

    if robot._dashboard_conn.is_connected:
        try:
            print("Nova DisableRobot:", commands(robot.robot_control))
            return
        except Exception as exc:
            print("WARNING: primary dashboard shutdown failed:", exc)

    print("Reconnecting to Nova for shutdown attempt...")
    try:
        with DobotRobot(NOVA_IP, receive_timeout=DOBOT_RECEIVE_TIMEOUT_S) as recovery:
            print("Nova DisableRobot (new connection):",
                  commands(recovery.robot_control))
    except Exception as exc:
        print("DANGER: Could not confirm Nova shutdown:", exc)
        print("Check robot state at the controller. Use physical E-stop if moving.")


def parse_reply(reply, count):
    if not isinstance(reply, str):
        raise RuntimeError("Invalid Dobot reply: {!r}".format(reply))
    try:
        status = int(reply.split(",", 1)[0])
    except (TypeError, ValueError) as exc:
        raise RuntimeError("Unparseable Dobot reply: {!r}".format(reply)) from exc
    if status != 0:
        raise RuntimeError("Dobot error: {}".format(reply))
    match = re.search(r"\{([^{}]*)\}", reply)
    if match is None:
        raise RuntimeError("Missing Dobot payload: {!r}".format(reply))
    values = [float(s) for s in match.group(1).split(",") if s.strip()]
    if len(values) != count or not all(math.isfinite(v) for v in values):
        raise RuntimeError("Invalid Dobot payload: {!r}".format(reply))
    return values


def require_ok(reply, name):
    if not isinstance(reply, str):
        raise RuntimeError("{} invalid response: {!r}".format(name, reply))
    try:
        status = int(reply.split(",", 1)[0])
    except (TypeError, ValueError) as exc:
        raise RuntimeError("{} invalid response: {!r}".format(name, reply)) from exc
    if status != 0:
        raise RuntimeError("{} failed: {}".format(name, reply))


def read_nova(robot):
    return parse_reply(robot.robot_control.GetAngle(), 6)


def wait_until_enabled(robot, timeout=ENABLE_TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        response = robot.robot_control.RobotMode()
        mode = int(parse_reply(response, 1)[0])
        print("RobotMode during enable:", response)
        if mode == 5:
            return
        if mode == 9:
            raise RuntimeError(
                "Nova entered ERROR mode: {}".format(
                    robot.robot_control.GetErrorID()
                )
            )
        time.sleep(0.1)
    raise TimeoutError("Nova did not enter RobotMode=5")


def robot_mode(robot):
    return int(parse_reply(robot.robot_control.RobotMode(), 1)[0])


def read_mycobot(mc):
    start = time.monotonic()
    q = mc.get_angles()
    elapsed = time.monotonic() - start
    if not isinstance(q, list) or len(q) != 6:
        raise RuntimeError("myCobot read failed: {!r}".format(q))
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in q):
        raise RuntimeError("Invalid myCobot joint angles: {!r}".format(q))
    if elapsed > MAX_LEADER_READ_TIME_S:
        raise RuntimeError("Slow myCobot read: {:.3f}s".format(elapsed))
    return [float(v) for v in q]


def hold_mycobot_joints(mc, joints):
    """Re-focus released joints at their current measured angles."""
    if not joints:
        return

    angles = read_mycobot(mc)
    targets = {j: round(angles[j - 1], 2) for j in joints}
    print("Holding myCobot joints at current angles:", targets)

    # Refresh the stored target while torque is released, then re-enable torque.
    for joint, angle in targets.items():
        mc._mesg(
            ProtocolCode.SEND_ANGLE, joint, [int(round(angle * 100))], MYCOBOT_HOLD_SPEED,
            _async=True,
        )
    time.sleep(0.1)
    focus_errors = []
    for joint in joints:
        try:
            set_servo_enabled(mc, joint, enabled=True)
        except Exception as exc:
            focus_errors.append("J{}: {}".format(joint, exc))
    time.sleep(0.1)

    # Reassert the captured target after focus in case the controller retained
    # an older target internally while the servo was released.
    for joint, angle in targets.items():
        mc._mesg(
            ProtocolCode.SEND_ANGLE, joint, [int(round(angle * 100))], MYCOBOT_HOLD_SPEED,
            _async=True,
        )

    if focus_errors:
        raise RuntimeError("Could not re-focus all released joints: {}".format(
            "; ".join(focus_errors)
        ))


def angle_diff(a, b):
    """Shortest difference a-b, appropriate for periodic angle observations."""
    return (a - b + 180.0) % 360.0 - 180.0


class Deadman:
    """Space toggles one motion session; another press or Escape stops it."""

    def __init__(self):
        self.started = threading.Event()
        self.stop_requested = threading.Event()
        self.space_down = False

    def on_press(self, key):
        from pynput import keyboard
        if key == keyboard.Key.esc:
            self.stop_requested.set()
            return False
        if key == keyboard.Key.space and not self.space_down:
            self.space_down = True
            if self.started.is_set():
                self.stop_requested.set()
            else:
                self.started.set()

    def on_release(self, key):
        from pynput import keyboard
        if key == keyboard.Key.space:
            self.space_down = False


def wait_for_start(deadman, listener):
    print("Press SPACE once to start; press SPACE again or ESC to stop.")
    while not deadman.started.is_set():
        if deadman.stop_requested.is_set():
            raise RuntimeError("Stopped before starting")
        if not listener.is_alive():
            raise RuntimeError("Keyboard listener unavailable; check macOS permissions")
        time.sleep(0.02)


def run_teleop(execute=False, joints=None):
    joints = tuple(DEFAULT_JOINTS if joints is None else joints)
    if not joints or len(set(joints)) != len(joints) or any(j not in SIGNS for j in joints):
        raise ValueError("Supported joints are 1 through 6 (e.g. --joints 1 2 3 4 5 6)")
    idxs = {j - 1 for j in joints}

    mc = MyCobot280Socket(MYCOBOT_IP, MYCOBOT_PORT)
    # A shorter receive timeout avoids some multi-second stalls; it is not a hard safety bound.
    with DobotRobot(NOVA_IP, receive_timeout=DOBOT_RECEIVE_TIMEOUT_S) as robot:
        leader_q = read_mycobot(mc)
        follower_q = read_nova(robot)
        mode = robot_mode(robot)
        print("myCobot initial:", [round(x, 2) for x in leader_q])
        print("Nova initial:   ", [round(x, 2) for x in follower_q])
        print("Selected joints:", joints, "Nova mode:", mode)
        print("Mapping: Nova_abs = Nova_zero + sign * (myCobot_abs - myCobot_zero)")
        print("Nova zero:", {j: NOVA_ZERO[j - 1] for j in joints})
        print("myCobot zero:", {j: MYCOBOT_ZERO[j - 1] for j in joints})
        print("Signs:", {j: SIGNS[j] for j in joints})

        if not execute:
            print("DRY RUN: no servo releases, robot enabling or motion commands sent.")
            return

        if mode != 4:
            raise RuntimeError(
                "Nova must start disabled (RobotMode=4); got {}. "
                "Inspect the Nova controller before retrying.".format(mode)
            )

        # The TCP test server permits this query (0x51). Ensures J2/J3 and others
        # are holding BEFORE releasing just the selected leader servos.
        servo_state = mc.is_all_servo_enable()
        if servo_state != 1:
            raise RuntimeError(
                "All myCobot servos must be safely holding before starting "
                "(is_all_servo_enable()={!r}). Do not auto-FOCUS J6.".format(servo_state)
            )

        print("WARNING: Selected myCobot joints will be released without damping; other joints remain powered.")
        print("WARNING: No geometry/collision checking. Clear Nova's entire swept volume.")
        print("WARNING: Keep the physical Nova E-stop within immediate reach; SPACE is not safety-rated.")
        if input("Type ARM to prepare this one-shot session: ") != "ARM":
            print("Cancelled.")
            return

        enable_attempted = False
        motion_attempted = False
        teleop_completed = False
        released_joints = []
        listener = None
        try:
            set_status_led(mc, LED_WAITING)

            # Known-working initialization from test_servoj.py.
            # Keep the follower stationary until the user presses SPACE.
            resp = robot.robot_control.RequestControl()
            print("RequestControl:", resp)
            require_ok(resp, "RequestControl")

            resp = robot.robot_control.ClearError()
            print("ClearError:", resp)
            require_ok(resp, "ClearError")

            mode_now = robot_mode(robot)
            print("Initial RobotMode after RequestControl:", mode_now)
            if mode_now == 10:
                print("Nova is PAUSED: Stop() to clear previous queue")
                require_ok(robot.robot_control.Stop(), "Stop paused queue")
                time.sleep(0.5)
                print("RobotMode after Stop:", robot.robot_control.RobotMode())

            print("GetErrorID:", robot.robot_control.GetErrorID())
            mode_now = robot_mode(robot)
            if mode_now != 4:
                raise RuntimeError(
                    "Nova must be disabled (mode 4) before enable; got {}".format(mode_now)
                )

            enable_attempted = True  # timeout does NOT prove enable failed
            resp = robot.robot_control.EnableRobot()
            print("EnableRobot:", resp)
            require_ok(resp, "EnableRobot")
            wait_until_enabled(robot)
            print("Nova enabled (mode=5); no ServoJ command has been sent yet")

            # mode=1 disables damping and fully releases each selected servo.
            for j in joints:
                if j not in released_joints:
                    released_joints.append(j)
                set_servo_enabled(mc, j, enabled=False)
            time.sleep(0.3)
            read_mycobot(mc)  # must still have valid communication

            from pynput import keyboard
            deadman = Deadman()
            listener = keyboard.Listener(on_press=deadman.on_press, on_release=deadman.on_release)
            listener.start()
            wait_for_start(deadman, listener)
            set_status_led(mc, LED_TELEOP)

            # Anchor both robots at the instant teleoperation starts:
            # no jump to absolute zero, even if their initial poses differ.
            m0 = read_mycobot(mc)
            n0 = read_nova(robot)
            current_setpoint = list(n0)
            previous_leader = list(m0)
            previous_update = time.monotonic()
            last_status_check = previous_update
            last_log = previous_update
            cycle = 0

            while not deadman.stop_requested.is_set():
                if not listener.is_alive() and not deadman.stop_requested.is_set():
                    raise RuntimeError("Keyboard listener input lost")
                now = time.monotonic()
                leader = read_mycobot(mc)
                desired = list(n0)
                for j in joints:
                    i = j - 1
                    step = angle_diff(leader[i], previous_leader[i])
                    if abs(step) > MAX_LEADER_STEP_DEG:
                        raise RuntimeError("Large leader step on J{}: {:.2f}deg".format(j, step))
                    desired[i] = (
                        NOVA_ZERO[i]
                        + SIGNS[j] * angle_diff(leader[i], MYCOBOT_ZERO[i])
                    )

                # Fail closed if nominally held leader joints drift significantly.
                for i in range(6):
                    if i not in idxs and abs(angle_diff(leader[i], m0[i])) > MAX_PASSIVE_LEADER_DRIFT_DEG:
                        raise RuntimeError("Unselected myCobot J{} moved unexpectedly".format(i + 1))

                dt = min(0.20, max(0.0, time.monotonic() - previous_update))
                max_step = MAX_FOLLOWER_SPEED_DEG_S * dt
                for j in joints:
                    i = j - 1
                    step = max(-max_step, min(max_step, desired[i] - current_setpoint[i]))
                    current_setpoint[i] += step
                previous_update = time.monotonic()
                previous_leader = leader

                motion_attempted = True  # a timeout does not prove ServoJ was rejected
                require_ok(
                    robot.motion.ServoJ(current_setpoint, t=SERVO_T,
                                        aheadtime=AHEADTIME, gain=GAIN),
                    "ServoJ",
                )
                cycle += 1

                if time.monotonic() - last_status_check >= 0.25:
                    actual = read_nova(robot)
                    for i in range(6):
                        delta = abs(actual[i] - n0[i])
                        if i not in idxs and delta > MAX_FOLLOWER_UNSELECTED_DRIFT_DEG:
                            raise RuntimeError("Unselected Nova J{} moved".format(i + 1))
                    current_mode = robot_mode(robot)
                    if current_mode not in (5, 7):  # enabled idle or running
                        raise RuntimeError("Unexpected Nova mode: {}".format(current_mode))
                    last_status_check = time.monotonic()

                if time.monotonic() - last_log >= 0.5:
                    parts = ["J{} leader={:+.2f} nova_cmd={:+.2f}".format(
                        j, leader[j - 1], current_setpoint[j - 1]) for j in joints]
                    print(" | ".join(parts))
                    last_log = time.monotonic()

                loop_elapsed = time.monotonic() - now
                if loop_elapsed > 0.50:
                    raise RuntimeError(
                        "Teleop loop stalled for {:.3f}s".format(loop_elapsed)
                    )
                time.sleep(max(0.0, PERIOD_S - loop_elapsed))

            print("Stop requested or session completed: stopping Nova")
            teleop_completed = True
        finally:
            set_status_led(mc, LED_STOPPING)
            if listener is not None:
                try:
                    listener.stop()
                except Exception as exc:
                    print("Keyboard listener cleanup warning:", exc)

            if enable_attempted:
                safe_shutdown_nova(robot, motion_attempted)

            if released_joints:
                try:
                    hold_mycobot_joints(mc, released_joints)
                    print("myCobot stop-pose hold/focus commands sent for joints {}.".format(
                        released_joints
                    ))
                except Exception as exc:
                    print("WARNING: Could not hold myCobot at its stop pose:", exc)
                    print("myCobot joints {} may remain released.".format(released_joints))
                    teleop_completed = False

            set_status_led(mc, LED_SUCCESS if teleop_completed else LED_ERROR)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="Enable Nova and execute teleop")
    parser.add_argument("--joints", nargs="+", type=int, default=list(DEFAULT_JOINTS))
    args = parser.parse_args()
    run_teleop(execute=args.execute, joints=args.joints)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted")
