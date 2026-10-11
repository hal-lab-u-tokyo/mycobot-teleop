import json
import re
import time
from pathlib import Path

from dobot_sdk import DobotRobot
from pymycobot import MyCobot280Socket
from pymycobot.common import ProtocolCode
from mycobot_control import (
    LED_ERROR,
    LED_INCOMPLETE,
    LED_SUCCESS,
    LED_SYNCING,
    set_servo_enabled,
    set_status_led,
)

NOVA_IP = "192.168.5.1"
MYCOBOT_IP = "192.168.50.2"
MYCOBOT_PORT = 9000

CALIBRATION_FILE = Path(__file__).with_name("joint_calibration.json")

# Joint mapping (要キャリブレーション)
NOVA_ZERO = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
MYCOBOT_ZERO = [0.0, 0.0, 0.0, 0.0, -90.0, 90.0]

SIGN = [1, 1, 1, 1, 1, 1]
SCALE = [1, 1, 1, 1, 1, 1]

# Conservative limits for the initial experiment
JOINT_MIN = [-168, -125, -135, -135, -145, -170]
JOINT_MAX = [168, 125, 135, 135, 150, 170]

# Safety settings
MAX_DELTA_DEG = 180.0
MAX_NOVA_DRIFT_DEG = 0.5
MAX_FOCUS_DRIFT_DEG = 3.0
TARGET_TOLERANCE_DEG = 1.0

SPEED = 10
MOVE_TIMEOUT = 20.0
POLL_PERIOD = 0.1
STABLE_SAMPLES = 5
ENCODER_STABLE_SAMPLES = 15
ANGLE_STABILITY_DEG = 0.03
MAX_FINE_CORRECTIONS = 10
ENCODER_ESCALATION_INCREMENT = 8
MAX_ENCODER_CORRECTION = 64


def parse_nova(response):
    error = int(response.split(",", 1)[0])

    if error != 0:
        raise RuntimeError(f"Nova command failed: {response}")

    match = re.search(r"\{([^{}]*)\}", response)

    if match is None:
        raise RuntimeError(f"Invalid Nova response: {response}")

    return [float(x) for x in match.group(1).split(",")]


def read_nova(robot):
    angles = parse_nova(robot.robot_control.GetAngle())

    if len(angles) != 6:
        raise RuntimeError(f"Invalid Nova angles: {angles}")

    return angles


def read_mycobot(mc):
    angles = mc.get_angles()

    if not isinstance(angles, list) or len(angles) != 6:
        raise RuntimeError(f"Invalid myCobot angles: {angles}")

    return [float(x) for x in angles]


def read_encoder(mc, joint):
    encoder = mc.get_encoder(joint)
    if not isinstance(encoder, (int, float)):
        raise RuntimeError(f"Failed to read J{joint} encoder: {encoder!r}")
    return int(encoder)


def wait_all_stable(mc, robot, nova_q):
    deadline = time.monotonic() + MOVE_TIMEOUT
    previous = None
    stable_count = 0
    latest = None
    while time.monotonic() < deadline:
        latest = read_mycobot(mc)
        check_nova_state(robot)
        if max_error(read_nova(robot), nova_q) > MAX_NOVA_DRIFT_DEG:
            raise RuntimeError("Nova moved during synchronization.")
        if (previous is not None and all(
                abs(angular_difference_deg(a, b)) <= ANGLE_STABILITY_DEG
                for a, b in zip(latest, previous))):
            stable_count += 1
            if stable_count >= STABLE_SAMPLES:
                return latest
        else:
            stable_count = 0
        previous = latest
        time.sleep(POLL_PERIOD)
    raise TimeoutError("myCobot did not settle within {:.1f}s".format(MOVE_TIMEOUT))


def wait_joint_stable(mc, robot, nova_q, joint):
    deadline = time.monotonic() + MOVE_TIMEOUT
    previous = None
    stable_count = 0
    latest = None
    while time.monotonic() < deadline:
        angles = read_mycobot(mc)
        encoder = read_encoder(mc, joint)
        latest = (angles[joint - 1], encoder)
        check_nova_state(robot)
        if max_error(read_nova(robot), nova_q) > MAX_NOVA_DRIFT_DEG:
            raise RuntimeError("Nova moved during synchronization.")
        if (previous is not None
                and abs(angular_difference_deg(latest[0], previous[0])) <= ANGLE_STABILITY_DEG
                and latest[1] == previous[1]):
            stable_count += 1
            if stable_count >= ENCODER_STABLE_SAMPLES:
                return latest
        else:
            stable_count = 0
        previous = latest
        time.sleep(POLL_PERIOD)
    raise TimeoutError("myCobot J{} did not settle within {:.1f}s".format(
        joint, MOVE_TIMEOUT))


def angular_difference_deg(a, b):
    """a - b の最短角度差を [-180, 180) で返す"""
    return (a - b + 180.0) % 360.0 - 180.0


def max_error(a, b):
    errors = [
        abs(angular_difference_deg(x, y)) if i == 5
        else abs(x - y)
        for i, (x, y) in enumerate(zip(a, b))
    ]
    return max(errors)


def check_nova_state(robot):
    mode = int(parse_nova(robot.robot_control.RobotMode())[0])

    if mode not in (4, 5):
        raise RuntimeError(
            f"Nova must be disabled or idle, got mode={mode}"
        )


def calculate_target(nova_current):
    return [
        m_zero + sign * scale * (n - n_zero)
        for n, n_zero, m_zero, sign, scale in zip(
            nova_current,
            NOVA_ZERO,
            MYCOBOT_ZERO,
            SIGN,
            SCALE,
        )
    ]

def validate_target(current, target):
    for i, (q, lo, hi) in enumerate(
        zip(target, JOINT_MIN, JOINT_MAX)
    ):
        if not lo <= q <= hi:
            raise RuntimeError(
                f"J{i+1} target {q:.2f} outside [{lo}, {hi}]"
            )

    for i, (q0, q1) in enumerate(zip(current, target)):
        delta = abs(q1 - q0)

        if delta > MAX_DELTA_DEG:
            raise RuntimeError(
                f"J{i+1}: requested motion {delta:.2f} deg "
                f"exceeds {MAX_DELTA_DEG} deg"
            )


def sync(robot, mc):
    check_nova_state(robot)

    nova_q = read_nova(robot)
    mycobot_q = read_mycobot(mc)

    desired = calculate_target(nova_q)

    requested_j1 = desired[0]
    desired[0] = max(JOINT_MIN[0], min(JOINT_MAX[0], requested_j1))
    if desired[0] != requested_j1:
        print(
            "WARNING: Nova J1 target {:+.2f} deg is outside myCobot's "
            "[{:+.0f}, {:+.0f}] deg range; commanding limit {:+.2f} deg."
            .format(requested_j1, JOINT_MIN[0], JOINT_MAX[0], desired[0])
        )

    validate_target(mycobot_q, desired)

    print("\nJoint | Current | Target | Delta")
    print("---------------------------------")

    for i, (current, target) in enumerate(zip(mycobot_q, desired)):
        print(
            f"J{i+1}    | "
            f"{current:7.2f} | "
            f"{target:7.2f} | "
            f"{target-current:+7.2f}"
        )

    # Keep the pose/state checks even though the explicit MOVE prompt is gone.
    check_nova_state(robot)
    if max_error(read_nova(robot), nova_q) > MAX_NOVA_DRIFT_DEG:
        raise RuntimeError("Nova pose changed. Aborting.")
    if max_error(read_mycobot(mc), mycobot_q) > 1.0:
        raise RuntimeError("myCobot pose changed. Aborting.")

    try:
        set_status_led(mc, LED_SYNCING)

        # Enable myCobot servo position control
        print("Enabling myCobot servos...")
        for joint in range(1, 7):
            set_servo_enabled(mc, joint, enabled=True)

        time.sleep(0.4)

        actual = read_mycobot(mc)

        if max_error(actual, mycobot_q) > MAX_FOCUS_DRIFT_DEG:
            raise RuntimeError(
                "myCobot moved unexpectedly during servo enabling."
            )

        validate_target(actual, desired)
        coarse_start_angles = actual[:]
        initial_encoders = [read_encoder(mc, joint) for joint in range(1, 7)]

        mc.send_angles(
            [round(q, 2) for q in desired],
            SPEED,
            _async=True,
        )

        print("send_angles command sent")
        actual = wait_all_stable(mc, robot, nova_q)
        print("\nAfter angle move:", actual)

        coarse_encoders = [read_encoder(mc, joint) for joint in range(1, 7)]
        local_slopes = []
        for i in range(6):
            angle_delta = (
                angular_difference_deg(actual[i], coarse_start_angles[i])
                if i == 5 else actual[i] - coarse_start_angles[i]
            )
            encoder_delta = coarse_encoders[i] - initial_encoders[i]
            slope = encoder_delta / angle_delta if abs(angle_delta) >= 0.5 else 0.0
            local_slopes.append(slope)

        for i in range(6):
            angle_error = angular_difference_deg(desired[i], actual[i])
            slope = local_slopes[i]
            if abs(angle_error) <= TARGET_TOLERANCE_DEG:
                continue
            if not slope:
                print(f"J{i+1}: cannot estimate encoder/degree ratio; leaving residual")
                continue

            print(f"J{i+1}: residual={angle_error:+.2f} deg, local slope={slope:+.3f} counts/deg")
            stagnant_attempts = 0
            angle, encoder = actual[i], coarse_encoders[i]
            for correction_number in range(1, MAX_FINE_CORRECTIONS + 1):
                angle_error = angular_difference_deg(desired[i], angle)
                if abs(angle_error) <= TARGET_TOLERANCE_DEG:
                    break
                nominal_delta = round(slope * angle_error)
                if nominal_delta == 0:
                    print(f"  J{i+1}: residual maps to less than one encoder count; stopping")
                    break

                min_step = min(
                    ENCODER_ESCALATION_INCREMENT * (stagnant_attempts + 1),
                    MAX_ENCODER_CORRECTION,
                )
                encoder_delta = int(
                    (1 if nominal_delta > 0 else -1)
                    * min(max(abs(nominal_delta), min_step), MAX_ENCODER_CORRECTION)
                )
                encoder_target = encoder + encoder_delta
                if not 0 <= encoder_target <= 4096:
                    print(f"  J{i+1}: encoder target {encoder_target} outside 0..4096; stopping")
                    break

                print(f"  correction {correction_number}: encoder {encoder} -> {encoder_target} ({encoder_delta:+d})")
                previous_angle, previous_encoder = angle, encoder
                mc._mesg(ProtocolCode.SET_ENCODER, i + 1, [encoder_target], SPEED,
                         _async=True)
                angle, encoder = wait_joint_stable(mc, robot, nova_q, i + 1)
                actual[i] = angle
                if angle == previous_angle and encoder == previous_encoder:
                    stagnant_attempts += 1
                    print(f"    no measured movement; next minimum correction is {min(ENCODER_ESCALATION_INCREMENT * (stagnant_attempts + 1), MAX_ENCODER_CORRECTION)} counts")
                else:
                    stagnant_attempts = 0
                print(f"    J{i+1}: angle={angle:.2f} deg, encoder={encoder}, "
                      f"error={angular_difference_deg(desired[i], angle):+.2f} deg")

        final_angles = read_mycobot(mc)
        residuals = [angular_difference_deg(target, angle)
                     for target, angle in zip(desired, final_angles)]
        unresolved = [i + 1 for i, error in enumerate(residuals)
                      if abs(error) > TARGET_TOLERANCE_DEG]
        print("myCobot actual:", final_angles)
        if unresolved:
            print("Synchronization finished with residuals above {:.2f} deg "
                  "on joints: {}".format(TARGET_TOLERANCE_DEG, unresolved))
            set_status_led(mc, LED_INCOMPLETE)
        else:
            print("Synchronization complete within {:.2f} deg.".format(
                TARGET_TOLERANCE_DEG))
            set_status_led(mc, LED_SUCCESS)

    except Exception:
        set_status_led(mc, LED_ERROR)
        raise

    finally:
        # Stop pending movement, keep servo torque enabled.
        try:
            mc._mesg(ProtocolCode.STOP, _async=True)
            print("\nmyCobot stop command sent")
        except Exception as e:
            print("WARNING: stop failed:", e)


def main():
    mc = MyCobot280Socket(MYCOBOT_IP, MYCOBOT_PORT)

    with DobotRobot(NOVA_IP) as robot:
        sync(robot, mc)

if __name__ == "__main__":
    main()
