# test_servoj.py

import re
import time

from dobot_sdk import DobotRobot


ROBOT_IP = "192.168.5.1"

# ServoJ settings
PERIOD = 0.03          # 30 ms = ~33 Hz (DOBOT recommended)
SERVO_T = 0.1
AHEADTIME = 50.0
GAIN = 500.0

# Motion test
J1_DELTA_DEG = 5.0     # J1を+5 deg
MOVE_TIME = 2.0        # 5 degを2秒かけて動かす
HOLD_TIME = 0.5


def parse_values(response: str) -> list[float]:
    """
    Example:
      0,{0.0001,-0.0001,-0.0003,0.0000,-0.0003,0.0000},GetAngle();

    -> [0.0001, -0.0001, ...]
    """
    m = re.search(r"\{([^{}]*)\}", response)
    if not m:
        raise RuntimeError(f"Could not parse response: {response}")

    body = m.group(1).strip()
    if not body:
        return []

    return [float(x.strip()) for x in body.split(",")]


def get_error_code(response: str) -> int:
    """
    DOBOT response:
      0,{...},Command();
      -7,{},Command();
    """
    return int(response.split(",", 1)[0])


def wait_until_enabled(robot, timeout: float = 10.0):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        response = robot.robot_control.RobotMode()
        print("RobotMode:", response)

        mode = parse_values(response)

        if mode and int(mode[0]) == 5:
            return

        if mode and int(mode[0]) == 9:
            raise RuntimeError(
                f"Robot entered ERROR state: "
                f"{robot.robot_control.GetErrorID()}"
            )

        time.sleep(0.1)

    raise TimeoutError("Robot did not reach mode 5")


def servo_ramp(
    robot,
    q_start: list[float],
    q_end: list[float],
    duration: float,
):
    steps = max(1, round(duration / PERIOD))

    start_time = time.monotonic()

    for i in range(1, steps + 1):
        alpha = i / steps

        target = [
            a + alpha * (b - a)
            for a, b in zip(q_start, q_end)
        ]

        response = robot.motion.ServoJ(
            target,
            t=SERVO_T,
            aheadtime=AHEADTIME,
            gain=GAIN,
        )

        error_code = get_error_code(response)

        if error_code != 0:
            raise RuntimeError(
                f"ServoJ failed:\n"
                f"  target = {target}\n"
                f"  response = {response}\n"
                f"  RobotMode = {robot.robot_control.RobotMode()}\n"
                f"  errors = {robot.robot_control.GetErrorID()}"
            )

        # 30 ms周期をなるべく一定にする
        next_time = start_time + i * PERIOD
        sleep_time = next_time - time.monotonic()

        if sleep_time > 0:
            time.sleep(sleep_time)


def main():
    with DobotRobot(ROBOT_IP) as robot:
        try:
            # --------------------------------------------------
            # Initialization
            # --------------------------------------------------

            print("RequestControl:")
            print(robot.robot_control.RequestControl())

            print("ClearError:")
            print(robot.robot_control.ClearError())

            # 以前のpaused queue等が残っていないか確認
            mode_response = robot.robot_control.RobotMode()
            print("Initial RobotMode:", mode_response)

            mode = parse_values(mode_response)

            if mode and int(mode[0]) == 10:
                print("Robot is PAUSED. Sending Stop()...")
                print(robot.robot_control.Stop())
                time.sleep(0.5)

                print(
                    "RobotMode after Stop:",
                    robot.robot_control.RobotMode(),
                )

            print("Errors:")
            print(robot.robot_control.GetErrorID())

            # --------------------------------------------------
            # Enable
            # --------------------------------------------------

            print("EnableRobot:")
            print(robot.robot_control.EnableRobot())

            wait_until_enabled(robot)

            print("Robot enabled.")

            # --------------------------------------------------
            # Read initial joint position
            # --------------------------------------------------

            angle_response = robot.robot_control.GetAngle()
            print("GetAngle:", angle_response)

            q0 = parse_values(angle_response)

            if len(q0) != 6:
                raise RuntimeError(f"Unexpected joint vector: {q0}")

            print()
            print("q0 =", q0)

            # --------------------------------------------------
            # Target: J1 only +5 degrees
            # --------------------------------------------------

            q1 = q0.copy()
            q1[0] += J1_DELTA_DEG

            print("q1 =", q1)
            print()
            print(
                f"Moving J1 by +{J1_DELTA_DEG} deg "
                f"over {MOVE_TIME} s..."
            )

            # q0 -> q1
            servo_ramp(
                robot,
                q_start=q0,
                q_end=q1,
                duration=MOVE_TIME,
            )

            print("Reached target.")
            print(
                "Actual:",
                robot.robot_control.GetAngle(),
            )

            time.sleep(HOLD_TIME)

            # --------------------------------------------------
            # Return
            # --------------------------------------------------

            print()
            print("Returning to q0...")

            servo_ramp(
                robot,
                q_start=q1,
                q_end=q0,
                duration=MOVE_TIME,
            )

            time.sleep(0.2)

            print("Final:")
            print(robot.robot_control.GetAngle())

        except KeyboardInterrupt:
            print("\nInterrupted by user.")

        finally:
            # Servo command queueを止めてからdisable
            print("\nStopping robot...")
            try:
                print(robot.robot_control.Stop())
                time.sleep(0.2)
            except Exception as e:
                print("Stop failed:", e)

            try:
                print("DisableRobot:")
                print(robot.robot_control.DisableRobot())
            except Exception as e:
                print("DisableRobot failed:", e)


if __name__ == "__main__":
    main()