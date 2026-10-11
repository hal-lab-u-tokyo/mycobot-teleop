"""Shared myCobot servo and status LED helpers."""

import time

from pymycobot.common import ProtocolCode


SERVO_COMMAND_ATTEMPTS = 3
SERVO_RETRY_DELAY_S = 0.15
SERVO_STATE_SETTLE_DELAY_S = 0.25
RELEASE_WITHOUT_DAMPING_MODE = 1
SERVO_TORQUE_ENABLE_ADDRESS = 40

LED_SYNCING = (0, 0, 255)
LED_SUCCESS = (0, 255, 0)
LED_INCOMPLETE = (255, 128, 0)
LED_ERROR = (255, 0, 0)
LED_WAITING = (255, 128, 0)
LED_TELEOP = (0, 128, 255)
LED_STOPPING = (255, 64, 0)


def set_servo_enabled(mc, joint, enabled):
    """Set and verify a joint's torque state through its servo register."""
    opcode = ProtocolCode.FOCUS_SERVO if enabled else ProtocolCode.RELEASE_SERVO
    state_name = "focus" if enabled else "release"
    expected_state = 1 if enabled else 0

    for attempt in range(1, SERVO_COMMAND_ATTEMPTS + 1):
        try:
            if enabled:
                mc._mesg(opcode, joint, _async=True)
            else:
                # pymycobot documents mode=1 as canceling release damping.
                mc._mesg(
                    opcode, joint, RELEASE_WITHOUT_DAMPING_MODE,
                    _async=True,
                )
            # Focus/release has no Atom acknowledgment. Address 40 is the
            # Feetech SMS/STS torque-enable register; allow the command to
            # reach the servo before reading it back.
            time.sleep(SERVO_STATE_SETTLE_DELAY_S)
            torque_enabled = mc.get_servo_data(
                joint, SERVO_TORQUE_ENABLE_ADDRESS
            )
            if torque_enabled == expected_state:
                print(
                    "J{} {} confirmed by servo address {} (torque_enabled={})".format(
                        joint, state_name, SERVO_TORQUE_ENABLE_ADDRESS,
                        torque_enabled,
                    )
                )
                return
            detail = "servo address {} returned {!r}, expected {}".format(
                SERVO_TORQUE_ENABLE_ADDRESS, torque_enabled, expected_state
            )
        except Exception as exc:
            detail = str(exc)

        print("J{} {} not confirmed (attempt {}/{}; {})".format(
            joint, state_name, attempt, SERVO_COMMAND_ATTEMPTS, detail
        ))
        if attempt < SERVO_COMMAND_ATTEMPTS:
            time.sleep(SERVO_RETRY_DELAY_S)

    raise RuntimeError(
        "myCobot J{} {} failed verification through servo address {} after {} attempts".format(
            joint, state_name, SERVO_TORQUE_ENABLE_ADDRESS,
            SERVO_COMMAND_ATTEMPTS,
        )
    )


def release_all_servos(mc):
    """Send release-all; verify joints individually with address 40 if needed."""
    for attempt in range(1, SERVO_COMMAND_ATTEMPTS + 1):
        try:
            mc._mesg(ProtocolCode.RELEASE_ALL_SERVOS, _async=True)
            print("Release-all command sent; execution state cannot be queried")
            return
        except Exception as exc:
            detail = str(exc)

        print("Release-all not confirmed (attempt {}/{}; {})".format(
            attempt, SERVO_COMMAND_ATTEMPTS, detail
        ))
        if attempt < SERVO_COMMAND_ATTEMPTS:
            time.sleep(SERVO_RETRY_DELAY_S)

    raise RuntimeError("myCobot release-all command could not be sent after {} attempts".format(
        SERVO_COMMAND_ATTEMPTS
    ))


def set_status_led(mc, rgb):
    """Set the Atom RGB LED; status indication must not interrupt motion."""
    try:
        mc._mesg(ProtocolCode.SET_COLOR, *rgb, _async=True)
    except Exception as exc:
        print("WARNING: Could not set myCobot status LED:", exc)
