"""Check that the myCobot TCP server accepts requests and returns joint angles.

By default, performs one request. Pass ``--watch`` to repeatedly query the
server and monitor response latency and successful request rate.
"""

import argparse
import math
import time

from pymycobot import MyCobot280Socket


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.50.2")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--watch", action="store_true",
                        help="Keep checking the server until Ctrl+C.")
    parser.add_argument("--interval", type=float, default=0.1,
                        help="Delay between watch requests in seconds (default: 0.1).")
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error("--interval must be greater than zero")
    return args


def validate_angles(angles):
    return (
        isinstance(angles, list)
        and len(angles) == 6
        and all(isinstance(value, (int, float)) and math.isfinite(value)
                for value in angles)
    )


def main():
    args = parse_args()
    mc = MyCobot280Socket(args.host, args.port)
    successful_requests = 0
    failed_requests = 0
    start = time.perf_counter()

    print("Checking myCobot server at {}:{}".format(args.host, args.port))
    if args.watch:
        print("Continuous check; press Ctrl+C to stop.")

    try:
        while True:
            request_start = time.perf_counter()
            try:
                angles = mc.get_angles()
                latency_ms = (time.perf_counter() - request_start) * 1000
                if not validate_angles(angles):
                    failed_requests += 1
                    print("FAIL: invalid angle response: {!r}".format(angles))
                else:
                    successful_requests += 1
                    joint_text = "  ".join(
                        "J{}:{:7.2f}°".format(i + 1, angle)
                        for i, angle in enumerate(angles)
                    )
                    if args.watch:
                        elapsed = time.perf_counter() - start
                        rate = successful_requests / elapsed if elapsed else 0.0
                        print("{} | response={:.1f} ms | success={:.1f} req/s".format(
                            joint_text, latency_ms, rate))
                    else:
                        print("OK: {} | response={:.1f} ms".format(
                            joint_text, latency_ms))
            except Exception as exc:
                failed_requests += 1
                print("FAIL: server request failed: {}".format(exc))
                if not args.watch:
                    raise SystemExit(1)

            if not args.watch:
                if failed_requests:
                    raise SystemExit(1)
                return
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped. Successful requests: {}; failed requests: {}".format(
            successful_requests, failed_requests))
    finally:
        mc.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
