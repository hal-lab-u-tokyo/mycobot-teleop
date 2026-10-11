import logging
import socket
import time

import serial

HOST = "0.0.0.0"
PORT = 9000
SERIAL_PORT = "/dev/ttyAMA0"
BAUD = 1_000_000

# Atomからの応答を待つ命令
QUERY_COMMANDS = {
    0x20,  # GET_ANGLES
    0x3B,  # GET_ENCODER
    0x50,  # IS_SERVO_ENABLE
    0x51,  # IS_ALL_SERVO_ENABLE
    0x53,  # GET_SERVO_DATA
}

# 応答を待たない命令
WRITE_COMMANDS = {
    0x13,  # RELEASE_ALL_SERVOS
    0x21,  # SEND_ANGLE
    0x22,  # SEND_ANGLES
    0x29,  # STOP
    0x3A,  # SET_ENCODER
    0x56,  # RELEASE_SERVO (no Atom reply)
    0x57,  # FOCUS_SERVO (no Atom reply)
    0x6A,  # SET_COLOR
}

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
)


def pop_frame(buffer):
    while True:
        start = buffer.find(b"\xfe\xfe")

        if start < 0:
            buffer[:] = (
                b"\xfe"
                if buffer.endswith(b"\xfe")
                else b""
            )
            return None

        del buffer[:start]

        if len(buffer) < 3:
            return None

        length = buffer[2]

        if not 2 <= length <= 96:
            del buffer[0]
            continue

        frame_size = length + 3

        if len(buffer) < frame_size:
            return None

        if buffer[frame_size - 1] != 0xFA:
            del buffer[0]
            continue

        frame = bytes(buffer[:frame_size])
        del buffer[:frame_size]
        return frame


def read_reply(ser, opcode, sent_frame, timeout=0.25):
    buffer = bytearray()
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        buffer.extend(chunk)

        while True:
            frame = pop_frame(buffer)

            if frame is None:
                break

            # コマンドのechoや別コマンドの遅延応答を除外
            if frame == sent_frame:
                continue

            if frame[3] == opcode:
                return frame

    return None


def handle_client(conn, ser):
    buffer = bytearray()

    while True:
        data = conn.recv(1024)

        if not data:
            return

        buffer.extend(data)

        while True:
            frame = pop_frame(buffer)

            if frame is None:
                break

            opcode = frame[3]

            if opcode not in QUERY_COMMANDS | WRITE_COMMANDS:
                logging.warning(
                    "Unsupported command: 0x%02X", opcode
                )
                continue

            logging.info("TX Atom: %s", frame.hex()) # Python 3.7 cannot use hex(" ") for bytes, so removed the space separator

            ser.write(frame)
            ser.flush()

            if opcode in QUERY_COMMANDS:
                reply = read_reply(ser, opcode, frame)

                if reply is not None:
                    conn.sendall(reply)
                    logging.info("RX Atom: %s", reply.hex()) # Python 3.7 cannot use hex(" ") for bytes, so removed the space separator
                else:
                    logging.warning(
                        "No response for 0x%02X", opcode
                    )


def main():
    with serial.Serial(
        SERIAL_PORT,
        BAUD,
        timeout=0.01,
        write_timeout=0.2,
    ) as ser:

        with socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM,
        ) as server:

            server.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_REUSEADDR,
                1,
            )

            server.bind((HOST, PORT))
            server.listen(1)

            print(f"Listening on {HOST}:{PORT}")

            while True:
                conn, addr = server.accept()
                logging.info("Connected: %s", addr)

                with conn:
                    try:
                        handle_client(conn, ser)
                    except Exception:
                        logging.exception("Client error")

                logging.info("Disconnected")


if __name__ == "__main__":
    main()
