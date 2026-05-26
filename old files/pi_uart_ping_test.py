import json
import time

import serial


PORT = "/dev/serial0"
BAUD = 115200


def read_json_line(ser: serial.Serial, timeout: float) -> dict | None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline().decode("utf-8", errors="replace").strip()
        if not raw:
            continue
        print(f"rx: {raw}")
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            continue
    return None


def main() -> None:
    with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
        print(f"opened {PORT} @ {BAUD}")

        ready = read_json_line(ser, 2.0)
        print(f"ready: {ready}")

        ser.write(b"PING\n")
        ser.flush()
        pong = read_json_line(ser, 2.0)
        if not pong or pong.get("type") != "pong":
            raise RuntimeError("Did not receive pong from ESP")

        ser.write(b"HELLO\n")
        ser.flush()
        hello = read_json_line(ser, 2.0)
        if not hello or not hello.get("ok"):
            raise RuntimeError("Did not receive hello confirmation from ESP")

        print("UART ping test passed")


if __name__ == "__main__":
    main()