import json
import os
import queue
import threading
import time
from dataclasses import dataclass

import serial


# This Pi 5 Ubuntu setup exposes the header UART on /dev/ttyAMA0.
# Override with PI_UART_PORT if your image uses a different device.
PORT = os.environ.get("PI_UART_PORT", "/dev/ttyAMA0")
BAUD = 115200
READ_TIMEOUT = 0.1

# Change to "interactive" if you want to type commands live.
MODE = "sequence"

# 0 deg = forward, 90 deg = left, -90 deg = right, 180 deg = backward.
COMMAND_SEQUENCE = [
    {"type": "move", "angle": 0, "dist": 60, "speed": 28, "heading": 0},
    {"type": "move", "angle": 90, "dist": 40, "speed": 24, "heading": 0},
    {"type": "turn", "heading": 90, "speed": 12},
    {"type": "move", "angle": 45, "dist": 35, "speed": 22, "heading": 90},
]


@dataclass
class Event:
    raw: str
    data: dict | None


class EspMecanumLink:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.seq = 1
        self.events: queue.Queue[Event] = queue.Queue()
        self.stop_event = threading.Event()
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=READ_TIMEOUT)
        self.ser.reset_input_buffer()
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()

    def close(self) -> None:
        self.stop_event.set()
        self.reader.join(timeout=1.0)
        self.ser.close()

    def next_seq(self) -> int:
        value = self.seq
        self.seq += 1
        return value

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                line = self.ser.readline()
            except serial.SerialException:
                return

            if not line:
                continue

            raw = line.decode("utf-8", errors="replace").strip()
            if not raw:
                continue

            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                json_start = raw.find("{")
                if json_start >= 0:
                    candidate = raw[json_start:]
                    try:
                        data = json.loads(candidate)
                    except json.JSONDecodeError:
                        self.events.put(Event(raw=raw, data=None))
                    else:
                        self.events.put(Event(raw=candidate, data=data))
                    continue
                self.events.put(Event(raw=raw, data=None))
                continue

            self.events.put(Event(raw=raw, data=data))

    def send(self, line: str) -> None:
        payload = (line.strip() + "\n").encode("utf-8")
        self.ser.write(payload)
        self.ser.flush()

    def send_command(self, line: str) -> int:
        seq = self.next_seq()
        if " seq=" not in line:
                        line = f"{line} seq={seq}"
        self.send(line)
        return seq

    def wait_for(self, seq: int | None, wanted_types: set[str], timeout: float, *, match_seq: bool = True) -> dict:
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = deadline - time.time()
            try:
                event = self.events.get(timeout=max(0.05, remaining))
            except queue.Empty:
                continue

            if event.data is None:
                print(f"noise: {event.raw}")
                continue

            event_type = event.data.get("type")
            seq_matches = (seq is None) or (event.data.get("seq") == seq)

            if event_type == "telemetry":
                self._print_telemetry(event.data)
                if event_type in wanted_types and (seq_matches or not match_seq):
                    return event.data
                continue

            if event_type in wanted_types and (seq_matches or not match_seq):
                return event.data

            print(f"event: {event.raw}")

        raise TimeoutError(f"Timed out waiting for {wanted_types} for seq {seq}")

    def drain_until_idle(self, seconds: float = 0.5) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            try:
                event = self.events.get(timeout=0.1)
            except queue.Empty:
                return
            if event.data and event.data.get("type") == "telemetry":
                self._print_telemetry(event.data)
            else:
                print(event.raw)

    @staticmethod
    def _print_telemetry(data: dict) -> None:
        pose = data.get("pose", {})
        imu = data.get("imu", {})
        move = data.get("move", {})
        rpm = data.get("rpm", [])
        print(
            "telemetry | "
            f"mode={data.get('mode')} "
            f"fwd={pose.get('forwardCm', 0):>7}cm "
            f"str={pose.get('strafeCm', 0):>7}cm "
            f"prog={pose.get('progressCm', 0):>7}cm "
            f"rem={pose.get('remainingCm', 0):>7}cm "
            f"yaw={imu.get('yawDeg', 0):>7}deg "
            f"herr={move.get('headingErrorDeg', 0):>7}deg "
            f"rpm={rpm}"
        )


def require_ok_ack(data: dict) -> None:
    if data.get("ok", False):
        return

    cmd = data.get("cmd", "UNKNOWN")
    message = data.get("message", "command rejected")
    raise RuntimeError(f"{cmd} rejected: {message}")


def move_cmd(angle: float, dist: float, speed: float, heading: float | None = None, timeout: int | None = None) -> str:
    parts = [f"MOVE angle={angle}", f"dist={dist}", f"speed={speed}"]
    if heading is not None:
        parts.append(f"heading={heading}")
    if timeout is not None:
        parts.append(f"timeout={timeout}")
    return " ".join(parts)


def turn_cmd(heading: float, speed: float = 12, timeout: int | None = None) -> str:
    parts = [f"TURN heading={heading}", f"speed={speed}"]
    if timeout is not None:
        parts.append(f"timeout={timeout}")
    return " ".join(parts)


def twist_cmd(forward: float, strafe: float, turn: float, timeout_ms: int) -> str:
    return f"TWIST forward={forward} strafe={strafe} turn={turn} timeout={timeout_ms}"


def normalize_direction(word: str) -> float:
    mapping = {
        "forward": 0.0,
        "fwd": 0.0,
        "back": 180.0,
        "backward": 180.0,
        "left": 90.0,
        "right": -90.0,
        "fl": 45.0,
        "front_left": 45.0,
        "fr": -45.0,
        "front_right": -45.0,
        "bl": 135.0,
        "back_left": 135.0,
        "br": -135.0,
        "back_right": -135.0,
    }
    key = word.strip().lower()
    if key not in mapping:
        raise ValueError(f"Unknown direction {word}")
    return mapping[key]


def execute_sequence(link: EspMecanumLink) -> None:
    for item in COMMAND_SEQUENCE:
        kind = item["type"]
        if kind == "move":
            line = move_cmd(
                angle=float(item["angle"]),
                dist=float(item["dist"]),
                speed=float(item["speed"]),
                heading=float(item.get("heading", 0.0)),
                timeout=item.get("timeout"),
            )
        elif kind == "turn":
            line = turn_cmd(
                heading=float(item["heading"]),
                speed=float(item.get("speed", 12.0)),
                timeout=item.get("timeout"),
            )
        elif kind == "twist":
            line = twist_cmd(
                forward=float(item.get("forward", 0.0)),
                strafe=float(item.get("strafe", 0.0)),
                turn=float(item.get("turn", 0.0)),
                timeout_ms=int(item.get("timeout", 1000)),
            )
        else:
            raise ValueError(f"Unsupported command type {kind}")

        print(f"send: {line}")
        seq = link.send_command(line)
        ack = link.wait_for(seq, {"ack"}, timeout=3.0)
        print(f"ack : {ack}")
        require_ok_ack(ack)
        done = link.wait_for(seq, {"done"}, timeout=60.0)
        print(f"done: {done}")


def interactive_shell(link: EspMecanumLink) -> None:
    print("Interactive mode.")
    print("Examples:")
    print("  move forward 60 28 0")
    print("  move 45 40 24 90")
    print("  turn 90 12")
    print("  twist 15 0 0 1000")
    print("  stop")
    print("  zero_imu")
    print("  reset_enc")
    print("  status")
    print("  quit")

    while True:
        raw = input("> ").strip()
        if not raw:
            continue
        if raw.lower() in {"quit", "exit"}:
            return

        parts = raw.split()
        cmd = parts[0].lower()

        if cmd == "move":
            if len(parts) < 4:
                print("usage: move <direction|angleDeg> <distCm> <speedRpm> [headingDeg]")
                continue

            try:
                angle = float(parts[1])
            except ValueError:
                angle = normalize_direction(parts[1])

            dist = float(parts[2])
            speed = float(parts[3])
            heading = float(parts[4]) if len(parts) >= 5 else None
            line = move_cmd(angle=angle, dist=dist, speed=speed, heading=heading)
            seq = link.send_command(line)
            ack = link.wait_for(seq, {"ack"}, timeout=3.0)
            print(ack)
            require_ok_ack(ack)
            print(link.wait_for(seq, {"done"}, timeout=60.0))
            continue

        if cmd == "turn":
            if len(parts) < 2:
                print("usage: turn <headingDeg> [speedRpm]")
                continue
            heading = float(parts[1])
            speed = float(parts[2]) if len(parts) >= 3 else 12.0
            seq = link.send_command(turn_cmd(heading, speed))
            ack = link.wait_for(seq, {"ack"}, timeout=3.0)
            print(ack)
            require_ok_ack(ack)
            print(link.wait_for(seq, {"done"}, timeout=60.0))
            continue

        if cmd == "twist":
            if len(parts) != 5:
                print("usage: twist <forwardRpm> <strafeRpm> <turnRpm> <timeoutMs>")
                continue
            seq = link.send_command(twist_cmd(float(parts[1]), float(parts[2]), float(parts[3]), int(parts[4])))
            ack = link.wait_for(seq, {"ack"}, timeout=3.0)
            print(ack)
            require_ok_ack(ack)
            print(link.wait_for(seq, {"done"}, timeout=10.0))
            continue

        if cmd == "stop":
            seq = link.send_command("STOP")
            ack = link.wait_for(seq, {"ack"}, timeout=3.0)
            print(ack)
            require_ok_ack(ack)
            continue

        if cmd == "zero_imu":
            seq = link.send_command("ZERO_IMU")
            ack = link.wait_for(seq, {"ack"}, timeout=3.0)
            print(ack)
            require_ok_ack(ack)
            continue

        if cmd == "reset_enc":
            seq = link.send_command("RESET_ENC")
            ack = link.wait_for(seq, {"ack"}, timeout=3.0)
            print(ack)
            require_ok_ack(ack)
            continue

        if cmd == "status":
            seq = link.send_command("STATUS")
            ack = link.wait_for(seq, {"ack"}, timeout=3.0)
            print(ack)
            require_ok_ack(ack)
            data = link.wait_for(None, {"telemetry"}, timeout=3.0, match_seq=False)
            print(data)
            continue

        print("unknown command")


def main() -> None:
    link = EspMecanumLink(PORT, BAUD)
    try:
        print(f"opened {PORT} @ {BAUD}")
        link.drain_until_idle(2.0)
        if MODE == "sequence":
            execute_sequence(link)
        else:
            interactive_shell(link)
    finally:
        link.close()


if __name__ == "__main__":
    main()
