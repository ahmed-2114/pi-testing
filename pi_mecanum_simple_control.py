import json
import os
import queue
import threading
import time
from dataclasses import dataclass

import serial


PORT = os.environ.get("PI_UART_PORT", "/dev/ttyAMA0")
BAUD = 115200
READ_TIMEOUT = 0.1

DEFAULT_TURN_SPEED_RPM = 12.0
PING_TIMEOUT_S = 2.5
INIT_TIMEOUT_S = 8.0
ACK_TIMEOUT_S = 3.0
MOVE_DONE_TIMEOUT_S = 180.0
TURN_DONE_TIMEOUT_S = 120.0
PROGRESS_PRINT_INTERVAL_S = 0.35


@dataclass
class Event:
    raw: str
    data: dict | None


class EspPiControlLink:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.seq = 1
        self.events: queue.Queue[Event] = queue.Queue()
        self.stop_event = threading.Event()
        self.last_telemetry: dict | None = None
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=READ_TIMEOUT)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
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

    def drain(self, seconds: float = 0.8) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            try:
                event = self.events.get(timeout=0.1)
            except queue.Empty:
                return

            if event.data and event.data.get("type") == "telemetry":
                self.last_telemetry = event.data

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
                continue

            data = event.data
            event_type = data.get("type")
            if event_type == "telemetry":
                self.last_telemetry = data

            seq_matches = (seq is None) or (data.get("seq") == seq)
            if event_type in wanted_types and (seq_matches or not match_seq):
                return data

        raise TimeoutError(f"Timed out waiting for {wanted_types} for seq {seq}")

    def wait_for_done(self, seq: int, timeout: float) -> dict:
        deadline = time.time() + timeout
        last_progress_print = 0.0

        while time.time() < deadline:
            remaining = deadline - time.time()
            try:
                event = self.events.get(timeout=max(0.05, remaining))
            except queue.Empty:
                continue

            if event.data is None:
                continue

            data = event.data
            event_type = data.get("type")

            if event_type == "telemetry":
                self.last_telemetry = data
                now = time.time()
                if now - last_progress_print >= PROGRESS_PRINT_INTERVAL_S:
                    print(progress_summary(data))
                    last_progress_print = now
                continue

            if event_type == "done" and data.get("seq") == seq:
                return data

        raise TimeoutError(f"Timed out waiting for done for seq {seq}")


def require_ok_ack(data: dict) -> None:
    if data.get("ok", False):
        return

    cmd = data.get("cmd", "UNKNOWN")
    message = data.get("message", "command rejected")
    raise RuntimeError(f"{cmd} rejected: {message}")


def try_ack(link: EspPiControlLink, line: str, timeout: float) -> dict | None:
    seq = link.send_command(line)
    try:
        return link.wait_for(seq, {"ack"}, timeout=timeout)
    except TimeoutError:
        return None


def move_command(direction: str, heading_deg: float, distance_cm: float, speed_rpm: float) -> str:
    direction_key = direction.strip().lower()
    if direction_key in {"forward", "fwd", "f"}:
        angle_deg = 0.0
    elif direction_key in {"back", "backward", "b"}:
        angle_deg = 180.0
    else:
        raise ValueError("Direction must be forward or back")

    return f"MOVE angle={angle_deg} dist={abs(distance_cm)} speed={speed_rpm} heading={heading_deg}"


def turn_command(heading_deg: float, speed_rpm: float) -> str:
    return f"TURN heading={heading_deg} speed={speed_rpm}"


def telemetry_summary(data: dict) -> str:
    pose = data.get("pose", {})
    imu = data.get("imu", {})
    move = data.get("move", {})
    return (
        "status | "
        f"mode={data.get('mode')} "
        f"yaw={imu.get('yawDeg', 0):.2f}deg "
        f"fwd={pose.get('forwardCm', 0):.2f}cm "
        f"str={pose.get('strafeCm', 0):.2f}cm "
        f"prog={pose.get('progressCm', 0):.2f}cm "
        f"rem={pose.get('remainingCm', 0):.2f}cm "
        f"herr={move.get('headingErrorDeg', 0):.2f}deg"
    )


def progress_summary(data: dict) -> str:
    pose = data.get("pose", {})
    imu = data.get("imu", {})
    move = data.get("move", {})
    rpm = data.get("rpm", [])
    return (
        "progress | "
        f"mode={data.get('mode')} "
        f"prog={pose.get('progressCm', 0):.2f}cm "
        f"rem={pose.get('remainingCm', 0):.2f}cm "
        f"yaw={imu.get('yawDeg', 0):.2f}deg "
        f"herr={move.get('headingErrorDeg', 0):.2f}deg "
        f"rpm={rpm}"
    )


def print_done(done: dict) -> None:
    print(
        "done | "
        f"result={done.get('result')} "
        f"heading={done.get('headingDeg', 0):.2f}deg "
        f"forward={done.get('forwardCm', 0):.2f}cm "
        f"strafe={done.get('strafeCm', 0):.2f}cm "
        f"progress={done.get('progressCm', 0):.2f}cm"
    )


def initialize_robot(link: EspPiControlLink) -> None:
    print("checking UART link")
    link.drain(1.0)

    if link.last_telemetry is None:
        seq = link.send_command("PING")
        try:
            pong = link.wait_for(seq, {"pong"}, timeout=PING_TIMEOUT_S)
        except TimeoutError as exc:
            raise RuntimeError(
                "No UART response from ESP. "
                "The Pi opened /dev/ttyAMA0 but the ESP did not answer PING. "
                "Check power, reset the ESP, confirm TX0/RX0 wiring, and make sure the flashed sketch is running."
            ) from exc
        print(f"link ok | pong seq={pong.get('seq')}")
    else:
        print("link ok | passive telemetry received")

    print("initializing IMU: calibrate bias + zero yaw")
    ack = try_ack(link, "INIT_IMU", INIT_TIMEOUT_S)
    if ack is not None:
        require_ok_ack(ack)
        print(f"init | {ack.get('message')}")
        return

    print("init fallback | INIT_IMU did not answer, trying CAL_IMU then ZERO_IMU")
    ack = try_ack(link, "CAL_IMU", INIT_TIMEOUT_S)
    if ack is None:
        raise RuntimeError(
            "ESP answered PING earlier, but did not answer INIT_IMU or CAL_IMU. "
            "Flash esp_mecanum_pi_control.ino, then reset the ESP and retry."
        )

    require_ok_ack(ack)
    print(f"init | {ack.get('message')}")

    ack = try_ack(link, "ZERO_IMU", ACK_TIMEOUT_S)
    if ack is None:
        raise RuntimeError("IMU bias calibration finished, but ZERO_IMU did not answer.")
    require_ok_ack(ack)
    print(f"init | {ack.get('message')}")


def request_status(link: EspPiControlLink) -> dict:
    seq = link.send_command("STATUS")
    ack = link.wait_for(seq, {"ack"}, timeout=ACK_TIMEOUT_S)
    require_ok_ack(ack)
    return link.wait_for(None, {"telemetry"}, timeout=ACK_TIMEOUT_S, match_seq=False)


def send_stop(link: EspPiControlLink) -> None:
    seq = link.send_command("STOP")
    ack = link.wait_for(seq, {"ack"}, timeout=ACK_TIMEOUT_S)
    require_ok_ack(ack)
    print(f"stop | {ack.get('message')}")


def execute_move(link: EspPiControlLink, direction: str, heading_deg: float, distance_cm: float, speed_rpm: float) -> None:
    line = move_command(direction, heading_deg, distance_cm, speed_rpm)
    print(f"send | {line}")
    seq = link.send_command(line)
    ack = link.wait_for(seq, {"ack"}, timeout=ACK_TIMEOUT_S)
    require_ok_ack(ack)
    print(f"ack  | {ack.get('message')}")
    done = link.wait_for_done(seq, timeout=MOVE_DONE_TIMEOUT_S)
    print_done(done)


def execute_turn(link: EspPiControlLink, heading_deg: float, speed_rpm: float) -> None:
    line = turn_command(heading_deg, speed_rpm)
    print(f"send | {line}")
    seq = link.send_command(line)
    ack = link.wait_for(seq, {"ack"}, timeout=ACK_TIMEOUT_S)
    require_ok_ack(ack)
    print(f"ack  | {ack.get('message')}")
    done = link.wait_for_done(seq, timeout=TURN_DONE_TIMEOUT_S)
    print_done(done)


def print_help() -> None:
    print("Commands:")
    print("  move forward <heading_deg> <distance_cm> <speed_rpm>")
    print("  move back <heading_deg> <distance_cm> <speed_rpm>")
    print("  turn <heading_deg> [speed_rpm]")
    print("  status")
    print("  reinit")
    print("  stop")
    print("  help")
    print("  quit")


def interactive_shell(link: EspPiControlLink) -> None:
    print_help()

    while True:
        try:
            raw = input("> ").strip()
        except EOFError:
            return

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        try:
            if cmd in {"quit", "exit"}:
                return

            if cmd == "help":
                print_help()
                continue

            if cmd == "status":
                data = request_status(link)
                print(telemetry_summary(data))
                continue

            if cmd == "reinit":
                initialize_robot(link)
                data = request_status(link)
                print(telemetry_summary(data))
                continue

            if cmd == "stop":
                send_stop(link)
                continue

            if cmd == "turn":
                if len(parts) not in {2, 3}:
                    print("usage: turn <heading_deg> [speed_rpm]")
                    continue

                heading_deg = float(parts[1])
                speed_rpm = float(parts[2]) if len(parts) == 3 else DEFAULT_TURN_SPEED_RPM
                execute_turn(link, heading_deg, speed_rpm)
                continue

            if cmd == "move":
                if len(parts) != 5:
                    print("usage: move <forward|back> <heading_deg> <distance_cm> <speed_rpm>")
                    continue

                direction = parts[1]
                heading_deg = float(parts[2])
                distance_cm = float(parts[3])
                speed_rpm = float(parts[4])
                execute_move(link, direction, heading_deg, distance_cm, speed_rpm)
                continue

            print("unknown command")
        except ValueError as exc:
            print(f"input error | {exc}")
        except TimeoutError as exc:
            print(f"timeout | {exc}")
        except RuntimeError as exc:
            print(f"error | {exc}")


def main() -> None:
    link = EspPiControlLink(PORT, BAUD)
    try:
        print(f"opened {PORT} @ {BAUD}")
        link.drain(1.2)
        initialize_robot(link)
        data = request_status(link)
        print(telemetry_summary(data))
        interactive_shell(link)
    finally:
        link.close()


if __name__ == "__main__":
    main()
