#!/usr/bin/env python3
# Listen for limit switch events sent from an ESP over UART.
# Also allows sending test commands from the Pi to the ESP.

import argparse
import importlib.util
import os
import sys
import threading
import time

SYSTEM_PYTHON = "/usr/bin/python3"
REEXEC_GUARD = "PI_TESTING_SYSTEM_PYTHON"
DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 115200


def ensure_pyserial_runtime():
    if importlib.util.find_spec("serial") is not None:
        return

    if os.path.exists(SYSTEM_PYTHON) and os.environ.get(REEXEC_GUARD) != "1":
        os.environ[REEXEC_GUARD] = "1"
        os.execv(SYSTEM_PYTHON, [SYSTEM_PYTHON, os.path.abspath(__file__), *sys.argv[1:]])

    raise SystemExit(
        "pyserial is not installed for this Python interpreter.\n"
        f"Run this script with {SYSTEM_PYTHON} or install python3-serial."
    )


ensure_pyserial_runtime()

import serial


def normalize_limit_state(line):
    text = line.strip().upper()

    if text in ("LIMIT:PRESSED", "PRESSED", "LIMIT=1", "LIMIT 1"):
        return "PRESSED"
    if text in ("LIMIT:RELEASED", "RELEASED", "LIMIT=0", "LIMIT 0", "OPEN"):
        return "RELEASED"
    return None


def open_serial(port, baud):
    try:
        return serial.Serial(port=port, baudrate=baud, timeout=0.2)
    except serial.SerialException as exc:
        raise SystemExit(f"Could not open serial port {port}: {exc}") from exc


def send_line(link, text):
    payload = text.strip()
    if not payload:
        return
    link.write(f"{payload}\n".encode())


def print_help():
    print()
    print("Pi -> ESP commands:")
    print("  ping        expect PONG from ESP")
    print("  status      ask ESP for current limit switch state")
    print("  test high   drive ESP test output pin HIGH")
    print("  test low    drive ESP test output pin LOW")
    print("  raw <text>  send any custom UART line to the ESP")
    print("  help        print this help")
    print("  quit        stop the script")
    print()


def main():
    parser = argparse.ArgumentParser(description="Listen for UART limit switch events from an ESP32.")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port to use. Default: {DEFAULT_PORT}")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"UART baud rate. Default: {DEFAULT_BAUD}")
    args = parser.parse_args()

    last_state = None

    with open_serial(args.port, args.baud) as link:
        print(f"UART limit listener started on {args.port} @ {args.baud} baud")
        print("Waiting for ESP messages. Type help for Pi -> ESP test commands.")
        print_help()

        # Ask the ESP to immediately report its current switch state.
        send_line(link, "STATUS?")

        stop_event = threading.Event()

        def reader():
            nonlocal last_state
            while not stop_event.is_set():
                try:
                    raw = link.readline()
                except (serial.SerialException, OSError):
                    if stop_event.is_set():
                        return
                    print("Serial link closed unexpectedly.")
                    stop_event.set()
                    return

                if not raw:
                    continue

                line = raw.decode(errors="replace").strip()
                if not line:
                    continue

                timestamp = time.strftime("%H:%M:%S")
                state = normalize_limit_state(line)
                if state is None:
                    print(f"[{timestamp}] RX {line}")
                    continue

                changed = state != last_state
                last_state = state
                suffix = " changed" if changed else ""
                print(f"[{timestamp}] Limit switch {state}{suffix}")
                send_line(link, f"ACK:{state}")

        worker = threading.Thread(target=reader, daemon=True)
        worker.start()

        try:
            while True:
                command = input("> ").strip()
                if not command:
                    continue

                lowered = command.lower()
                if lowered in ("quit", "exit"):
                    break
                if lowered in ("help", "h"):
                    print_help()
                    continue
                if lowered == "ping":
                    send_line(link, "PING")
                    continue
                if lowered == "status":
                    send_line(link, "STATUS?")
                    continue
                if lowered == "test high":
                    send_line(link, "TEST HIGH")
                    continue
                if lowered == "test low":
                    send_line(link, "TEST LOW")
                    continue
                if lowered.startswith("raw "):
                    send_line(link, command[4:])
                    continue

                print("Unknown command. Type help.")
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            stop_event.set()
            worker.join(timeout=1.0)


if __name__ == "__main__":
    main()
