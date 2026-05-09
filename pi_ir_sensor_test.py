#!/usr/bin/env python3
# Simple 6 IR sensor test for Raspberry Pi.
# Terminal output replaces Arduino Serial Monitor.

import importlib.util
import os
import sys

SYSTEM_PYTHON = "/usr/bin/python3"
REEXEC_GUARD = "PI_TESTING_SYSTEM_PYTHON"


def module_exists(name):
    try:
        return importlib.util.find_spec(name) is not None
    except ModuleNotFoundError:
        return False


def reexec_target():
    argv0 = sys.argv[0] if sys.argv else ""
    if argv0 and argv0 != "-" and os.path.exists(argv0):
        return os.path.abspath(argv0)
    return os.path.abspath(__file__)


def ensure_gpiozero_runtime():
    has_gpiozero = module_exists("gpiozero")
    has_backend = any(module_exists(name) for name in ("lgpio", "RPi.GPIO", "pigpio"))
    if has_gpiozero and has_backend:
        return

    if (
        os.path.exists(SYSTEM_PYTHON)
        and os.environ.get(REEXEC_GUARD) != "1"
        and os.path.abspath(sys.executable) != SYSTEM_PYTHON
    ):
        os.environ[REEXEC_GUARD] = "1"
        os.execv(SYSTEM_PYTHON, [SYSTEM_PYTHON, reexec_target(), *sys.argv[1:]])

    if not has_gpiozero:
        raise SystemExit(
            "gpiozero is not installed for this Python interpreter.\n"
            f"Run this script with {SYSTEM_PYTHON} or install python3-gpiozero."
        )

    raise SystemExit(
        "gpiozero is installed, but no supported GPIO pin backend is available for this Python interpreter.\n"
        f"Run this script with {SYSTEM_PYTHON} or install lgpio/pigpio/RPi.GPIO for {sys.executable}."
    )


ensure_gpiozero_runtime()

from gpiozero import DigitalInputDevice
from signal import pause
import time

IR_COUNT = 6

# Raspberry Pi GPIO pins from your mapping:
# IR1 -> GPIO23
# IR2 -> GPIO24
# IR3 -> GPIO25
# IR4 -> GPIO17
# IR5 -> GPIO27
# IR6 -> GPIO22
ir_pins = [23, 24, 25, 17, 27, 22]

sensors = [DigitalInputDevice(pin, pull_up=False) for pin in ir_pins]

def main():
    print("6 IR sensor test started")
    print("Format: IR1 IR2 IR3 IR4 IR5 IR6")
    try:
        while True:
            parts = []
            for i in range(IR_COUNT):
                reading = 1 if sensors[i].value else 0
                parts.append(f"IR{i+1}={reading}")
            print("  ".join(parts), flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()
