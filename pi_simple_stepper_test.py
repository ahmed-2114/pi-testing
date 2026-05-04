#!/usr/bin/env python3
# DRV8825 + NEMA17 terminal speed/direction test for Raspberry Pi.
# Terminal input/output replaces Arduino Serial Monitor.

from gpiozero import DigitalOutputDevice
import threading
import time
import sys

# Wiring on Raspberry Pi:
# GPIO6  -> STEP
# GPIO13 -> DIR
# GPIO5  -> EN

STEP_PIN = 6
DIR_PIN = 13
EN_PIN = 5

STEPS_PER_REV = 200
STEP_HIGH_US = 10
MIN_SPEED_SPS = 0.0
MAX_SPEED_SPS = 1200.0

step_pin = DigitalOutputDevice(STEP_PIN, initial_value=False)
dir_pin = DigitalOutputDevice(DIR_PIN, initial_value=False)
en_pin = DigitalOutputDevice(EN_PIN, initial_value=False)

input_line = ""
speed_sps = 100.0
direction_sign = 1
enabled = True

state_lock = threading.Lock()
running = True

def constrain(x, lo, hi):
    return max(lo, min(hi, x))

def print_help():
    print()
    print("Commands:")
    print("  d 1        direction one way")
    print("  d -1       direction other way")
    print("  s 100      speed in steps/second")
    print("  r 30       speed in RPM, based on STEPS_PER_REV")
    print("  stop       stop stepping")
    print("  start      enable stepping")
    print("  status     print current settings")
    print()

def print_status():
    with state_lock:
        rpm = (speed_sps * 60.0) / float(STEPS_PER_REV)
        print(f"Enabled={'YES' if enabled else 'NO'} | Direction={direction_sign} | Speed={speed_sps:.1f} steps/s | RPM={rpm:.2f}")

def apply_outputs():
    with state_lock:
        if direction_sign > 0:
            dir_pin.on()
        else:
            dir_pin.off()

        # EN is active LOW
        if enabled:
            en_pin.off()
        else:
            en_pin.on()

def set_speed_sps(sps):
    global speed_sps, enabled
    with state_lock:
        speed_sps = constrain(sps, MIN_SPEED_SPS, MAX_SPEED_SPS)
        if speed_sps <= 0.01:
            enabled = False
        else:
            enabled = True
    apply_outputs()

def handle_command(cmd):
    global enabled, direction_sign
    cmd = cmd.strip().lower()
    if len(cmd) == 0:
        return

    if cmd in ("help", "h"):
        print_help()
    elif cmd == "status":
        print_status()
        return
    elif cmd == "stop":
        with state_lock:
            enabled = False
        apply_outputs()
        print("Stopped")
    elif cmd == "start":
        with state_lock:
            if speed_sps > 0.01:
                enabled = True
        apply_outputs()
        print("Started")
    elif cmd.startswith("d "):
        try:
            d = int(cmd[2:].strip())
            with state_lock:
                direction_sign = 1 if d >= 0 else -1
            apply_outputs()
            print("Direction updated")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("s "):
        try:
            sps = float(cmd[2:].strip())
            set_speed_sps(sps)
            print("Speed updated in steps/s")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("r "):
        try:
            rpm = float(cmd[2:].strip())
            set_speed_sps((rpm * float(STEPS_PER_REV)) / 60.0)
            print("Speed updated in RPM")
        except ValueError:
            print("Unknown command. Type: help")
    else:
        print("Unknown command. Type: help")

    print_status()

def run_stepper():
    global running
    while running:
        with state_lock:
            local_enabled = enabled
            local_speed_sps = speed_sps

        if not local_enabled or local_speed_sps <= 0.01:
            step_pin.off()
            time.sleep(0.001)
            continue

        interval_us = int(1000000.0 / local_speed_sps)
        interval_us = max(interval_us, STEP_HIGH_US + 50)

        step_pin.on()
        time.sleep(STEP_HIGH_US / 1_000_000.0)
        step_pin.off()
        time.sleep(max((interval_us - STEP_HIGH_US) / 1_000_000.0, 0.0))

def main():
    global running
    step_pin.off()
    apply_outputs()

    print("DRV8825 serial stepper test started")
    print_help()
    print_status()

    worker = threading.Thread(target=run_stepper, daemon=True)
    worker.start()

    try:
        while True:
            line = input()
            if len(line) > 40:
                print("Input too long. Type: help")
                continue
            handle_command(line)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        running = False
        step_pin.off()
        en_pin.on()  # disable driver

if __name__ == "__main__":
    main()