#!/usr/bin/env python3
# DRV8825 + limit switch homing calibration script for Raspberry Pi.
# Terminal input/output replaces Arduino Serial Monitor.

from gpiozero import DigitalOutputDevice, DigitalInputDevice
import threading
import time
import math

STEP_PIN = 6
DIR_PIN = 13
EN_PIN = 5
LIMIT_PIN = 19   # using your available cluster area would conflict with buzzer; adjust only if your actual limit switch is elsewhere

STEP_HIGH_US = 10
MIN_SPEED_SPS = 0.0
MAX_SPEED_SPS = 2500.0
LIMIT_DEBOUNCE_MS = 25

step_pin = DigitalOutputDevice(STEP_PIN, initial_value=False)
dir_pin = DigitalOutputDevice(DIR_PIN, initial_value=False)
en_pin = DigitalOutputDevice(EN_PIN, initial_value=True)  # disabled initially
limit_pin = DigitalInputDevice(LIMIT_PIN, pull_up=True)

input_line = ""

speed_sps = 500.0
home_speed_sps = 500.0
backoff_speed_sps = 500.0
direction_sign = 1
limit_direction_sign = -1
down_direction_sign = 1
up_direction_sign = -1
home_backoff_steps = 100

enabled = False
continuous_run = False
move_active = False
move_remaining_steps = 0
move_direction_sign = 1
move_speed_sps = 500.0

homing_active = False
homing_phase = 0  # 0 idle, 1 seek switch, 2 back off, 3 done

current_position_steps = 0
last_move_start_steps = 0
last_move_delta_steps = 0
calibration_start_steps = 0
calibration_active = False
steps_per_mm = 0.0

limit_active_low = False
limit_stable_pressed = False
limit_last_raw_pressed = False
limit_last_change_ms = 0.0

last_status_ms = 0.0
running = True
state_lock = threading.Lock()

def millis():
    return int(time.monotonic() * 1000)

def constrain(x, lo, hi):
    return max(lo, min(hi, x))

def print_help():
    print()
    print("Manual movement:")
    print("  start              continuous run using current direction/speed")
    print("  stop               stop motor")
    print("  d 1                direction one way")
    print("  d -1               direction other way")
    print("  s 200              speed in steps/second")
    print("  up                 continuous run upward, same as d -1 + start")
    print("  down               continuous run downward, same as d 1 + start")
    print("  jogup 500          move upward by 500 steps")
    print("  jogdown 500        move downward by 500 steps")
    print("  jog 500            move +500 steps")
    print("  jog -500           move -500 steps")
    print("  goto 1000          move to absolute step position")
    print("  zero               set current position to 0")
    print()
    print("Limit switch:")
    print("  ls                 print limit switch state")
    print("  lsinv 0            pressed = LOW, normal switch to GND")
    print("  lsinv 1            pressed = HIGH, current default")
    print()
    print("Homing calibration:")
    print("  hdir -1            direction toward limit switch")
    print("  hspeed 120         homing seek speed in steps/second")
    print("  backoff 100        steps to move away from switch after hit")
    print("  home               seek switch, back off, set position to 0")
    print()
    print("Height calibration:")
    print("  calstart           remember current step position")
    print("  caldone 25.0       enter measured movement in mm")
    print("  mm 10              move by +10 mm after caldone")
    print("  mm -10             move by -10 mm after caldone")
    print()
    print("Info:")
    print("  status             print current settings")
    print("  help               print this menu")
    print()

def read_limit_raw_pressed():
    raw = 1 if limit_pin.value else 0
    return (raw == 0) if limit_active_low else (raw == 1)

def update_limit_debounce():
    global limit_last_raw_pressed, limit_last_change_ms, limit_stable_pressed
    raw_pressed = read_limit_raw_pressed()
    now = millis()

    if raw_pressed != limit_last_raw_pressed:
        limit_last_raw_pressed = raw_pressed
        limit_last_change_ms = now

    if (now - limit_last_change_ms) >= LIMIT_DEBOUNCE_MS:
        limit_stable_pressed = raw_pressed

def apply_outputs():
    if direction_sign > 0:
        dir_pin.on()
    else:
        dir_pin.off()

    # EN is active LOW
    if enabled:
        en_pin.off()
    else:
        en_pin.on()

def set_enabled(on):
    global enabled, continuous_run, move_active, homing_active, homing_phase
    enabled = on
    apply_outputs()
    if not on:
        step_pin.off()
        continuous_run = False
        move_active = False
        homing_active = False
        homing_phase = 0

def print_status():
    msg = (
        f"\nEnabled={'YES' if enabled else 'NO'}"
        f" | PosSteps={current_position_steps}"
        f" | Dir={direction_sign}"
        f" | UpDir={up_direction_sign}"
        f" | DownDir={down_direction_sign}"
        f" | Speed={speed_sps:.1f} steps/s"
        f" | Limit={'PRESSED' if limit_stable_pressed else 'OPEN'}"
        f" | LimitDir={limit_direction_sign}"
        f" | Backoff={home_backoff_steps} steps"
    )
    if steps_per_mm > 0.0:
        msg += f" | Steps/mm={steps_per_mm:.3f} | PosMm={current_position_steps / steps_per_mm:.3f}"
    print(msg)

def start_relative_move(steps, sps):
    global move_direction_sign, move_remaining_steps, move_speed_sps
    global direction_sign, continuous_run, move_active, enabled, last_move_start_steps

    if steps == 0:
        return

    move_direction_sign = 1 if steps >= 0 else -1
    move_remaining_steps = abs(int(steps))
    move_speed_sps = constrain(sps, 1.0, MAX_SPEED_SPS)
    direction_sign = move_direction_sign
    continuous_run = False
    move_active = True
    enabled = True
    last_move_start_steps = current_position_steps
    apply_outputs()

def finish_move():
    global move_active, last_move_delta_steps
    move_active = False
    last_move_delta_steps = current_position_steps - last_move_start_steps
    print(f"Move done. Delta steps={last_move_delta_steps} | Position={current_position_steps}")

def do_one_step(sps, sign):
    global current_position_steps
    if not enabled or sps <= 0.01:
        step_pin.off()
        return False

    interval_us = int(1000000.0 / sps)
    interval_us = max(interval_us, STEP_HIGH_US + 50)

    step_pin.on()
    time.sleep(STEP_HIGH_US / 1_000_000.0)
    step_pin.off()
    current_position_steps += 1 if sign >= 0 else -1
    time.sleep(max((interval_us - STEP_HIGH_US) / 1_000_000.0, 0.0))
    return True

def run_motion():
    global homing_phase, homing_active, current_position_steps, enabled
    global move_remaining_steps, direction_sign

    if homing_active:
        if homing_phase == 1:
            direction_sign = limit_direction_sign
            apply_outputs()

            if limit_stable_pressed:
                print("Limit switch hit. Backing off...")
                start_relative_move(-limit_direction_sign * home_backoff_steps, backoff_speed_sps)
                homing_phase = 2
                return

            do_one_step(home_speed_sps, limit_direction_sign)
            return

        if homing_phase == 2:
            if move_active:
                if do_one_step(move_speed_sps, move_direction_sign):
                    move_remaining_steps -= 1
                    if move_remaining_steps <= 0:
                        finish_move()
                        current_position_steps = 0
                        homing_active = False
                        homing_phase = 0
                        enabled = False
                        apply_outputs()
                        print("Homing complete. Current position set to 0.")
                        print_status()
            return

    if move_active:
        if do_one_step(move_speed_sps, move_direction_sign):
            move_remaining_steps -= 1
            if move_remaining_steps <= 0:
                finish_move()
        return

    if continuous_run:
        do_one_step(speed_sps, direction_sign)

def start_home():
    global homing_active, homing_phase, continuous_run, move_active, enabled, direction_sign
    if limit_stable_pressed:
        print("Limit already pressed. Backing off first...")
        homing_active = True
        homing_phase = 2
        start_relative_move(-limit_direction_sign * home_backoff_steps, backoff_speed_sps)
        return

    print("Homing started. Seeking limit switch...")
    continuous_run = False
    move_active = False
    homing_active = True
    homing_phase = 1
    enabled = True
    direction_sign = limit_direction_sign
    apply_outputs()

def handle_command(cmd):
    global direction_sign, speed_sps, current_position_steps, calibration_start_steps
    global calibration_active, steps_per_mm, limit_direction_sign, home_speed_sps
    global home_backoff_steps, limit_active_low, continuous_run, move_active, homing_active
    global enabled

    cmd = cmd.strip().lower()
    if len(cmd) == 0:
        return

    if cmd in ("help", "h"):
        print_help()
    elif cmd == "status":
        print_status()
    elif cmd == "ls":
        print(f"Limit raw={'PRESSED' if read_limit_raw_pressed() else 'OPEN'} stable={'PRESSED' if limit_stable_pressed else 'OPEN'}")
    elif cmd == "stop":
        set_enabled(False)
        print("Stopped")
    elif cmd == "start":
        continuous_run = True
        move_active = False
        homing_active = False
        enabled = True
        apply_outputs()
        print("Continuous run started")
    elif cmd == "up":
        direction_sign = up_direction_sign
        continuous_run = True
        move_active = False
        homing_active = False
        enabled = True
        apply_outputs()
        print("Moving upward continuously")
    elif cmd == "down":
        direction_sign = down_direction_sign
        continuous_run = True
        move_active = False
        homing_active = False
        enabled = True
        apply_outputs()
        print("Moving downward continuously")
    elif cmd == "zero":
        current_position_steps = 0
        print("Current position set to 0")
    elif cmd == "home":
        start_home()
    elif cmd == "calstart":
        calibration_start_steps = current_position_steps
        calibration_active = True
        print(f"Calibration start position={calibration_start_steps}")
    elif cmd.startswith("caldone "):
        try:
            mm = float(cmd[8:].strip())
            delta = current_position_steps - calibration_start_steps
            if (not calibration_active) or abs(mm) < 0.001 or delta == 0:
                print("Calibration failed. Use calstart, move, then caldone <measured_mm>.")
            else:
                steps_per_mm = abs(float(delta) / mm)
                calibration_active = False
                print(f"Calibration done. Delta steps={delta} | Measured mm={mm:.3f} | Steps/mm={steps_per_mm:.3f}")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("d "):
        try:
            d = int(cmd[2:].strip())
            direction_sign = 1 if d >= 0 else -1
            apply_outputs()
            print("Direction updated")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("s "):
        try:
            speed_sps = constrain(float(cmd[2:].strip()), MIN_SPEED_SPS, MAX_SPEED_SPS)
            print("Speed updated")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("jog "):
        try:
            steps = int(cmd[4:].strip())
            start_relative_move(steps, speed_sps)
            print(f"Jog started: {steps} steps")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("jogup "):
        try:
            steps = abs(int(cmd[6:].strip()))
            start_relative_move(up_direction_sign * steps, speed_sps)
            print(f"Jog up started: {steps} steps")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("jogdown "):
        try:
            steps = abs(int(cmd[8:].strip()))
            start_relative_move(down_direction_sign * steps, speed_sps)
            print(f"Jog down started: {steps} steps")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("goto "):
        try:
            target = int(cmd[5:].strip())
            start_relative_move(target - current_position_steps, speed_sps)
            print(f"Moving to absolute step position {target}")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("mm "):
        try:
            mm = float(cmd[3:].strip())
            if steps_per_mm <= 0.0:
                print("No height calibration yet. Use calstart, move, then caldone <mm>.")
            else:
                steps = round(mm * steps_per_mm)
                start_relative_move(steps, speed_sps)
                print(f"Moving {mm:.3f} mm = {steps} steps")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("hdir "):
        try:
            d = int(cmd[5:].strip())
            limit_direction_sign = 1 if d >= 0 else -1
            print("Limit direction updated")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("hspeed "):
        try:
            home_speed_sps = constrain(float(cmd[7:].strip()), 1.0, MAX_SPEED_SPS)
            print("Home speed updated")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("backoff "):
        try:
            home_backoff_steps = max(0, int(cmd[8:].strip()))
            print("Backoff steps updated")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("lsinv "):
        try:
            inv = int(cmd[6:].strip())
            limit_active_low = (inv == 0)
            print("Limit mode: pressed LOW" if limit_active_low else "Limit mode: pressed HIGH")
        except ValueError:
            print("Unknown command. Type: help")
    else:
        print("Unknown command. Type: help")

def motion_worker():
    global running, last_status_ms
    while running:
        update_limit_debounce()
        run_motion()

        now = millis()
        if (now - last_status_ms) >= 2000:
            last_status_ms = now
            if enabled or homing_active or move_active or continuous_run:
                print_status()

        if not (enabled or homing_active or move_active or continuous_run):
            time.sleep(0.01)

def main():
    global limit_last_raw_pressed, limit_stable_pressed, limit_last_change_ms, running

    step_pin.off()
    set_enabled(False)

    limit_last_raw_pressed = read_limit_raw_pressed()
    limit_stable_pressed = limit_last_raw_pressed
    limit_last_change_ms = millis()

    print("Stepper homing calibration started")
    print_help()
    print_status()

    worker = threading.Thread(target=motion_worker, daemon=True)
    worker.start()

    try:
        while True:
            line = input()
            if len(line) > 60:
                print("Input too long. Type: help")
                continue
            handle_command(line)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        running = False
        step_pin.off()
        en_pin.on()

if __name__ == "__main__":
    main()