#!/usr/bin/env python3
# 3 LED pattern tester for Raspberry Pi.
# Terminal input/output replaces Arduino Serial Monitor.

from gpiozero import DigitalOutputDevice
import time
import random
import threading

# Raspberry Pi traffic light mapping:
# GPIO16 -> green
# GPIO20 -> yellow
# GPIO21 -> red

LED1_PIN = 16
LED2_PIN = 20
LED3_PIN = 21

led_pins = [
    DigitalOutputDevice(LED1_PIN, initial_value=False),
    DigitalOutputDevice(LED2_PIN, initial_value=False),
    DigitalOutputDevice(LED3_PIN, initial_value=False),
]

pattern = 1
speed_ms = 150
enabled = True
running = True

def all_off():
    for led in led_pins:
        led.off()

def set_led(index):
    all_off()
    if 0 <= index < 3:
        led_pins[index].on()

def print_help():
    print()
    print("Commands:")
    print("  p 1       chase forward")
    print("  p 2       bounce")
    print("  p 3       fill and clear")
    print("  p 4       sparkle")
    print("  s 150     speed in milliseconds")
    print("  on        enable pattern")
    print("  off       turn all LEDs off")
    print("  status    print current settings")
    print()

def print_status():
    print(f"Enabled={'YES' if enabled else 'NO'} | Pattern={pattern} | Speed={speed_ms} ms")

def constrain(x, lo, hi):
    return max(lo, min(hi, x))

def handle_command(cmd):
    global pattern, speed_ms, enabled
    cmd = cmd.strip().lower()
    if len(cmd) == 0:
        return

    if cmd in ("help", "h"):
        print_help()
    elif cmd == "status":
        print_status()
        return
    elif cmd == "on":
        enabled = True
        print("Patterns enabled")
    elif cmd == "off":
        enabled = False
        all_off()
        print("LEDs off")
    elif cmd.startswith("p "):
        try:
            pattern = constrain(int(cmd[2:].strip()), 1, 4)
            all_off()
            print("Pattern updated")
        except ValueError:
            print("Unknown command. Type: help")
    elif cmd.startswith("s "):
        try:
            speed_ms = constrain(int(cmd[2:].strip()), 30, 2000)
            print("Speed updated")
        except ValueError:
            print("Unknown command. Type: help")
    else:
        print("Unknown command. Type: help")

    print_status()

def wait_with_terminal(ms):
    time.sleep(ms / 1000.0)

def pattern_chase():
    for i in range(3):
        if not enabled or pattern != 1:
            break
        set_led(i)
        wait_with_terminal(speed_ms)

def pattern_bounce():
    order = [0, 1, 2, 1]
    for i in order:
        if not enabled or pattern != 2:
            break
        set_led(i)
        wait_with_terminal(speed_ms)

def pattern_fill_clear():
    all_off()
    for i in range(3):
        if not enabled or pattern != 3:
            break
        led_pins[i].on()
        wait_with_terminal(speed_ms)
    for i in range(2, -1, -1):
        if not enabled or pattern != 3:
            break
        led_pins[i].off()
        wait_with_terminal(speed_ms)

def pattern_sparkle():
    a = random.randint(0, 2)
    b = random.randint(0, 2)
    all_off()
    led_pins[a].on()
    led_pins[b].on()
    wait_with_terminal(speed_ms)
    all_off()
    wait_with_terminal(max(30, speed_ms // 2))

def pattern_worker():
    global running
    while running:
        if not enabled:
            all_off()
            time.sleep(0.02)
            continue

        if pattern == 1:
            pattern_chase()
        elif pattern == 2:
            pattern_bounce()
        elif pattern == 3:
            pattern_fill_clear()
        elif pattern == 4:
            pattern_sparkle()

def main():
    global running
    all_off()

    print("3 LED pattern tester started")
    print("Pins: LED1=GPIO16, LED2=GPIO20, LED3=GPIO21")
    print_help()
    print_status()

    worker = threading.Thread(target=pattern_worker, daemon=True)
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
        all_off()

if __name__ == "__main__":
    main()