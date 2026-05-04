#!/usr/bin/env python3
# Simple 6 IR sensor test for Raspberry Pi.
# Terminal output replaces Arduino Serial Monitor.

from gpiozero import DigitalInputDevice
from signal import pause
import time
import sys

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