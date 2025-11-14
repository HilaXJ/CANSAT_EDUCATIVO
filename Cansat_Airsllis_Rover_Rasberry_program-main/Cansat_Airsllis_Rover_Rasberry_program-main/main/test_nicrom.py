#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import RPi.GPIO as GPIO

PIN_NICROM = 6      # GPIO6 en modo BCM
TIEMPO_ON = 30      # segundos

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_NICROM, GPIO.OUT, initial=GPIO.LOW)

    print("NICROM ON (GPIO6 HIGH)")
    GPIO.output(PIN_NICROM, GPIO.HIGH)

    time.sleep(TIEMPO_ON)

    print("NICROM OFF (GPIO6 LOW)")
    GPIO.output(PIN_NICROM, GPIO.LOW)

    GPIO.cleanup()
    print("Fin del programa.")

if __name__ == "__main__":
    main()
