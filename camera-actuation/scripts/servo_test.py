#!/usr/bin/env python

from servo import Servo
import time

pwm_pin = 12
pwm_frequency = 50
dc_min = 0.05  # dc_min and max for SG90 servo
dc_max = 0.1
scale_servo_range = 0.9
computer = 'nanopi'

servo = Servo(pwm_pin, pwm_frequency, dc_min,
              dc_max, scale_servo_range, computer)

servo.enable()
servo.set_position(0)
while True:
    pass
    position = input("Input servo position[-1, 1])
    servo.set_position(position)
    print "set servo to position", position
