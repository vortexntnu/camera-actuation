#!/usr/bin/env python

from servo import Servo
import time

pwm_pin = 12
pwm_frequency = 50
dc_min = 0.05  # dc_min and max for SG90 servo
dc_max = 0.1
scale_servo_range = 0.9
computer = 'nanopi'

servo = ServoInterface()
servo.spin()
