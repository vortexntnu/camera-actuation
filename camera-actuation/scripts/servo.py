#!/usr/bin/env python
import time
from numpy import clip


class Servo:
    def __init__(self, pwm_pin, pwm_frequency, duty_cycle_min, duty_cycle_max, scale_range, computer):
        # dc_min and dc_max are max and min duty cycles for the servo
        # for the tower pro SG90 9g servo this is [0.05, 0.1]
        """Initialise a non continous servo
        scale_range - shrink servo range to allow for shitty pwm, 0.8 recomended for nanopi"""
        self.computer = computer
        if self.computer == 'nanopi':
            import RPi.GPIO as GPIO
            print('servo.py: Starting in nanopi mode')
        else:
            print('stepper.py: Invalid computer!')

        self.GPIO = GPIO
        self.pwm_pin = pwm_pin
        self.pwm_frequency = pwm_frequency
        self.duty_cycle_min = duty_cycle_min
        self.duty_cylce_max = duty_cycle_max
        self.scale_range = clip(scale_range, 0, 1)

        self.GPIO.setmode(GPIO.BOARD)
        self.GPIO.setup(self.pwm_pin, self.GPIO.OUT)

        self.pwm = self.GPIO.PWM(self.pwm_pin, self.pwm_frequency)
        self.enable()

    def set_position(self, position):
        # Set servo position
        if abs(position) <= 1:
            self.pwm.ChangeDutyCycle(self.position_to_dc(position)*100)
            time.sleep(0.1)
            self.pwm.ChangeDutyCycle(0)
        else:
            print 'Servo position out of range, ignoring'

    def enable(self):
        self.pwm.start(self.position_to_dc(0))

    def disable(self):
        if self.computer == 'nanopi':
            pwm.stop()
            print('Disabling servo')

    def shutdown(self):
        if self.computer == 'nanopi':
            print('servo.py: Shutting down and cleaning GPIO pins.')
            self.pwm.stop()
            self.GPIO.cleanup()

    def position_to_duty_cycle(self, position):
        # transform position [-1, 1] to a scaled duty cycle [duty_cycle_min, duty_cycle_max]
        return ((self.duty_cycle_max - self.duty_cycle_min) *
                (position*(self.scale_range) + 1)/2 + self.dc_min)
