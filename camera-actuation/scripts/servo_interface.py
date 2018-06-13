#!/usr/bin/env python

import rospy

from vortex_msgs.msg import CameraPosition
from servo import Servo

COMPUTER = rospy.get_param('/computer')

PWM_FREQENCY = rospy.get_param('/tilt_servo/steps_per_rev')
SERVO_PWM_PIN = rospy.get_param('/tilt_servo/pwm/pin')
DUTY_CYCLE_MIN = rospy.get_param('/tilt_servo/duty_cycle_min')
DUTY_CYCLE_MAX = rospy.get_param('/tilt_servo/duty_cycle_max')


def healthy_message(msg):
    if abs(msg.camera_tilt) > 1:
        rospy.logwarn_throttle(
            1, 'Camera servo position out of range. Ignoring message...')
        return False
    return True


class ServoInterface(object):
    def __init__(self):
        self.is_initialized = False
        rospy.init_node('servo_interface', anonymous=False)
        self.sub = rospy.Subscriber(
            'camera_direction', std_msgs.msg.Float64, self.callback)

        rospy.on_shutdown(self.shutdown)

        try:
            self.tilt_servo = Servo(SERVO_PWM_PIN,
                                    PWM_FREQUENCY,
                                    DUTY_CYCLE_MIN,
                                    DUTY_CYCLE_MAX,
                                    SCALE_RANGE,
                                    COMPUTER)
            self.tilt_position = 0
        except NameError:
            rospy.logfatal('Could not initialize servo.py. Is /computer parameter set correctly? '
                           'Shutting down node...')
            rospy.signal_shutdown('')

        self.tilt_servo.set_position(tilt_position)
        rospy.loginfo('Initialized camera tilt servo in neutral position.')
        self.is_initialized = True
        self.spin()

    def spin(self):
        rate = rospy.Rate(PWM_FREQUENCY)

        while not rospy.is_shutdown():
            tilt_servo_position = tilt_servo_position + \
                tilt_servo_resolution * tilt_servo_direction
            # Move servo if nonzero direction
            if abs(self.tilt_servo_direction) == 1:
                self.tilt_servo.set_position(tilt_position)

            rate.sleep()

    def shutdown(self):
        self.tilt_servo.shutdown()

    def callback(self, msg):
        if not self.is_initialized:
            rospy.logwarn('Callback before node initialized, ignoring...')
            return

        if not healthy_message(msg):
            return

        self.tilt_position = msg.data


if __name__ == '__main__':
    try:
        servo_interface = ServoInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
