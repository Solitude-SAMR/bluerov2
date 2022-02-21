#!/usr/bin/env python
"""
Node for controlling pwm lights from pi
Subscribes: light level topic
Sets lights to correct PWM value
"""
import time
import rospy
from std_msgs.msg import Float32
import pigpio

LIGHTSGPIO = 24 #CHECK!!!!!!
TILTGPIO = 23 #CHECK!!!!!!


minPWM = 1000
maxPWM = 2000

class GPIO_PWM_Controller():
    def __init__(self, name, GPIO, value=0.0, test=True, invert=False):
        self.name = name
        self.GPIO = GPIO
        self.value = value
        self.piPWM = pigpio.pi()
        self.invert = invert
        self.piPWM.set_mode(self.GPIO, pigpio.OUTPUT)
        rospy.Subscriber('bluerov2/'+self.name+'/set', Float32, self.update)
        self.debug_pub = rospy.Publisher('bluerov2/'+self.name+'/value', Float32, queue_size=10)
        
        if test:
            self.test()
        self.update(Float32(self.value))
    def update(self, msg):
        self.value = msg.data
        pwm = (msg.data * (maxPWM-minPWM)) + minPWM
        if self.invert:
            pwm = 3000 - pwm
        self.debug_pub.publish(self.value)
        self.piPWM.set_servo_pulsewidth(self.GPIO, pwm)
        
    def test(self):
        '''Power-On Self Test'''
        rospy.logwarn('GPIO_PWM_Controller test: ' + self.name + ' : max')
        self.piPWM.set_servo_pulsewidth(self.GPIO, maxPWM)
        time.sleep(2)
        rospy.logwarn('GPIO_PWM_Controller test: ' + self.name + ' : min')
        self.piPWM.set_servo_pulsewidth(self.GPIO, minPWM)


if __name__ == '__main__':
    try:
        rospy.init_node('GPIO_PWM_Controller_node', anonymous=True)
        lights = GPIO_PWM_Controller("light", LIGHTSGPIO, value=0.0) #In future read these from args?
        tilt = GPIO_PWM_Controller("camera_tilt", TILTGPIO, value=0.5, invert=True)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
