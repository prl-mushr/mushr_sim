#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import sys
from threading import Lock

class JoySim:
    def __init__(self):
        self.DEADZONE = 0.01
        self.AUTOREPEAT_RATE = 0.0
        if rospy.has_param("joy_node/deadzone"):
            self.DEADZONE = rospy.get_param("joy_node/deadzone")
        if rospy.has_param("joy_node/autorepeat_rate"):
            self.AUTOREPEAT_RATE = rospy.get_param("joy_node/autorepeat_rate")

        self.key_sub = rospy.Subscriber('key_vel', geometry_msgs.msg.Twist, self.convert_twist_to_joy_cb)
        self.joy_pub = rospy.Publisher('joy', sensor_msgs.msg.Joy, queue_size=1)

        self.msg_lock = Lock()
        self.joy_msg = sensor_msgs.msg.Joy()
        self.joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        if self.AUTOREPEAT_RATE >= sys.float_info.epsilon:
            self.timer = rospy.Timer(rospy.Duration.from_sec(1.0/self.AUTOREPEAT_RATE), self.timer_cb)

    def convert_twist_to_joy_cb(self, msg):
        self.msg_lock.acquire()

        if msg.linear.x < -self.DEADZONE:
            self.joy_msg.axes[1] = -1.0
            self.joy_msg.buttons[4] = 1
            self.joy_msg.buttons[5] = 0
        elif msg.linear.x > self.DEADZONE:
            self.joy_msg.axes[1] = 1.0
            self.joy_msg.buttons[4] = 1
            self.joy_msg.buttons[5] = 0
	else:
	    self.joy_msg.axes[1] = 0.0
	    self.joy_msg.buttons[4] = 0
	    self.joy_msg.buttons[5] = 1

        if msg.angular.z < -self.DEADZONE:
            self.joy_msg.axes[3] = -1.0
            self.joy_msg.buttons[4] = 1
            self.joy_msg.buttons[5] = 0
        elif msg.angular.z > self.DEADZONE:
            self.joy_msg.axes[3] = 1.0
            self.joy_msg.buttons[4] = 1
            self.joy_msg.buttons[5] = 0
	else:
	    self.joy_msg.axes[3] = 0.0

        if (msg.linear.x > -self.DEADZONE and msg.linear.x < self.DEADZONE and
            msg.angular.z > -self.DEADZONE and msg.angular.z < self.DEADZONE):
            self.joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.joy_msg.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]

        if self.AUTOREPEAT_RATE < sys.float_info.epsilon:
            self.joy_pub.publish(self.joy_msg)

        self.msg_lock.release()

    def timer_cb(self, event):
        self.msg_lock.acquire()
        self.joy_pub.publish(self.joy_msg)
        self.msg_lock.release()

if __name__ == '__main__':
    rospy.init_node('joy_sim')

    js = JoySim()
    rospy.spin()
