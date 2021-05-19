#!/usr/bin/env python

"""
Integration testing for mushr_sim and its dependencies

Author: Schmittle
"""
import roslib; roslib.load_manifest("mushr_sim")  # This line is not needed with Catkin.

from geometry_msgs.msg import PoseStamped
import time
import rospy
import rostest
import sys
import unittest

class TestIntegration(unittest.TestCase):
    """Testing class for integration testing the sim"""

    def __init__(self, *args):
        super(TestIntegration, self).__init__(*args)
        self.pose_success = False

    def pose_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation

        position_success = False
        orientation_success = False
        if abs(position.x) < 0.001 and abs(position.y) < 0.001 and abs(position.z) < 0.001:
            position_success = True
        if abs(orientation.x) < 0.001 and abs(orientation.y) < 0.001 and abs(orientation.z) < 0.001 and abs(orientation.w) - 1.0 < 0.001 :
            orientation_success = True

        self.pose_success = position_success and orientation_success

    def test_car_pose(self):
        print("init test")
        rospy.init_node("car_pose_tester", anonymous=True)
        rospy.Subscriber("/car/car_pose", PoseStamped, self.pose_callback)
        timeout = time.time() + 5.0 #10 sec
        while not rospy.is_shutdown() and not self.pose_success and time.time() < timeout:
            time.sleep(0.1)
        self.assert_(self.pose_success)

if __name__ == "__main__":
    rostest.rosrun("mushr_sim", "test_sim", TestIntegration)
