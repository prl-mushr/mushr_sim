#!/usr/bin/env python

"""
Integration testing for mushr_sim and its dependencies

Author: Schmittle
"""
import roslib; roslib.load_manifest("mushr_sim")  # This line is not needed with Catkin.

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import time
import rospy
import rostest
import sys
import numpy as np
import unittest
unittest.TestLoader.sortTestMethodsUsing = None

class TestIntegration(unittest.TestCase):
    """Testing class for integration testing the sim"""

    def __init__(self, *args):
        """Constructor"""
        super(TestIntegration, self).__init__(*args)
        self.car_pose = None
        rospy.init_node("car_pose_tester", anonymous=True)
        rospy.Subscriber("/car/car_pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/car/scan", LaserScan, self.scan_callback)
        self.ctrl_pub = rospy.Publisher("/car/mux/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.init_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped , queue_size=1)

    def pose_callback(self, msg):
        """car_pose callback"""
        self.car_pose = msg

    def scan_callback(self, msg):
        """Scan callback. Averages values"""
        self.scan_avg = np.nanmean(msg.ranges)

    def test_a(self):
        """Test if car starts at 0, 0, 0"""
        timeout = time.time() + 5.0
        while not rospy.is_shutdown() and time.time() < timeout:
            if self.car_pose is not None:
                break
            time.sleep(0.1)

        position = self.car_pose.pose.position
        orientation = self.car_pose.pose.orientation
        position_success = False
        orientation_success = False
        if abs(position.x) < 0.001 and abs(position.y) < 0.001 and abs(position.z) < 0.001:
            position_success = True
        if abs(orientation.x) < 0.001 and abs(orientation.y) < 0.001 and abs(orientation.z) < 0.001 and abs(orientation.w) - 1.0 < 0.001 :
            orientation_success = True

        self.assert_(position_success and orientation_success)

    def test_b(self):
        """Test left control"""
        timeout = time.time() + 3.0 
        # drive slightly left
        while time.time() < timeout:
            msg = AckermannDriveStamped()
            msg.header.frame_id = "map"
            msg.drive.steering_angle = 0.2
            msg.drive.speed = 1.0
            self.ctrl_pub.publish(msg)
            
        self.assert_(self.car_pose.pose.position.x > 1.0 and self.car_pose.pose.position.y > 1.0)

        # Stop car
        timeout = time.time() + 1.0 
        while time.time() < timeout:
            msg.drive.speed = 0.0
            self.ctrl_pub.publish(msg)

    def test_c(self):
        """Test setting initial position"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 5.5
        msg.pose.pose.position.y = 3.5
        msg.pose.pose.orientation.x = 0.0 
        msg.pose.pose.orientation.y = 0.0 
        msg.pose.pose.orientation.z = 0.0 
        msg.pose.pose.orientation.w = 1.0 

        timeout = time.time() + 2.0 
        while time.time() < timeout:
            self.init_pub.publish(msg)
            time.sleep(0.1)
            
        self.assert_(round(self.car_pose.pose.position.x,2) == 5.5 \
                and round(self.car_pose.pose.position.y,2) == 3.5 \
                and round(self.car_pose.pose.orientation.x,2) == 0.0 \
                and round(self.car_pose.pose.orientation.y,2) == 0.0 \
                and round(self.car_pose.pose.orientation.z,2) == 0.0 \
                and round(self.car_pose.pose.orientation.w,2) == 1.0)

    def test_d(self):
        """Test setting initial position in occupied space"""
        car_pose = [self.car_pose.pose.position.x, self.car_pose.pose.position.y]

        # position is in a wall in sandbox
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 49.701
        msg.pose.pose.position.y = -7.689 

        timeout = time.time() + 2.0 
        while time.time() < timeout:
            self.init_pub.publish(msg)
            time.sleep(0.1)
        
        # car should not move
        self.assert_(round(self.car_pose.pose.position.x,2) == round(car_pose[0],2) \
                and round(self.car_pose.pose.position.y,2) == round(car_pose[1],2))

    def test_e(self):
        """Test if car stops moving when it hits a wall"""

        # position is near wall in sandbox
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 48.0
        msg.pose.pose.position.y = 0.0 

        timeout = time.time() + 2.0 
        while time.time() < timeout:
            self.init_pub.publish(msg)
            time.sleep(0.1)

        timeout = time.time() + 3.0 
        # drive straight 
        while time.time() < timeout:
            msg = AckermannDriveStamped()
            msg.header.frame_id = "map"
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 1.0
            self.ctrl_pub.publish(msg)

        # car should stop 
        self.assert_(round(self.car_pose.pose.position.x,1) == 49.4 \
                and round(self.car_pose.pose.position.y,1) == 0.0)

    def test_f(self):
        """Roughly test laser scan"""

        # position is freespace in sandbox
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0 

        timeout = time.time() + 2.0 
        while time.time() < timeout:
            self.init_pub.publish(msg)
            time.sleep(0.1)

        # no hits, scan avg should be high
        self.assert_(round(self.scan_avg,1) > 50.0)

        # position in corner
        msg.pose.pose.position.x = 48.0
        msg.pose.pose.position.y = 48.0 

        timeout = time.time() + 2.5 
        while time.time() < timeout:
            self.init_pub.publish(msg)
            time.sleep(0.1)

        # many close hits, scan avg should be low
        self.assert_(round(self.scan_avg,1) < 35.0)




if __name__ == "__main__":
    rostest.rosrun("mushr_sim", "test_sim", TestIntegration)
