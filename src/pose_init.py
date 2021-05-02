#!/usr/bin/env python

import rospy
import tf
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import math as m
import time
import random

rospy.init_node("pose_initializer")

count = 0;

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

def pose_callback(data,args):
	global count
	if(count<args):
		count = args

# list of publishers and subscribers
pub = []
goal_pub = []
sub = []
# this is basically initializing all the subscribers for counting the number of cars and publishers for initiailizing pose and goal points.
for i in range(8):
	subscriber = rospy.Subscriber("/car" + str(i+1) + "/car_pose", PoseStamped, pose_callback,(i+1))
	publisher = rospy.Publisher("/car" + str(i+1) + "/initialpose", PoseWithCovarianceStamped, queue_size=1)
	goal_publisher = rospy.Publisher("/car" + str(i+1) + "/move_base_simple/goal", PoseStamped, queue_size=1)
	sub.append(subscriber)
	pub.append(publisher)
	goal_pub.append(goal_publisher)


time.sleep(5) # give some time for the code to initialize.

# initial pose setting -> not required in real; comment if not required.
now = rospy.Time.now()
cur_pose = PoseWithCovarianceStamped()
cur_pose.header.frame_id = "/map"
cur_pose.header.stamp = now

# sets the cars on the circumference of a circle with radius = R. the positions are equi-distant (3 cars at 120 degrees, 4 at 90 and so on)
R = 2.5
if(count==1): # just one agent fellas.
	R = 0
# print(count)
for i in range(count):
	fraction = float(i)/float(count)
	angle = 2*m.pi*fraction
	cur_pose.pose.pose.position.x = R*m.cos(angle) + R
	cur_pose.pose.pose.position.y = 0.6*R*m.sin(angle) - 0.6*R
	cur_pose.pose.pose.position.z = 0.0
	if(count==1):
		rot = 0
	else:
		rot = angle - m.pi
	#wrap around
	if(rot>2*m.pi):
		rot -= 2*m.pi
	if(rot< -2*m.pi):
		rot += 2*m.pi
	cur_pose.pose.pose.orientation = angle_to_quaternion(rot)
	pub[i].publish(cur_pose)
	time.sleep(1.1) # give the simulator some time bruh.


# goal setting: sets the position of the goal points. goal points are diametrically opposite to the car's starting position.
now = rospy.Time.now()
goal_pose = PoseStamped()
goal_pose.header.frame_id = "/map"
goal_pose.header.stamp = now
for i in range(count):
	fraction = float(i)/float(count)
	angle = m.pi + 2*m.pi*fraction
	goal_pose.pose.position.x = R*m.cos(angle)
	goal_pose.pose.position.y = R*m.sin(angle)
	goal_pose.pose.position.z = 0.0
	rot = angle
	#wrap around
	if(rot>2*m.pi):
		rot -= 2*m.pi
	if(rot< -2*m.pi):
		rot += 2*m.pi
	goal_pose.pose.orientation = angle_to_quaternion(rot)
	# goal_pub[i].publish(goal_pose)
