#!/usr/bin/env python

"""
Converts FoxGlove ROS messages into a format compatible with the MuSHR stack.
Author: Schiffer
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped

class NavMsgConverter:
  def __init__(self) -> None:
    """
    Initialize Navigation Messages Converter.
    Attributes:
        name (string) rosnode name
    """
    self.pose = None
    self.type = None
    self.last_point = None

    # Setup params
    # config_file = open(rospy.get_param("~config_path"))
    # params = yaml.load(config_file)

    # Create the subscribers
    self.point_sub = rospy.Subscriber(
      rospy.get_param("~point_topic"), PointStamped, self.point_to_pose, queue_size=100
    )
    self.type_sub = rospy.Subscriber(
      rospy.get_param("~type_topic"), String, self.save_type, queue_size=100
    )

    # Create the publishers
    self.goal_pub = rospy.Publisher(rospy.get_param("~goal_topic"), PoseStamped, queue_size=1)
    self.sim_car_pose_pub = rospy.Publisher(rospy.get_param("~start_topic"), PoseStamped, queue_size=1)    

  def point_to_pose(self, point_msg: PointStamped) -> None:
    """
    Take a point stamped message and convert it into a pose stamped message.
    """
    if self.last_point == point_msg.point:
      return
    self.last_point = point_msg.point
    as_pose = PoseStamped(header=point_msg.header)
    as_pose.pose.position = point_msg.point
    as_pose.pose.orientation.w = 1
    self.pose = as_pose
  
  def save_type(self, type_msg: String) -> None:
    self.type = type_msg.data
    if self.type != 'pose' and self.type != 'goal':
      raise Exception(f'Invalid type detected {self.type}')
    if self.pose is not None:
        self.publish()
        self.pose = None
        self.type = None
  
  def publish(self) -> None:
    if self.type == 'pose':
      self.sim_car_pose_pub.publish(self.pose)
    else:
      self.goal_pub.publish(self.pose)
