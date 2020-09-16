#!/usr/bin/python
import argparse
import os
import rospy
from std_msgs.msg import Float32, Int64, Header
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs.msg import Image as ImageMsg
from nav_msgs.msg import Odometry
import rospkg
import numpy as np
from cv_bridge import CvBridge
import tf
from gibson2.envs.locomotor_env import NavigateEnv, NavigateRandomEnv
from gibson2.utils.utils import parse_config
import matplotlib.pyplot as plt
from utils import angle_to_quaternion

class SimNode:
    def __init__(self):
        rospy.init_node('gibson2_sim')
        rospack = rospkg.RosPack()
        path = rospack.get_path('mushr_sim')
        config_filename = os.path.join(path, 'config/gibson_sim.yaml')

        self.position = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]

        self.cmdx = 0.0
        self.cmdy = 0.0

        self.image_pub = rospy.Publisher("/gibson_ros/camera/rgb/image", ImageMsg, queue_size=10)
        self.depth_pub = rospy.Publisher("/gibson_ros/camera/depth/image", ImageMsg, queue_size=10)
        self.lidar_pub = rospy.Publisher("/gibson_ros/lidar/points", PointCloud2, queue_size=10)

        self.depth_raw_pub = rospy.Publisher("/gibson_ros/camera/depth/image_raw",
                                             ImageMsg,
                                             queue_size=10)

        self.camera_info_pub = rospy.Publisher("/gibson_ros/camera/depth/camera_info",
                                               CameraInfo,
                                               queue_size=10)
        self.bridge = CvBridge()
        self.br = tf.TransformBroadcaster()

        model_id = rospy.get_param("model_id")
        if model_id:
            self.env = NavigateEnv(config_file=config_filename,
                                mode='headless',
                                model_id=model_id,
                                action_timestep=1 / 30.0)    # assume a 30Hz simulation
        else: 
            self.env = NavigateEnv(config_file=config_filename,
                                mode='headless',
                                action_timestep=1 / 30.0) 

        obs = self.env.reset()
        rospy.Subscriber("/car/car_pose", PoseStamped, self.tp_robot_callback)

        self.tp_time = None

    def run(self):
        while not rospy.is_shutdown():
            self.env.robots[0].set_position_orientation(self.position, self.orientation)
            obs, _, _, _ = self.env.step([self.cmdx, self.cmdy])
            rgb = (obs["rgb"] * 255).astype(np.uint8)
            depth = obs["depth"].astype(np.float32)
            image_message = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
            depth_raw_image = (obs["depth"] * 1000).astype(np.uint16)
            depth_raw_message = self.bridge.cv2_to_imgmsg(depth_raw_image, encoding="passthrough")
            depth_message = self.bridge.cv2_to_imgmsg(depth, encoding="passthrough")

            now = rospy.Time.now()

            image_message.header.stamp = now
            depth_message.header.stamp = now
            depth_raw_message.header.stamp = now
            image_message.header.frame_id = "/car/camera_depth_frame" #camera_depth_optical_frame
            depth_message.header.frame_id = "/car/camera_depth_frame" #camera_depth_optical_frame
            depth_raw_message.header.frame_id = "/car/camera_depth_frame" #camera_depth_optical_frame

            self.image_pub.publish(image_message)
            self.depth_pub.publish(depth_message)
            self.depth_raw_pub.publish(depth_raw_message)
            msg = CameraInfo(height=1024,
                             width=1024,
                             distortion_model="plumb_bob",
                             D=[0.0, 0.0, 0.0, 0.0, 0.0],
                             K=[640, 0.0, 640, 0.0, 640, 640, 0.0, 0.0, 1.0],
                             R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                             P=[640, 0.0, 640, 0.0, 0.0, 640, 640, 0.0, 0.0, 0.0, 1.0, 0.0])
            msg.header.stamp = now
            msg.header.frame_id = "/car/camera_depth_frame" #camera_depth_optical_frame
            self.camera_info_pub.publish(msg)

            lidar_points = obs['scan']
            points = np.array(lidar_points).tolist()
            lidar_header = Header()
            lidar_header.stamp = now
            lidar_header.frame_id = 'car/laser_link' #'scan_link'
            lidar_message = pc2.create_cloud_xyz32(lidar_header, points)
            self.lidar_pub.publish(lidar_message)

    def tp_robot_callback(self, data):
        position = [round(data.pose.position.x, 5), round(data.pose.position.y, 5), round(data.pose.position.z, 5)]
        orientation = [
            data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z,
            data.pose.orientation.w
        ]
        self.position = position
        self.orientation = orientation
        self.env.robots[0].set_position_orientation(position, orientation)


if __name__ == '__main__':
    node = SimNode()
    node.run()
