#!/usr/bin/env python

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan

import range_libc
import utils


class FakeURGNode:
    def __init__(self):
        self.UPDATE_RATE = float(rospy.get_param("~update_rate", 10.0))
        self.THETA_DISCRETIZATION = float(rospy.get_param("~theta_discretization", 656))
        self.MIN_RANGE_METERS = float(rospy.get_param("~min_range_meters", 0.02))
        self.MAX_RANGE_METERS = float(rospy.get_param("~max_range_meters", 5.6))
        self.ANGLE_STEP = float(rospy.get_param("~angle_step", 0.00613592332229))
        self.ANGLE_MIN = float(rospy.get_param("~angle_min", -2.08621382713))
        self.ANGLE_MAX = float(rospy.get_param("~angle_max", 2.09234976768))
        self.ANGLES = np.arange(
            self.ANGLE_MIN, self.ANGLE_MAX, self.ANGLE_STEP, dtype=np.float32
        )
        self.CAR_LENGTH = float(rospy.get_param("~car_length", 0.33))
        self.Z_SHORT = float(rospy.get_param("~z_short", 0.03))
        self.Z_MAX = float(rospy.get_param("~z_max", 0.16))
        self.Z_BLACKOUT_MAX = float(rospy.get_param("~z_blackout_max", 50))
        self.Z_RAND = float(rospy.get_param("~z_rand", 0.01))
        self.Z_HIT = float(rospy.get_param("~z_hit", 0.8))
        self.Z_SIGMA = float(rospy.get_param("~z_sigma", 0.03))

        map_msg = self.get_map()
        occ_map = range_libc.PyOMap(map_msg)
        max_range_px = int(self.MAX_RANGE_METERS / map_msg.info.resolution)
        self.range_method = range_libc.PyCDDTCast(
            occ_map, max_range_px, self.THETA_DISCRETIZATION
        )

        self.tl = tf.TransformListener()
        now = rospy.Time.now()
        self.tl.waitForTransform("base_link", "laser_link", now, rospy.Duration(5.0))
        position, orientation = self.tl.lookupTransform("base_link", "laser_link", now)
        self.x_offset = -1 * self.CAR_LENGTH / 2.0 + position[0]

        self.laser_pub = rospy.Publisher("/scan", LaserScan, queue_size=1)

        # Hack to wait for tfs to be available. self.tl.frame_exists doesn't sem to work
        rospy.wait_for_message("/car_pose", PoseStamped)

        self.update_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.UPDATE_RATE), self.timer_cb
        )

    def noise_laser_scan(self, ranges):
        indices = np.zeros(ranges.shape[0], dtype=np.int)
        prob_sum = self.Z_HIT + self.Z_RAND + self.Z_SHORT
        hit_count = int((self.Z_HIT / prob_sum) * indices.shape[0])
        rand_count = int((self.Z_RAND / prob_sum) * indices.shape[0])
        short_count = indices.shape[0] - hit_count - rand_count
        indices[hit_count : hit_count + rand_count] = 1
        indices[hit_count + rand_count :] = 2
        np.random.shuffle(indices)

        hit_indices = indices == 0
        ranges[hit_indices] += np.random.normal(
            loc=0.0, scale=self.Z_SIGMA, size=hit_count
        )[:]

        rand_indices = indices == 1
        ranges[rand_indices] = np.random.uniform(
            low=self.MIN_RANGE_METERS, high=self.MAX_RANGE_METERS, size=rand_count
        )[:]

        short_indices = indices == 2
        ranges[short_indices] = np.random.uniform(
            low=self.MIN_RANGE_METERS, high=ranges[short_indices], size=short_count
        )[:]

        max_count = (self.Z_MAX / (prob_sum + self.Z_MAX)) * ranges.shape[0]
        while max_count > 0:
            cur = np.random.randint(low=0, high=ranges.shape[0], size=1)
            blackout_count = np.random.randint(low=1, high=self.Z_BLACKOUT_MAX, size=1)
            while (
                cur > 0
                and cur < ranges.shape[0]
                and blackout_count > 0
                and max_count > 0
            ):
                if not np.isnan(ranges[cur]):
                    ranges[cur] = np.nan
                    cur += 1
                    blackout_count -= 1
                    max_count -= 1
                else:
                    break

    def timer_cb(self, event):

        now = rospy.Time.now()
        ls = LaserScan()
        ls.header.frame_id = "laser_link"
        ls.header.stamp = now
        ls.angle_increment = self.ANGLE_STEP
        ls.angle_min = self.ANGLE_MIN
        ls.angle_max = self.ANGLE_MAX
        ls.range_min = self.MIN_RANGE_METERS
        ls.range_max = self.MAX_RANGE_METERS
        ls.intensities = []

        ranges = np.zeros(len(self.ANGLES) * 1, dtype=np.float32)

        ps1 = PoseStamped()
        ps1.header.frame_id = "base_link"
        ps1.header.stamp = rospy.Time(0)
        ps1.pose.position.x = self.x_offset
        ps1.pose.position.y = 0.0
        ps1.pose.position.z = 0.0
        ps1.pose.orientation.x = 0.0
        ps1.pose.orientation.y = 0.0
        ps1.pose.orientation.z = 0.0
        ps1.pose.orientation.w = 1.0

        laser_pose = self.tl.transformPose("map", ps1)
        range_pose = np.array(
            (
                laser_pose.pose.position.x,
                laser_pose.pose.position.y,
                utils.quaternion_to_angle(laser_pose.pose.orientation),
            ),
            dtype=np.float32,
        ).reshape(1, 3)
        self.range_method.calc_range_repeat_angles(range_pose, self.ANGLES, ranges)
        self.noise_laser_scan(ranges)
        ls.ranges = ranges.tolist()
        self.laser_pub.publish(ls)

    def get_map(self):
        # Use the 'static_map' service (launched by MapServer.launch) to get the map
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        return map_msg


if __name__ == "__main__":
    rospy.init_node("fake_urg_node")

    furgn = FakeURGNode()

    rospy.spin()
