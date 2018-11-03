#!/usr/bin/env python

import rospy
import numpy as np
import tf
import tf.transformations
import utils
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64
from threading import Lock
from nav_msgs.srv import GetMap


class SimCarPose:
    def __init__(self):
        self.SPEED_TO_ERPM_OFFSET = float(rospy.get_param("/vesc/speed_to_erpm_offset", 0.0))
        self.SPEED_TO_ERPM_GAIN = float(rospy.get_param("/vesc/speed_to_erpm_gain", 4614.0))
        self.STEERING_TO_SERVO_OFFSET = float(rospy.get_param("/vesc/steering_angle_to_servo_offset", 0.5304))
        self.STEERING_TO_SERVO_GAIN = float(rospy.get_param("/vesc/steering_angle_to_servo_gain", -1.2135))
        self.UPDATE_RATE = float(rospy.get_param("~update_rate", 20.0))
        self.SPEED_OFFSET = float(rospy.get_param("~speed_offset",0.00001))
        self.SPEED_NOISE = float(rospy.get_param("~speed_noise", 0.00001))
        self.STEERING_ANGLE_OFFSET = float(rospy.get_param("~steering_angle_offset", 0.00001))
        self.STEERING_ANGLE_NOISE = float(rospy.get_param("~steering_angle_noise", 0.00001))
        self.FORWARD_OFFSET = float(rospy.get_param("~forward_offset", 0.0))
        self.FORWARD_FIX_NOISE = float(rospy.get_param("~forward_fix_noise", 0.00001))
        self.FORWARD_SCALE_NOISE = float(rospy.get_param("~forward_scale_noise", 0.001))
        self.SIDE_OFFSET = float(rospy.get_param("~side_offset", 0.0))
        self.SIDE_FIX_NOISE = float(rospy.get_param("~side_fix_noise", 0.000001))
        self.SIDE_SCALE_NOISE = float(rospy.get_param("~side_scale_noise", 0.001))
        self.THETA_OFFSET = float(rospy.get_param("~theta_offset", 0.0))
        self.THETA_FIX_NOISE = float(rospy.get_param("~theta_fix_noise", 0.01))
        self.CAR_LENGTH = float(rospy.get_param("~car_length", 0.33))

        self.permissible_region, self.map_info = self.get_map()

        self.can_step = False

        self.last_stamp = None

        self.last_speed = None
        self.last_speed_lock = Lock()

        self.last_steering_angle = None
        self.last_steering_angle_lock = Lock()

        self.cur_pose = None
        self.cur_pose_lock = Lock()

        self.br = None

        self.br = tf.TransformBroadcaster()

        self.cur_pose_pub = rospy.Publisher("/sim_car_pose/pose", PoseStamped, queue_size=1)

        self.init_pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.init_pose_cb, queue_size=1)
        self.speed_sub = rospy.Subscriber("/vesc/sensors/core", VescStateStamped, self.speed_cb, queue_size=1)
        self.servo_sub = rospy.Subscriber("/vesc/sensors/servo_position_command", Float64, self.servo_cb, queue_size=1)
        self.update_timer = rospy.Timer(rospy.Duration.from_sec(1.0/self.UPDATE_RATE), self.timer_cb)

    def init_pose_cb(self, msg):
        rx_pose = np.array([msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            utils.quaternion_to_angle(msg.pose.pose.orientation)])
        map_rx_pose = utils.world_to_map(rx_pose, self.map_info)

        if self.permissible_region[int(map_rx_pose[1]+0.5), int(map_rx_pose[0]+0.5)] == 1:
            self.cur_pose_lock.acquire()
            self.cur_pose = np.array([rx_pose[0], rx_pose[1], rx_pose[2]])
            self.cur_pose_lock.release()
            if (not self.can_step) and (self.last_speed is not None) and (self.last_steering_angle is not None):
                self.can_step = True

    def speed_cb(self, msg):
        self.last_speed_lock.acquire()
        self.last_speed = (msg.state.speed - self.SPEED_TO_ERPM_OFFSET)/self.SPEED_TO_ERPM_GAIN
        self.last_speed_lock.release()

        if (not self.can_step) and (self.cur_pose is not None) and (self.last_steering_angle is not None):
            self.can_step = True

    def servo_cb(self, msg):
        self.last_steering_angle_lock.acquire()
        self.last_steering_angle = (msg.data - self.STEERING_TO_SERVO_OFFSET)/self.STEERING_TO_SERVO_GAIN
        self.last_steering_angle_lock.release()
        if (not self.can_step) and (self.cur_pose is not None) and (self.last_speed is not None):
            self.can_step = True

    def timer_cb(self, event):
        if not self.can_step:
            return

        now = rospy.Time.now()
        if self.last_stamp is None:
            self.last_stamp = now
        dt = (now-self.last_stamp).to_sec()

        self.last_speed_lock.acquire()
        v = self.last_speed + np.random.normal(loc=self.SPEED_OFFSET*self.last_speed, scale=self.SPEED_NOISE, size=1)
        self.last_speed_lock.release()

        self.last_steering_angle_lock.acquire()
        delta = self.last_steering_angle + np.random.normal(loc=self.STEERING_ANGLE_OFFSET*self.last_steering_angle, scale=self.STEERING_ANGLE_NOISE, size=1)
        self.last_steering_angle_lock.release()

        self.cur_pose_lock.acquire()
        new_pose = np.array(self.cur_pose)
        if np.abs(delta) < 1e-2:
            dx = v * np.cos(self.cur_pose[2]) * dt
            dy = v * np.sin(self.cur_pose[2]) * dt
            dtheta = 0
        else:
            beta = np.arctan(0.5 * np.tan(delta))
            sin2beta = np.sin(2 * beta)
            dtheta = ((v / self.CAR_LENGTH) * sin2beta) * dt
            dx = (self.CAR_LENGTH / sin2beta) * (np.sin(self.cur_pose[2] + dtheta) - np.sin(self.cur_pose[2]))
            dy = (self.CAR_LENGTH / sin2beta) * (-1 * np.cos(self.cur_pose[2] + dtheta) + np.cos(self.cur_pose[2]))


        new_pose[0] += dx + np.random.normal(loc=self.FORWARD_OFFSET, scale=self.FORWARD_FIX_NOISE, size=1) + np.random.normal(loc=0.0, scale=np.abs(v)*self.FORWARD_SCALE_NOISE, size=1)
        new_pose[1] += dy + np.random.normal(loc=self.SIDE_OFFSET, scale=self.SIDE_FIX_NOISE, size=1) + np.random.normal(loc=0.0, scale=np.abs(v) * self.SIDE_SCALE_NOISE, size=1)
        new_pose[2] += dtheta + np.random.normal(loc=self.THETA_OFFSET, scale=self.THETA_FIX_NOISE, size=1)

        while new_pose[2] > 2*np.pi:
            new_pose[2] -= 2*np.pi

        while new_pose[2] < 2*np.pi:
            new_pose[2] += 2*np.pi

        new_map_pose = utils.world_to_map(new_pose, self.map_info)

        # Only update cur_pose if new pose is in bounds
        if self.permissible_region[int(new_map_pose[1]+0.5), int(new_map_pose[0]+0.5)] == 1:

            self.cur_pose = np.array(new_pose)

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = now
        ps.pose.position.x = self.cur_pose[0]
        ps.pose.position.y = self.cur_pose[1]
        ps.pose.position.z = 0.0
        ps.pose.orientation = utils.angle_to_quaternion(self.cur_pose[2])

        self.cur_pose_pub.publish(ps)

        self.last_stamp = now

        self.br.sendTransform((ps.pose.position.x, ps.pose.position.y, ps.pose.position.z),
                               tf.transformations.quaternion_from_euler(0, 0, self.cur_pose[2]),
                                now, "sim_pose", "map")
        self.cur_pose_lock.release()

    def get_map(self):
        # Use the 'static_map' service (launched by MapServer.launch) to get the map
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        map_info = map_msg.info  # Save info about map for later use

        # Create numpy array representing map for later use
        array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        permissible_region = np.zeros_like(array_255, dtype=bool)
        permissible_region[array_255 == 0] = 1  # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
                                                # With values 0: not permissible, 1: permissible
        return permissible_region, map_info


if __name__ == '__main__':
    rospy.init_node('sim_car_pose_node')

    scp = SimCarPose()

    rospy.spin()
