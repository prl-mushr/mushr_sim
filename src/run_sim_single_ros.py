#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
import json
import time
import numpy as np
from sim_client import SDClient
import math as m
import traceback # this is just for seeing the cause of the error if one is thrown during runtime without stopping the code
###########################################

def angle_to_quaternion(angle):
    """
    Convert an angle in radians into a quaternion _message_.

    Params:
        angle in radians
    Returns:
        quaternion (unit quaternion)
    """
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

def quaternion_to_angle(q):
    """
    
    Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion.

    This function was finessed from the mushr_base/src/utils.py
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return roll, pitch, yaw

class SimpleClient(SDClient):
    """
    A class to create an interface with the unity game/sim
    """
    def __init__(self, address, poll_socket_sleep_time=0.001):
        """
        init function. 

        Params:
            address (host,port)
            arguments related to ...something. Not sure where I use them, but I'm sure it would break if I didn't have this.
            poll_socket_sleep time: sleep time for the polling system. Keep as low as possible
        
        Returns:
            None
        """
        super(SimpleClient,self).__init__(*address, poll_socket_sleep_time=poll_socket_sleep_time)
        self.last_image = None
        self.car_loaded = False
        self.now = time.time()
        self.pose = np.zeros(7)
        self.twist = np.zeros(6)
        self.accel = np.zeros(3)
        self.speed = 0
        self.steering_angle = 0
        self.heading = 0
        self.time_stamp = time.time()
        self.throttle_failsafe = time.time()

    def on_msg_recv(self, json_packet):
        """
        Telemetry receiving callback.

        Called when unity sim publishes the car's state.

        Params: 
            json packet. This callback is called by a function in a different file, so you won't see where this function is actually "called"
        
        Returns:
            None
        """
        if json_packet['msg_type'] == "car_loaded":
            self.car_loaded = True
        
        if json_packet['msg_type'] == "telemetry":
            self.time_stamp = time.time() # this time stamp is useful for time-sensitive control algorithms.
            self.pose[:3] = np.array([ json_packet["pos_x"], json_packet["pos_y"], json_packet["pos_z"] ])
            self.pose[:2] -= 50.0 # the sim needs a 50x50 offset because starting at 0,0 is the equivalent of starting from the edge and can throw weird bugs..
            self.pose[3:] = np.array([ json_packet["quat_x"],json_packet["quat_y"], json_packet["quat_z"], json_packet["quat_w"]])
            roll, pitch, yaw = tf.transformations.euler_from_quaternion((self.pose[3], self.pose[4] , self.pose[5], self.pose[6]))
            print(roll*57.3,pitch*57.3,yaw*57.3)
            self.twist[:3] = np.array([ json_packet["vel_x"],json_packet["vel_y"], json_packet["vel_z"]])
            self.twist[3:] = np.array([ json_packet["gyro_x"], json_packet["gyro_z"], json_packet["gyro_y"]])
            self.accel = np.array([json_packet["Ax"],json_packet["Ay"],json_packet["Az"]])
            self.heading = json_packet["heading"]
            self.speed = json_packet["speed"]
            self.steering_angle = json_packet["steering_angle"]

            dt = time.time() - self.now # in case you wanna measure the fps of the sim engine.
            self.now = time.time()

    def send_controls(self, steering, speed):
        """
        Send controls to the unity sim engine

        Params:
            scaled steering angle (-1,1), absolute speed (0-12 m/s)
        """
        p = { "msg_type" : "control",
                "steering" : steering.__str__(),
                "throttle" : speed.__str__(),
                "brake" : "0.0" }
        msg = json.dumps(p)
        self.send(msg)
        #this sleep lets the SDClient thread poll our message and send it out.
        time.sleep(self.poll_socket_sleep_sec)
    
class RacecarState:
    """
    __init__: Initialize params, publishers, subscribers, and timer
    """

    def __init__(self):
        self.create_client()
        self.X_offset = float(rospy.get_param("~X_offset",0.0))
        self.Y_offset = float(rospy.get_param("~Y_offset",0.0))
        self.angle_offset = float(rospy.get_param("~angle_offset",0.0))
        # Length of the car
        self.CAR_LENGTH = float(rospy.get_param("vesc/chassis_length", 0.33))

        # Width of the car
        self.CAR_WIDTH = float(rospy.get_param("vesc/wheelbase", 0.29))

        # The radius of the car wheel in meters
        self.CAR_WHEEL_RADIUS = 0.0976 / 2.0

        # Rate at which to publish joints and tf
        self.UPDATE_RATE = float(rospy.get_param("~update_rate", 50.0))

        # Append this prefix to any broadcasted TFs
        self.TF_PREFIX = str(rospy.get_param("~tf_prefix", "").rstrip("/"))
        if len(self.TF_PREFIX) > 0:
            self.TF_PREFIX = self.TF_PREFIX + "/"
        # The most recent time stamp
        self.last_stamp = None

        # The most recent transform from odom to base_footprint
        self.cur_odom_to_base_trans = np.array([0, 0], dtype=np.float)
        self.cur_odom_to_base_rot = 0.0

        # The most recent transform from the map to odom
        self.cur_map_to_odom_trans = np.array([0,0], dtype=np.float)
        self.cur_map_to_odom_rot = 0.0

        # Message used to publish joint values
        self.joint_msg = JointState()
        self.joint_msg.name = [
            "front_left_wheel_throttle",
            "front_right_wheel_throttle",
            "back_left_wheel_throttle",
            "back_right_wheel_throttle",
            "front_left_wheel_steer",
            "front_right_wheel_steer",
        ]
        self.joint_msg.position = [0, 0, 0, 0, 0, 0]
        self.joint_msg.velocity = []
        self.joint_msg.effort = []

        # Publishes joint messages
        self.br = tf.TransformBroadcaster()

        # Duration param controls how often to publish default map to odom tf
        # if no other nodes are publishing it
        self.transformer = tf.TransformListener()

        # Publishes joint values
        self.state_publisher = rospy.Publisher("car_pose", PoseStamped, queue_size=1)
        self.odom_publisher = rospy.Publisher("car_odom", Odometry, queue_size=10)
        # Publishes joint values
        self.cur_joints_pub = rospy.Publisher("joint_states", JointState, queue_size=1)

        self.imu_publisher = rospy.Publisher("imu", Imu, queue_size = 10)

        # Subscribes to the initial pose of the car
        self.init_pose_sub = rospy.Subscriber(
            "initialpose", PoseWithCovarianceStamped, self.init_pose_cb, queue_size=1
        )
        
        self.cur_odom = Odometry()
        self.cur_odom.header.frame_id = "/map"
        self.cur_pose = PoseStamped()
        self.cur_pose.header.frame_id = "/map"
        self.cur_joint = JointState()
        self.cur_joint.position = [0, 0, 0, 0, 0, 0]
        self.cur_joint.velocity = []
        self.cur_joint.effort = []
        self.cur_joint.name = [
            "front_left_wheel_throttle",
            "front_right_wheel_throttle",
            "back_left_wheel_throttle",
            "back_right_wheel_throttle",
            "front_left_wheel_steer",
            "front_right_wheel_steer",
        ]
        self.cur_imu = Imu()
        self.cur_imu.header.frame_id = "/base_link"

        # Timer to updates joints and tf
        self.update_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.UPDATE_RATE), self.timer_cb
        )
        
        control_subscriber = rospy.Subscriber("mux/ackermann_cmd_mux/output", AckermannDriveStamped, self.input_callback )


    def create_client(self):
        """
        Create clients that connect to the sim engine.
        Params:
            None (maybe initial position at some point)
        Returns:
            None
        """
        host = "127.0.0.1" # local host
        port = 9091

        # Start Clients
        self.client = SimpleClient(address=(host, port),poll_socket_sleep_time=0.001)
        msg = '{ "msg_type" : "load_scene", "scene_name" : "generated_road" }'
        self.client.send(msg)
        time.sleep(1)# Wait briefly for the scene to load.         
        # Car config: this is just for initialization. The position will be reset to 50,50 when the car loads
        msg = '{ "msg_type" : "car_config", "body_style" : "mushr", "body_r" : "0", "body_g" : "0", "body_b" : "255", "car_name" : "MUSHR", "font_size" : "100", "start_X" : "0.00", "start_Y" : "0.00", "start_Z" : "0.20", "yaw" : "0.0" }' # do not change
        self.client.send(msg)
        time.sleep(1)

        loaded = False
        while(not loaded):
            time.sleep(1.0)
            loaded = self.client.car_loaded  
            print("loading")

    def input_callback(self,data):
        """
        ROS-side callback for getting the car control inputs.

        Params:
            data (AckermannDriveStamped)
        
        Returns:
            None
        """
        self.client.throttle_failsafe = time.time()
        speed = data.drive.speed 
        steering = -data.drive.steering_angle*57.3/25 # scaling required before sending it.
        self.client.send_controls(steering,speed) # gotta normalize it

    def init_pose_cb(self,data):
        """
        initial pose setting callback.

        Params:
            initial pose (PoseStampedWithCovariance)

        Returns:
            None
        """
        X,Y = data.pose.pose.position.x + 50 ,data.pose.pose.position.y + 50 # we need a 50x50 offset in position because starting at 0,0 is like starting at the edge of the map and the car gets stuck/shows weird behavior
        roll, pitch, yaw = quaternion_to_angle(data.pose.pose.orientation)
        msg = '{ "msg_type" : "car_config", "body_style" : "mushr", "body_r" : "0", "body_g" : "0", "body_b" : "255", "car_name" : "MUSHR", "font_size" : "100", "start_X" : "' + str(round(X, 3)) + '", "start_Y" : "' + str(round(Y, 3)) + '", "start_Z" : "0.20", "yaw" : "' + str( round(yaw-m.pi/2, 3) ) + '" }' # do not change
        self.client.send(msg)
        time.sleep(1)

    def timer_cb(self,event):
        """
        timer callback: called at every update time step to get the state update from the simulator. simulator Max update rate is ~60 Hz
        
        Params:
            event
        Returns:
            None
        """
        try:
            now = rospy.Time.now() # technically one could use the client.time_stamp to be more accurate but I have a feeling it might lag behind by 1/60th of a second from this so idk
            self.cur_odom.header.stamp = now
            self.cur_odom.pose.pose.position.x = self.client.pose[0]
            self.cur_odom.pose.pose.position.y = self.client.pose[1]
            self.cur_odom.pose.pose.position.z = self.client.pose[2]
            rot = self.client.heading
            #wrap around
            if(rot>2*m.pi):
                rot -= 2*m.pi
            if(rot< -2*m.pi):
                rot += 2*m.pi
            self.cur_odom.pose.pose.orientation = angle_to_quaternion(rot)
            self.cur_odom.twist.twist.linear.x = self.client.twist[0]
            self.cur_odom.twist.twist.linear.y = self.client.twist[1]
            self.cur_odom.twist.twist.linear.z = self.client.twist[2]
            self.cur_odom.twist.twist.angular.x = self.client.twist[3]
            self.cur_odom.twist.twist.angular.y = self.client.twist[4]
            self.cur_odom.twist.twist.angular.z = self.client.twist[5]
            self.odom_publisher.publish(self.cur_odom)

            self.cur_pose.header.stamp = now
            self.cur_pose.pose.position.x = self.client.pose[0]
            self.cur_pose.pose.position.y = self.client.pose[1]
            self.cur_pose.pose.position.z = self.client.pose[2]
            rot = self.client.heading
            #wrap around
            if(rot>2*m.pi):
                rot -= 2*m.pi
            if(rot< -2*m.pi):
                rot += 2*m.pi
            self.cur_pose.pose.orientation = angle_to_quaternion(rot)
            
            self.br.sendTransform(
            (0,0,0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            now,
            self.TF_PREFIX + "odom",
            "/map",
            )  
            self.br.sendTransform(
            (self.client.pose[0],self.client.pose[1],self.client.pose[2]),
            tf.transformations.quaternion_from_euler(0, 0, rot),
            now,
            self.TF_PREFIX + "base_footprint",
            self.TF_PREFIX + "odom",
            )  
            self.state_publisher.publish(self.cur_pose)

            self.cur_joint.header.stamp = now
            #TODO: fix this
            delta = -self.client.steering_angle*(25.0/57.3)
            
            if np.abs(delta) < 1e-2:
                # New joint values
                joint_left_throttle = self.client.speed * (1.0/self.UPDATE_RATE) / self.CAR_WHEEL_RADIUS
                joint_right_throttle = self.client.speed * (1.0/self.UPDATE_RATE) / self.CAR_WHEEL_RADIUS
                joint_left_steer = 0.0
                joint_right_steer = 0.0
            else:    
                tan_delta = m.tan(delta)
                h_val = (self.CAR_LENGTH / tan_delta) - (self.CAR_WIDTH / 2.0)
                joint_outer_throttle = (
                    ((self.CAR_WIDTH + h_val) / (0.5 * self.CAR_WIDTH + h_val))
                    * self.client.speed
                    * (1.0/self.UPDATE_RATE)
                    / self.CAR_WHEEL_RADIUS
                )
                joint_inner_throttle = (
                    ((h_val) / (0.5 * self.CAR_WIDTH + h_val))
                    * self.client.speed
                    * (1.0/self.UPDATE_RATE)
                    / self.CAR_WHEEL_RADIUS
                )
                joint_outer_steer = m.atan2(self.CAR_LENGTH, self.CAR_WIDTH + h_val)
                joint_inner_steer = m.atan2(self.CAR_LENGTH, h_val)

                # Assign joint values according to whether we are turning left or right
                if (delta) > 0.0:
                    joint_left_throttle = joint_inner_throttle
                    joint_right_throttle = joint_outer_throttle
                    joint_left_steer = joint_inner_steer
                    joint_right_steer = joint_outer_steer

                else:
                    joint_left_throttle = joint_outer_throttle
                    joint_right_throttle = joint_inner_throttle
                    joint_left_steer = joint_outer_steer - m.pi
                    joint_right_steer = joint_inner_steer - m.pi

            self.cur_joint.position[0] += joint_left_throttle
            self.cur_joint.position[1] += joint_right_throttle
            self.cur_joint.position[2] += joint_left_throttle
            self.cur_joint.position[3] += joint_right_throttle
            self.cur_joint.position[4] = joint_left_steer
            self.cur_joint.position[5] = joint_right_steer
            self.cur_joints_pub.publish(self.cur_joint)

            self.cur_imu.header.stamp = now
            self.cur_imu.orientation.x = self.client.pose[3]
            self.cur_imu.orientation.y = self.client.pose[4]
            self.cur_imu.orientation.z = self.client.pose[5]
            self.cur_imu.orientation.w = self.client.pose[6]

            self.cur_imu.angular_velocity.x = self.client.twist[3]
            self.cur_imu.angular_velocity.y = self.client.twist[4]
            self.cur_imu.angular_velocity.z = self.client.twist[5]

            self.cur_imu.linear_acceleration.x = self.client.accel[0]
            self.cur_imu.linear_acceleration.y = self.client.accel[1]
            self.cur_imu.linear_acceleration.z = self.client.accel[2]

            self.imu_publisher.publish(self.cur_imu)
            
            ## failsafe mimicking
            if(time.time() - self.client.throttle_failsafe > 1):
                self.client.send_controls(0,0)

            if self.client.aborted:
                print("self.Client socket problem, stopping driving.")
                do_drive = False

        except KeyboardInterrupt:
            msg = '{ "msg_type" : "exit_scene" }'
            self.client.send(msg)

            time.sleep(1.0)

            # Close down client
            print("waiting for msg loop to stop")
            self.client.stop()

            print("stopped")
            exit()

        except Exception as e:
            print(traceback.format_exc())

        except:
            pass



if __name__ == "__main__":
    rospy.init_node("connector")
    rs = RacecarState()
    rospy.spin()