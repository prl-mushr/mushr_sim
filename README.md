[![Build Status](https://dev.azure.com/prl-mushr/mushr_sim/_apis/build/status/prl-mushr.mushr_sim?branchName=master)](https://dev.azure.com/prl-mushr/mushr_sim/_build/latest?definitionId=5&branchName=master)

# MuSHR Simulator
The MuSHR simulator is the easiest way to get started with MuSHR. The simulated car can be either controlled via keyboard teleoperation, or by a control algorithm. This simulator is designed to emulate the car's configuration as close as possible to reduce sim-to-real overhead. To do so, many of the hardware components are programmatically simulated (camera is not simulated).

### Tutorial
To install/run the simulator see the following [tutorial](https://prl-mushr.github.io/tutorials/quickstart/).

### API
For adjusting params see `config` it has the simulated vesc params and also the sensor params for each car. If you don't find the publishers or subscribers you were looking for, they are likely in [mushr_base](https://github.com/prl-mushr/mushr_base).

#### Publishers
Topic | Type | Description
------|------|------------
`/mux/ackermann_cmd_mux/input/teleop`| [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) | Publish teleop controls from keyboard
`/map` | [nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html) | Map from map server
`/map_metadata` | [nav_msgs/MapMetaData](http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html) | Map metadata
`/car/scan` | [sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) | Simulated laser scan topic

#### Subscribers
Topic | Type | Description
------|------|------------
`/tf` | [tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html) | Transforms for the laserscan
`/tf_static` | [tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html) | Static transforms for the laserscan
`/car/mux/ackermann_cmd_mux/input/teleop`| [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) | Publish teleop controls from keyboard
`/car/mux/ackermann_cmd_mux/input/navigation` | [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) | Programatic controller input control


#### NEW: unity physics engine for simulating car motion (optional):

#### User Prerequisites:
1) Familiarity with ROS, python.
2) Completed the MuSHR [quickstart tutorial](https://mushr.io/tutorials/quickstart/) up to the simulation part.

### Installation:

python dependencies: 
1) numpy
``` bash
$ pip install numpy
```

### Running the base example:
terminal command:
``` bash
$ cd ~/catkin_ws/src/mushr_sim/unity
$ ./donkey_sim.x86_64 -batchmode
```
If you have nvidia driver support on your linux machine (god bless), you can run it without the "-batchmode" tag. The tag makes the sim run in the headless mode which allows for higher fps if you don't have said driver support.

In a new tab:
``` bash
$ roslaunch mushr_sim unity_multi.launch
```
In another new tab:
``` bash
$ rosrun rviz rviz
```
To visualize the simulation in rviz, use the rviz config in 
``` bash
$ ~/mushr_sim/rviz/unity_backend.rviz
```
You should see 4 cars by default. The poses of these cars are set by the pose_init.py file. The car's initial pose is set the same way as done for the default mushr_sim; by publishing a message on the /car_name/initialpose topic. 

if you want to have only one car, you can launch unity_single.launch instead:
``` bash
$ roslaunch mushr_sim unity_single.launch
```
change the robot description topic from `/car1/robot_description` to `/robot_description`

As it has the same interface as the default mushr_sim multi_teleop.launch, you should be able to drive the cars with the WASD keys.

Note that collisions between the cars will also be simulated (try not to collide though. The purpose was simply to make the simulation somewhat interesting). 

### API
When using namespace in the launch file (as done in unity_multi.launch) the topics will have a prefix attached to them corresponding to the name given to the car. We're calling that name "car_name" for the sake of explanation.

For using the backend in your own launch file, replace the default single_car.launch with unity_single.launch as follows:

This is the default:
{{ < highlight xml >}}
    <group ns="$(arg car1_name)">
         <include file="$(find mushr_sim)/launch/single_car.launch" >
            <arg name="car_name" value="$(arg car1_name)"/>
            <arg name="racecar_version" value="racecar-uw-nano"/>
            <arg name="racecar_color" value="" />
        </include>
    </group>
{{ </ highlight >}}

This is what you replace it with:
{{ < highlight xml >}}
    <group ns="$(arg car1_name)">
        <include file="$(find unity_backend)/launch/unity_single.launch">
            <arg name="car_name" value="$(arg car1_name)"/>
            <arg name="racecar_version" value="racecar-uw-nano"/>
            <arg name="racecar_color" value="" />
        </include>
    </group>
{{ </ highlight >}}
Notice that we only changed a single line (the include line) to use unity_single.launch instead of single_car.launch

each car node has the following publishers and subscribers:
### Publishers
Topic | Type | Description
------|------|------------
`/car_name/car_pose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| position of the car
`/car_name/car_odom` | [nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)| odometry of the car
`/car_name/joint_states` | [sensor_msgs/JointState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html)| wheel positions
`/car_name/car_imu` | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)| Imu data

### Subscribers
Topic | Type | Description
------|------|------------
`/car_name/mux/ackermann_cmd_mux/output` | [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html)| steering and speed control commands to be sent to the car
`/car_name/initialpose` | [geometry_msgs/PoseStamp](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)| initial position of the car


## Credits:
Original Source Code

Tawn Kramer - https://github.com/tawnkramer/gym-donkeycar

Roma Sokolkov - https://github.com/r7vme/gym-donkeycar cloned with permission from https://github.com/tawnkramer/sdsandbox

Release Engineer

This package was created with Cookiecutter and the audreyr/cookiecutter-pypackage project template.