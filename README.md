# MuSHR Simulator
The MuSHR simulator is the easiest way to get started with MuSHR. The simulated car can be either controlled via keyboard teleoperation, or by a control algorithm. This simulator is designed to emulate the car's configuration as close as possible to reduce sim-to-real overhead. To do so, many of the hardware components are programmatically simulated (camera is not simulated).

To make running the sim easier for various platforms we have provided a docker container with all the necessary components included. See below for setup.

### Install
To run the sim you will need to install docker from [here](https://docs.docker.com/v17.12/install/) and docker-compose found [here](https://docs.docker.com/compose/install/). You will also need the host computer to have some nvidia driver. The default driver setup is for `nvidia-390`, you can check your driver version by running `nvidia-smi` in a terminal.

1. If docker deamon isn't running run `systemctl start docker`
2. Download the [mushr_sim](https://hub.docker.com/r/schmittle/mushr_sim) image using docker commands
3. Clone this repo and `cd .../mushr_sim/mushr_sim_docker/`
4. Using your favorite text editor, change .env to your driver version number ingoring numbers past decimal
5. `docker-compose up -d` This should create a container visible if you type `docker ps`
6. `docker exec -it <container ID> bash` This will get you into the container as user developer.
7. `roslaunch mushr_sim teleop.launch`
8. In a new terminal redo step 6 or use tmux before step 7 and create a new window
9. `rviz` a map with a car model and arrow of the car pose should now be visible
10. Give it an initial pose using the button at the top of the rviz window. And drive around using the WSDA keys while the small gray window is in focus  
  
NOTE: If you need to be root to download additional software run `docker exec -it -u 0 <container ID> bash`.

For more details, checkout the tutorial!

### Tutorials
Tutorials using the simulator can be found in the main [MuSHR repository](https://github.com/personalrobotics/mushr)

### API
For adjusting params see `config` it has the simulated vesc params and also the sensor params for each car. If you don't find the publishers or subscribers you were looking for, they are likely in [mushr_base](https://github.com/prl-mushr/mushr_base).

#### Publishers
Topic | Type | Description
------|------|------------
`/mux/ackermann_cmd_mux/input/teleop`| [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) | Publish teleop controls from keyboard
`/map` | [nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html) | Map from map server
`/map_metadata` | [nav_msgs/MapMetaData](http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html) | Map metadata
`/scan` | [sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) | Simulated laser scan topic

#### Subscribers
Topic | Type | Description
------|------|------------
`/tf` | [tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html) | Transforms for the laserscan
`/tf_static` | [tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html) | Static transforms for the laserscan
