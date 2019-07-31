# MuSHR Simulator
The MuSHR simulator is the easiest way to get started with MuSHR. The simulated car can be either controlled via keyboard teleoperation, or by a control algorithm. This simulator is designed to emulate the car's configuration as close as possible to reduce sim-to-real overhead. To do so, many of the hardware components are programmatically simulated (camera is not simulated).

To make running the sim easier for various platforms we have provided a docker container with all the necessary components included. See below for setup.

### Install
To run the sim you will need to install docker from [here](https://docs.docker.com/v17.12/install/) and docker-compose found [here](https://docs.docker.com/compose/install/). You will also need the host computer to have some nvidia driver. The default driver setup is for `nvidia-390`. You can check your driver version in [linux](https://linuxconfig.org/how-to-check-nvidia-driver-version-on-your-linux-system), [mac](https://bluebellflora.com/finding-out-which-nvidia-geforce-driver-version-your-mac-is-running/), and [windows](https://nvidia.custhelp.com/app/answers/detail/a_id/2039/~/how-do-i-determine-which-nvidia-display-driver-version-is-currently-installed). You will also need [git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git).

1. Open up a terminal
2. Check if docker is running: `docker run hello-world`. If not, run:
- Linux:`systemctl start docker` 
- Mac: `open /Applications/Docker.app` 
- Windows: `restart-service *docker*`  
3. Clone this repo: 
`git clone https://github.com/prl-mushr/mushr_sim && cd mushr_sim/docker/`
4. Using your favorite text editor, change .env to your nvidia driver version number ingoring numbers past decimal and ones digit. For example, 341.2 would become 340
5. `docker-compose up -d` This should create a container visible if you type `docker ps`
A map with a car model and arrow of the car pose should now be visible in rviz and a small gray window should also appear.
9. Give it an initial pose using the "2D Pose Estimate" button at the top of the rviz window. And drive around using the WSDA keys while the small gray window is in focus 
10. To enter the container use `docker exec -it <container ID> bash` You will be the developer user. If you need to be root to download additional software run `docker exec -it -u 0 <container ID> bash`.  

For more details, checkout the tutorial!

### Tutorials
Tutorials using the simulator can be found [here](https://prl-mushr.github.io/tutorials/quickstart/).

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
