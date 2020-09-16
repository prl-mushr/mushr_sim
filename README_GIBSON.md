# MuSHR-iGibson simulator
The MuSHR is the integration between MuSHR simulator and iGibson environment. Unlike the MuSHR simulator, iGibson contain database of 3D environments recreated from real indoor home, office, and other buildings, which provides a fast visual rendering. 

## Install MuSHR-iGibson
[Install standard iGibson](http://svl.stanford.edu/igibson/docs/installation.html)
### System Requirements
- Ubuntu 16.04
- Nvidia GPU with VRAM > 6.0 GB
- Nvidia driver >= 384
- CUDA >= 9.0, CuDNN >= v7
- MuHSR Simulator

### Install from source
```
git clone git@github.com:podshara/iGibson.git --recursive
cd iGibson
pip install -e .
```
### Downloading the Assets
```
python -m gibson2.utils.assets_utils --download_assets
python -m gibson2.utils.assets_utils --download_demo_data
```

For the full Gibson and iGibson datasets, you could fill up the following [license agreement](https://forms.gle/36TW9uVpjrE1Mkf9A). You will get `URL`, then run the following command to download the full dataset.
```
python -m gibson2.utils.assets_utils --download_dataset URL
```
### Uninstalling
Unintalling iGibson using `pip uninstall gibson2` \

## Running the Simulator
```
roslaunch mushr_sim gibson_sim.launch model_id:=gibson_environment_name map_dir:=your/costmap/topic/directory
```
* The simulator will use the cost map with the same name as the model_id in the map_dir
* Note that setting model_id in the argument will overwrite the setting in the [configuration file](#Mushr-iGibson-API)
#### Example

To start the iGibson sim run:
```
roslaunch mushr_sim gibson_sim.launch model_id:=Rs map:=`rospack find mushr_sim`/maps/
```
And in another termianl window launch rviz:
```
rviz
```
## Mushr iGibson API 
Simulator params can be set in `mushr_sim/config/gibson_sim.yaml` [See Example](http://svl.stanford.edu/igibson/docs/environments.html)

#### Publisher

Topic | Type | Description
------|------|--------------
`/gibson_ros/camera/rgb/image` | [ImageMsg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) | Simulated camera's color image topic.
`/gibson_ros/camera/depth/image` | [ImageMsg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) | Simulated camera's depth image topic.
`/gibson_ros/lidar/points` | [PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | Simulated lidar laser scan topic.
`/gibson_ros/camera/depth/image_raw` | [ImageMsg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) | Simulated camera's raw depth image topic.


#### Subscribers
Topic | Type | Description
------|------|--------------
`/car_pose`| [geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) |Pose of robot. Published when map &rarr; base_footprint transforms exist (sim).
