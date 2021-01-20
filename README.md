# cld1a0_pcl
cld1a0_pcl is a ROS package, which is designed to visualize 2D/3D distance dataset in real-time.
For more details in the 2D/3D ToF LiDAR, please visit http://www.cygbot.com

## How to build this package

### Preparation
1) Clone this project to your catkin's workspace
2) Run catkin_make

### Set-Up
Install the udev rule
```bash
roscd cld1a0_pcl
cd scripts
chmod +x create_udev_rules.sh
./create_udev_rules.sh
```

Run the launch file
```bash
roslaunch cld1a0_pcl cyglidar.launch
```

## Parameters
In cyglidar.launch, the version number of datasets can be switched 0(2D) and 1(3D) as below:

2D)
<h1 align="left">
  <img src="screenshots/param_2d.png" width="600"/>
</h1>

3D)
<h1 align="left">
  <img src="screenshots/param_3d.png" width="600"/>
</h1>

In Rviz, the fixed frame and the topic for Point Cloud must be updated.
```bash
// Fixed Frame
/laser_link
```

```bash
// Topic
/scan
```
