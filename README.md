# tugbot_autoware_pkgs

Ignition Gazebo's Tugbot to run in Autoware Universe
Refer to this article to install Ignition Gazebo and get tugbot's sdf working.

https://qiita.com/porizou1/items/2f2f82427c5220e95369


# Install Autoware

Install the main branch according to the Launching Autoware on this page.
https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/

# Creating a map


## PCD map


```bash
ign gazebo  tugbot_depot.sdf 
```

```bash
ros2 launch tugbot_autoware_pkg ign_ros2_bridge.launch.xml
```

```bash
ros2 launch tugbot_autoware_pkg lidarslam.launch.py 
```

Checking the map with rviz2

```bash
rviz2 -d mapping.rviz
```

Save PCD file

```bash
ros2 service call /map_save std_srvs/Empty
```

## Vector Map

Use Vector Map Builder.

https://tools.tier4.jp/vector_map_builder_ll2/

# Run

```bash
ign gazebo  tugbot_depot.sdf 
```

```bash
ros2 launch tugbot_autoware_pkg ign_ros2_bridge.launch.xml
```

Launch Autoware

```bash
ros2 launch tugbot_autoware_pkg autoware.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=/home/porizou/tugbot_ws/src/tugbot_autoware_pkg/map
```

