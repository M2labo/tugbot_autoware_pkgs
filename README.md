# tugbot_autoware_pkgs
Ignition Gazebo のTugbotをAutoware Universeで動かすためのもの


この記事を参照してigniton Gazebo をインストールしてtugbotのsdfを動くようにしておく

https://qiita.com/porizou1/items/2f2f82427c5220e95369

# ignition Gazeboの起動

```bash
ign gazebo  tugbot_depot.sdf 
```

# ROS2 bridgeの起動

```bash
ros2 launch tugbot_autoware_pkg ign_ros2_bridge.launch.xml
```

# Autowareのインストール

こちらのページのLaunching Autowareにしたがってawsim-stableブランチをインストールする。

https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/

# マップの作成

## PCDマップの作成
```bash
ign gazebo  tugbot_depot.sdf 
```

```bash
ros2 launch tugbot_autoware_pkg ign_ros2_bridge.launch.xml
```

```bash
ros2 launch tugbot_autoware_pkg lidarslam.launch.py 
```

rviz2でmapの確認

```bash
rviz2 -d mapping.rviz
```

PCDの保存

```bash
ros2 service call /map_save std_srvs/Empty
```

## Vector Mapの作成

Vector Map Builderを使用

https://tools.tier4.jp/vector_map_builder_ll2/

# Autoware で動かす

```bash
ign gazebo  tugbot_depot.sdf 
```

```bash
ros2 launch tugbot_autoware_pkg ign_ros2_bridge.launch.xml
```

Autoware 起動

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=/home/porizou/tugbot_ws/src/tugbot_autoware_pkg/map
```


追加で必要なノードの起動（現在はAckermannControlCommandをTwistに変換するノードのみ）

```bash
ros2 launch  tugbot_autoware_pkg tugbot_autoware.launch.py
```


