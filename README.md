# SmartCar

使用ROS2对智能小车进行控制，并进行建图和导航工作

## 功能

- 基于官方提供的底层库接口，发布小车移动话题；
- 基于cartographer框架和robot_localization框架，通过移动智能小车构建地图；
- 基于Navigation2框架，在构建的地图中实现导航功能；

## 环境要求

- Ubuntu 20.04
- ROS2 Foxy

## 运行方式

- 启动智能小车控制节点

```bash
ros2 run drive drive_car	# 启动控制节点
ros2 run teleop_twist_keyboard teleop_twist_keyboard	# 启动键盘控制节点
```

- 启动建图节点

```bash
ros2 launch nav_process map.launch.py	# 启动建图节点
ros2 launch nav_process display_map_rviz.launch.py 	# 启动rviz2显示节点
ros2 run teleop_twist_keyboard teleop_twist_keyboard	# 启动键盘控制节点

# 在rviz2中建图结束后
ros2 launch nav_process save_map.launch.py	#保存建图结果
```

- 启动导航节点

```bash
ros2 launch nav_process description_robot.launch.py	# 启动智能小车各控制节点
ros2 launch nav_process nav_dwa_planning.launch.py	# 启动导航节点
ros2 launch nav_process display_nav_rviz.launch.py 	# 启动rviz2显示节点
```

## TODO

- 由于使用导航节点时，使用Nav2提供的dwa算法进行路径规划工作，后续可以通过研读Nav2源码并了解如何使用Nav2框架编写自定义算法插件；
- 实现自定义的madgwick算法，完成对/ium九轴数据的融合工作；
- 实现自定义的卡尔曼滤波算法，完成对/ium数据和/odom数据的滤波工作