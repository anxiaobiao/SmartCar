from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')

    xacro_file = os.path.join(get_package_share_directory('nav_process'),
                              'urdf',
                              'X3_description.urdf')

    # 启动雷达节点
    lidar_a1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sllidar_ros2'), 'launch'),
            '/sllidar_launch.py'])
    )


    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str)
    
    # 机器人状态节点
    robtot_state_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
            )

    # 机器人关节节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # 发布数据节点
    pub_data = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('data_process'), 'launch'),
            '/pub_odom_ium.launch.py'])
    )

    # 连接雷达
    tf_base_link_to_laser = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.0435', '5.258E-05', '0.11', '3.14', '0', '0', 'base_link', 'laser']
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',     
        output='screen',
        arguments=['-d', get_package_share_directory("nav_process") + "/rviz/display.rviz"]
    )

    ld = LaunchDescription()

    ld.add_action(lidar_a1_launch)
    ld.add_action(gui_arg)
    ld.add_action(pub_data)
    ld.add_action(robtot_state_pub)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(tf_base_link_to_laser)
    ld.add_action(joint_state_publisher_gui_node)
    # ld.add_action(rviz2)


    return ld