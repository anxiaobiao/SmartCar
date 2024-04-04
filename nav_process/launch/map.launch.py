from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 机器人urdf描述节点
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav_process'), 'launch'),
            '/description_robot.launch.py'])
    )

    imu_filter_config = os.path.join(              
        get_package_share_directory('nav_process'),
        'param',
        'imu_filter_param.yaml'
    ) 

    # 使用madgwick算法对IUM发布的数据融合
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config]
    )


    # 启动cartographer节点
    cartographer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav_process'), 'launch'),
            '/cartographer.launch.py'])
    )
    
    # 数据融合的第三方包
    ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(
            "nav_process"), 'params', 'ekf_x1_x3.yaml')],
        remappings=[('/odometry/filtered', '/odom')]  # ekf融合后发布的里程计topic
    )

    ld = LaunchDescription()

    ld.add_action(robot_description)
    ld.add_action(cartographer_node)
    ld.add_action(imu_filter_node)
    ld.add_action(ekf_node)

    return ld