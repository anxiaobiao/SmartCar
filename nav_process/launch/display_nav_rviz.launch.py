from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',     
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        arguments=['-d', get_package_share_directory("nav_process") + "/rviz/nav.rviz"]
    )

    return LaunchDescription([
        rviz_node
    ])