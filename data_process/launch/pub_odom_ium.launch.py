from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    drive_car = Node(package="drive", executable="drive_car")

    pub_odom = Node(package="data_process", executable="pub_odom")



    return LaunchDescription([
        drive_car,
        pub_odom])