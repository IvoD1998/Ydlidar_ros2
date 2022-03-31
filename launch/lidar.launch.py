import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ydlidar_ros2",
            namespace="ydlidar",
            executable="ydlidar",
            output="screen",
            parameters=[{
                "port":"/dev/ttyUSB0",  
                "baudrate":115200,
                "frame_id":"laser_frame",
                "low_exposure":False,
                "resolution_fixed":True,
                "auto_reconnect":True,
                "reversion":False,
                "angle_min":-180. ,
                "angle_max":180. ,
                "range_min":0.1 ,
                "range_max":16.0 ,
                "ignore_array":"" ,
                "samp_rate":9,
                "frequency":7.
            }]
        )
    ])