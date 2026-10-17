import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'planar_target_tracker'
    params = os.path.join(get_package_share_directory(pkg), 'config', 'params.yaml')
    sim_time = {'use_sim_time': True}      # el bag se reproduce con --clock

    return LaunchDescription([
        Node(package=pkg, executable='depth_segmenter_node', name='depth_segmenter',
             parameters=[params, sim_time], output='screen'),
        Node(package=pkg, executable='pose_estimator_node', name='pose_estimator',
             parameters=[params, sim_time], output='screen'),
    ])
