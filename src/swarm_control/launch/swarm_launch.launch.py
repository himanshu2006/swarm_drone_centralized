#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    sdf_model_path = os.path.expanduser('~/swarm_ws/src/swarm_model/parrot_bebop_2/model.sdf')
    world_sdf_path = os.path.expanduser('~/swarm_ws/src/swarm_model/worlds/swarm_world.sdf')

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_sdf_path}'}.items(),
    )

    swarm_manifest = [
        {'name': 'mother',  'x': '0.0',  'y': '0.0',  'z': '0.5'},
        {'name': 'child_1', 'x': '-1.5', 'y': '1.5',  'z': '0.5'},
        {'name': 'child_2', 'x': '-1.5', 'y': '-1.5', 'z': '0.5'},
        {'name': 'child_3', 'x': '-3.0', 'y': '0.0',  'z': '0.5'}
    ]

    launch_pipeline = [gazebo_simulator]
    bridge_arguments = []

    for drone in swarm_manifest:
        drone_name = drone['name']

        namespaced_drone_group = GroupAction(
            actions=[
                PushRosNamespace(drone_name),
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    output='screen',
                    arguments=[
                        '-name', drone_name,
                        '-file', sdf_model_path,
                        '-x', drone['x'],
                        '-y', drone['y'],
                        '-z', drone['z'],
                    ]
                )
            ]
        )
        launch_pipeline.append(namespaced_drone_group)

        cmd_vel_bridge = f'/model/{drone_name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
        odom_bridge = f'/model/{drone_name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
        
        # Exact Gazebo Fortress World Topic Paths
        rgb_bridge = f'/world/swarm_world/model/{drone_name}/link/body/sensor/realsense_rgb/image@sensor_msgs/msg/Image[ignition.msgs.Image'
        depth_bridge = f'/world/swarm_world/model/{drone_name}/link/body/sensor/realsense_depth/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image'
        
        bridge_arguments.extend([
            cmd_vel_bridge, 
            odom_bridge, 
            rgb_bridge, 
            depth_bridge
        ])

    # Target Box Odometry Bridge
    box_odom_bridge = '/model/target_box/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
    bridge_arguments.append(box_odom_bridge)

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='swarm_parameter_bridge',
        output='screen',
        arguments=bridge_arguments
    )
    
    launch_pipeline.append(ros_gz_bridge_node)

    return LaunchDescription(launch_pipeline)