#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + os.path.join(get_package_share_directory('rescue_run'), 'models')
    else:
        os.environ['GAZEBO_MODEL_PATH'] = os.path.join(get_package_share_directory('rescue_run'), 'models')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    maze_map_config_file_path = os.path.join(get_package_share_directory('rescue_run'), 'config', 'maze_map.yaml')
    maze_world_path = os.path.join(get_package_share_directory('rescue_run'), 'worlds', 'world', 'maze.world')
    params_config_file_path = os.path.join(get_package_share_directory('rescue_run'), 'config', 'tb3_nav_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-7.5')    
    y_pose = LaunchConfiguration('y_pose', default='-7.5')    

    setting_turtlebot3_model = SetEnvironmentVariable(
        name = "TURTLEBOT3_MODEL",
        value="burger"
    )

    gzserver_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    ),
    launch_arguments={'world': maze_world_path}.items() 
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,   
            'y_pose': y_pose,   
        }.items()
    )
    
    maze_mapping_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource (
            launch_file_path=os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')                        
        )        
    )
    
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), "launch", "bringup_launch.py")
        ),
        launch_arguments = {
            'map': maze_map_config_file_path,
            'params_file': params_config_file_path,
            'use_sim_time': 'true',
            'slam': 'True',  
        }.items(),
    )   
    rviz_config = os.path.join(
       get_package_share_directory('rescue_run'),
    '   rviz',
      'config.rviz'
    )

    rviz_2 = Node(
      package='rviz2',
      executable='rviz2',
       name='rviz2',
       output='screen',
       arguments=['-d', rviz_config]
      )
     
    ld = LaunchDescription()
    ld.add_action(setting_turtlebot3_model)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(navigation)
    ld.add_action(rviz_2)
    
    
    return ld
