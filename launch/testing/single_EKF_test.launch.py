import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


config_file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../config/config.yml'))
bag_file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../bags/test_3'))
rviz_config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../config/rviz_config.rviz'))


def generate_launch_description():
    
    print(config_file_path)
    
    bag_play = ExecuteProcess(
    cmd=['ros2', 'bag', 'play', bag_file_path, '--loop'],
    output='screen',
    prefix="xterm -e"

    )
    
    
    localisation_engine = Node(
        package='localisation_engine',  
        executable='localisation_engine_node',  
        name='localisation_engine_node',
        parameters=[config_file_path,
                    {"EKF_TYPE" : "EKF_w_callibration"}],
        output='screen',
        prefix="xterm -e"


    )
    marker_generator = Node(
        package='localisation_engine',
        executable='marker_generator_node',
        name='marker_generator_node',
        parameters=[
            config_file_path,
            {'marker_green': 1.0, 'marker_red': 0.0, 'marker_blue': 0.0},
        ],
        output='screen',
        prefix="xterm -e"
        
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        prefix='xterm -e',
        arguments=['-d', rviz_config_path],

    )


    return LaunchDescription([
        bag_play,
        localisation_engine,
        marker_generator,
        rviz2
    ])