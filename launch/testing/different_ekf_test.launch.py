import os
from launch import LaunchDescription
from launch_ros.actions import Node


config_file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../config/config.yml'))
def generate_launch_description():
    
    print(config_file_path)
    
    #### NO VELOCITY
    
    
    localisation_engine_1 = Node(
        package='localisation_engine',
        executable='localisation_engine_node',  
        name='localisation_engine_node',
        parameters=[config_file_path,
                    {"EKF_TYPE" : "EKF_raw_no_vel"}],
        remappings=[
            ('/tag/coords', '/tag1/coords'),
        ],
        output='screen',

    )
    marker_generator_1 = Node(
        package='localisation_engine',
        executable='marker_generator_node',
        name='marker_generator_node',
        parameters=[
            config_file_path,
            {'marker_green': 1.0, 'marker_red': 0.0, 'marker_blue': 0.0},
        ],
        remappings=[
            ('/tag/coords', '/tag1/coords'),
            ('/tag/marker', '/tag1/marker'),
        ],
        output='screen',

    )
    
    
    #### WITH VELOCITY


    localisation_engine_2 = Node(
        package='localisation_engine',
        executable='localisation_engine_node',  
        name='localisation_engine_node',
        parameters=[config_file_path,
                    {"EKF_TYPE" : "EKF_raw_w_vel"}],
        remappings=[
            ('/tag/coords', '/tag2/coords'),
        ],
        output='screen',

    )
    marker_generator_2 = Node(
        package='localisation_engine',
        executable='marker_generator_node',
        name='marker_generator_node',
        parameters=[
            config_file_path,
            {'marker_green': 0.0, 'marker_red': 1.0, 'marker_blue': 0.0},
        ],
        remappings=[
            ('/tag/coords', '/tag2/coords'),
            ('/tag/marker', '/tag2/marker'),
        ],
        output='screen',

    )
    


    return LaunchDescription([
        localisation_engine_1,
        marker_generator_1,
        localisation_engine_2,
        marker_generator_2
    ])