import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

config_file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../config/config.yml'))
tmux_session_name = "localisation_engine"

def generate_launch_description():

    create_session = ExecuteProcess(
        cmd=['tmux', 'new-session', '-d', '-s', tmux_session_name, '-n', 'serial'],
        output='screen'
    )

    run_nodes = TimerAction(
        period=0.5,
        actions=[

            ExecuteProcess(cmd=['tmux', 'send-keys', '-t', f'{tmux_session_name}:serial', 
                 f'ros2 run serial_parser serial_parser --ros-args --params-file {config_file_path}', 'C-m']),
            
            ExecuteProcess(cmd=['tmux', 'new-window', '-t', tmux_session_name, '-n', 'localisation']),
            ExecuteProcess(cmd=['tmux', 'send-keys', '-t', f'{tmux_session_name}:localisation', 
                 f'ros2 run localisation_engine localisation_engine_node --ros-args --params-file {config_file_path}', 'C-m']),


            ExecuteProcess(cmd=['tmux', 'new-window', '-t', tmux_session_name, '-n', 'marker_gen']),
            ExecuteProcess(cmd=['tmux', 'send-keys', '-t', f'{tmux_session_name}:marker_gen', 
                 f'ros2 run localisation_engine marker_generator_node --ros-args --params-file {config_file_path}', 'C-m']),
        ]
    )

    return LaunchDescription([
        create_session,
        run_nodes
    ])