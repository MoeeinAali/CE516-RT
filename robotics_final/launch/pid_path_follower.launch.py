import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('robotics_final')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Enable simulated /clock'
    )

    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.5',
        description='Max linear velocity (m/s)'
    )

    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='1.0',
        description='Max angular velocity (rad/s)'
    )

    linear_kp_arg = DeclareLaunchArgument(
        'linear_kp',
        default_value='0.5',
        description='Linear P gain'
    )

    linear_ki_arg = DeclareLaunchArgument(
        'linear_ki',
        default_value='0.0',
        description='Linear I gain'
    )

    linear_kd_arg = DeclareLaunchArgument(
        'linear_kd',
        default_value='0.1',
        description='Linear D gain'
    )

    angular_kp_arg = DeclareLaunchArgument(
        'angular_kp',
        default_value='1.0',
        description='Angular P gain'
    )

    angular_ki_arg = DeclareLaunchArgument(
        'angular_ki',
        default_value='0.0',
        description='Angular I gain'
    )

    angular_kd_arg = DeclareLaunchArgument(
        'angular_kd',
        default_value='0.1',
        description='Angular D gain'
    )

    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.5',
        description='Lookahead distance (m)'
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.2',
        description='Goal tolerance (m)'
    )

    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='20.0',
        description='Control loop frequency (Hz)'
    )

    pid_path_follower_node = Node(
        package='robotics_final',
        executable='trajectory_controller.py',
        name='pid_path_follower',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
            'linear_kp': LaunchConfiguration('linear_kp'),
            'linear_ki': LaunchConfiguration('linear_ki'),
            'linear_kd': LaunchConfiguration('linear_kd'),
            'angular_kp': LaunchConfiguration('angular_kp'),
            'angular_ki': LaunchConfiguration('angular_ki'),
            'angular_kd': LaunchConfiguration('angular_kd'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'control_frequency': LaunchConfiguration('control_frequency'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        control_frequency_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        linear_kp_arg,
        linear_ki_arg,
        linear_kd_arg,
        angular_kp_arg,
        angular_ki_arg,
        angular_kd_arg,
        lookahead_distance_arg,
        goal_tolerance_arg,
        pid_path_follower_node,
    ])
