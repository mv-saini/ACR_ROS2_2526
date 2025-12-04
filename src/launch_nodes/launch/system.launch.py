from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='ros_tcp_endpoint',
            parameters=[{
                'ROS_IP': '0.0.0.0',
                'ROS_TCP_PORT': 10000
            }],
            output='screen'
        ),
        
        Node(
            package='robot_manager',
            executable='robot_manager_server.py',
            name='robot_manager',
            output='screen'
        ),

        Node(
            package='path_planner',
            executable='path_planner_server.py',
            name='path_planner_server',
            output='screen'
        ),

        Node(
            package='airport_grid',
            executable='publish_airport_grid.py',
            name='airport_grid_publisher',
            output='screen'
        ),
    ])