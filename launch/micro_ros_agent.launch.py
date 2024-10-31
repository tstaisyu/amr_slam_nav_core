import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # micro_ros_agent_acm0
    micro_ros_agent_acm0 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_acm0',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-v4'],
        output='screen'
    )

    # micro_ros_agent_acm1
    micro_ros_agent_acm1 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_acm1',
        arguments=['serial', '--dev', '/dev/ttyUSB1', '-v4'],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        micro_ros_agent_acm0,
        micro_ros_agent_acm1
    ])