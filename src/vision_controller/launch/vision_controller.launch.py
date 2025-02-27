from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_controller',
            executable='change_state_node.py',
            name='change_state',
        ),
        Node(
            package='vision_controller',
            executable='detect_body_node.py',
            name='detect_body',
        )
        # Node(
        #     package='vision_controller',
        #     executable='follow_person_node.py',
        #     name='follow_person',
        # )
    ])