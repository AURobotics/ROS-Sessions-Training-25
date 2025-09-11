from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld=LaunchDescription()
    
    demo_node=Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            remappings=[('/turtle1/cmd_vel', '/my_cmd_vel')]
        )
    
    ld.add_action(demo_node)
    return ld