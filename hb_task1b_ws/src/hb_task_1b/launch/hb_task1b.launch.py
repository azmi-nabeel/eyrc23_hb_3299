from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hb_task_1b',
            executable='controller.py'
        ),
        Node(
            package='hb_task_1b',
            executable='service_node.py'
        )
    ])
'''
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    controller_node = Node(
        package="hb_task_1b",           # Enter the name of your ROS2 package
        executable="controller.py",    # Enter the name of your executable
    )
    service_node = Node(
        package="hb_task_1b",           # Enter the name of your ROS2 package
        executable="service_node.py",    # Enter the name of your executable
    )
'''