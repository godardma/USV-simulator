from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    boat_node = Node(
        package="usv_simulator",
        executable="boat_simulator.py",

    )
    control_node = Node(
        package="usv_simulator",
        executable="boat_control.py",

    )

    ld.add_action(boat_node)
    ld.add_action(control_node)

    return ld