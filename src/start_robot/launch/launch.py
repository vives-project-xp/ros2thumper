from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    thumper_drive = Node(
        package="thumper_drive",
        executable="thumper_drive_node",
    )

    trex_motor_controller = Node(
        package="trex_motor_controller",
        executable="trex_motor_controller_node",
    )
 
    joy_linux = Node(
	package="joy_linux",
	executable="joy_linux_node",
    )
	
    ld.add_action(thumper_drive)
    ld.add_action(trex_motor_controller)
    ld.add_action(joy_linux)

    return ld
