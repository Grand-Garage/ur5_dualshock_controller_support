from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    dualshock_node = Node(
        name="dualshock",
        package="moveit_dualshock_controller",
        executable="dualshock_controller_node",
    )
    gripper_node = Node(
        name="gripper_control",
        package="moveit_dualshock_controller",
        executable="gripper_control_node"
    )
    ld.add_action(dualshock_node)
    ld.add_action(gripper_node)
    return ld