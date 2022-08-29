import asyncio
from .ur_robotiq_gripper.gripper import Gripper
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
#
# async def log_info(gripper):
#     print(f"Pos: {str(await gripper.get_current_position()): >3}  "
#           f"Open: {await gripper.is_open(): <2}  "
#           f"Closed: {await gripper.is_closed(): <2}  ")
#
# async def run():
#     gripper = Gripper('192.168.0.102')  # actual ip of the ur arm
#
#     await gripper.connect()
#     await gripper.activate()  # calibrates the gripper
#
#     await gripper.move_and_wait_for_pos(255, 255, 255)
#     await log_info(gripper)
#     await gripper.move_and_wait_for_pos(0, 255, 255)
#     await log_info(gripper)
#
# asyncio.run(run())

class GripperControlNode(Node):
    def __init__(self):
        super(GripperControlNode, self).__init__("gripper_control")

        # initialize the command line parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_ip', '192.168.0.102'),
                ('gripper_step_modifier', 4),
                ('gripper_speed', 4),
                ('gripper_force', 255)
            ]
        )

        self.gripper = Gripper(self.get_parameter('robot_ip').value)
        self.gripper_step_modifier = self.get_parameter('gripper_step_modifier').value
        self.gripper_speed = self.get_parameter('gripper_speed').value
        self.gripper_force = self.get_parameter('gripper_force').value

        # assume we are connected, no way to check it, the library is primitive
        self.loop = asyncio.new_event_loop()
        self.loop.run_until_complete(self.gripper.connect())
        # activate and calibrate the gripper
        self.loop.run_until_complete(self.gripper.activate())
        if not self.loop.run_until_complete(self.gripper.is_active()):
            raise Exception("Failed to connect to the gripper")

        # register the subscriber for the topic
        self.sub = self.create_subscription(
            Float64,
            '/gripper_control/float_cmds',
            self.listener_gripper_callback,
            1
        )

    def listener_gripper_callback(self, msg: Float64):
        # qos is set in a way that it is going to move in steps
        speed = msg.data
        target = self.gripper.get_closed_position() if speed < 0. \
            else self.gripper.get_open_position() if speed > 0. \
            else self.loop.run_until_complete(self.gripper.get_current_position())
        #target_speed = speed*self.gripper_speed
        print(target, self.gripper_speed, self.gripper_force)
        self.loop.run_until_complete(self.gripper.move(
            target,
            self.gripper_speed,
            self.gripper_force
        ))

def main(args=None):
    rclpy.init(args=args)

    controller_node = GripperControlNode()

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()