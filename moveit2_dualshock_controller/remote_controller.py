#!/usr/bin/env python

import numpy
from .input import Input

# import rclpy and all the ros api things
import rclpy
from rclpy.node import Node
from rclpy.node import Parameter, SetParametersResult, ParameterDescriptor
from rclpy.parameter_service import ListParameters
# this is the type of message expected
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Float64
from typing import List

from std_srvs.srv import Trigger


class ControllerNode(Node):
    def __init__(self):
        super(ControllerNode, self).__init__("DS_input_node")

        # initialize the command line parameters
        self.add_on_set_parameters_callback(self.set_parameters)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_to', '/servo_node/delta_twist_cmds'),
                ('gripper_topic', '/gripper_control/float_cmds'),
                ('publish_rate', 24),
                ('speed_reduction_factor', 5.), # reduce the translational velocity to balance out with the rotational velocity at higher reduction
                ('rotation_reduction_factor', 1.),
                ('controlled_joint', 'base_link'),
                ('device_index', 0),
                ('gripper_name', 'gripper_control_node')
            ]
        )
        # the parameter intializer should be called at this point, initializing all the parameters to their default/specified values

        # a veriable to control, wether the moveit servo is triggered, bound to home button
        self.servo_activated = False
        self.servo_starter = self.create_client(Trigger, srv_name='/servo_node/start_servo')
        self.servo_stopper = self.create_client(Trigger, srv_name='/servo_node/stop_servo')

        # for debug purposes
        # print(self.target_topic_name, self.gripper_topic_name, self.hz, self.speed_reduction, self.rotation_reduction, self.joint)

        # gripper control, required to be able to operate it in two mode for opening and
        # closing: move and grip and move and release
        self.mess = TwistStamped()

        # initialize the key bindings
        # name of the mapped axis goes along with direction modifier
        self.binding_dict = {
            'tr_x': {'ax':'lx', 'mod':1},
            'tr_y': {'ax':'ly', 'mod':1},
            'tr_z_forward': {'ax':'r2', 'mod':1},
            'tr_z_backward': {'ax':'l2', 'mod':1},
            'rot_x': {'ax':'rx', 'mod':1},
            'rot_y': {'ax':'ry', 'mod':1},
            'rot_z_cw': {'ax':'x', 'mod':1},
            'rot_z_ccw': {'bu':'x', 'mod':1},
            'act_deact': {'bu':'home'},
            'tool_grip': {'bu':'l'},
            'tool_release': {'bu':'r'},
        }

        # done

    def set_parameters(self, params:List[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name == "publish_to":
                self.target_topic_name = param.value
                if hasattr(self, "target_topic"):
                    self.target_topic.destroy()
                self.target_topic = self.create_publisher(TwistStamped, self.target_topic_name, 10)
            elif param.name == "gripper_topic":
                self.gripper_topic_name = param.value
                if hasattr(self, "gripper_topic"):
                    self.gripper_topic.destroy()
                self.gripper_topic = self.create_publisher(Float64, self.gripper_topic_name, 10)
            elif param.name == "publish_rate":
                self.hz = param.value
                if hasattr(self, "loop"):
                    self.loop.destroy()
                self.loop = self.create_timer(1./self.hz, self.timer_callback)
                print(f"Timer interval is {1. / self.hz}")
            elif param.name == "speed_reduction_factor":
                self.speed_reduction = param.value
            elif param.name == "rotation_reduction_factor":
                self.rotation_reduction = param.value
            elif param.name == "controlled_joint":
                self.joint = param.value
            elif param.name == "device_index":
                if hasattr(self, "ctr"):
                    del self.ctr
                self.ctr = Input(param.value)
        print(f"The following parameters have been set: {' | '.join([p.name+'='+str(p.value) for p in params])}")
        return SetParametersResult(successful=True, reason=f"The following parameters have been set: "
                                                           f"{' | '.join([p.name+'='+str(p.value) for p in params])}")


    def timer_callback(self):
        if self.ctr.available():

            # collect the event from the device, nonblocking
            publish_gripper = False
            gripper_value = 0.

            e = self.ctr.event()
            while e:
                if e.axis('lx') is not None:
                    self.mess.twist.linear.x = e.axis('lx') / self.speed_reduction
                if e.axis('ly') is not None:
                    self.mess.twist.linear.y = - e.axis('ly') / self.speed_reduction
                if e.axis('l2') is not None:
                    self.mess.twist.linear.z = ((e.axis('l2') + 1)/2) / self.speed_reduction
                if e.axis('r2') is not None:
                    self.mess.twist.linear.z = -((e.axis('r2') + 1)/2) / self.speed_reduction

                if e.axis('rx') is not None:
                    self.mess.twist.angular.x = e.axis('rx') / self.rotation_reduction
                if e.axis('ry') is not None:
                    self.mess.twist.angular.y = e.axis('ry') / self.rotation_reduction

                if e.axis('x') is not None:
                    self.mess.twist.angular.z = e.axis('x') / self.rotation_reduction

                if e.axis('y') is not None:
                    # axis for the gripper
                    publish_gripper = True
                    gripper_value = e.axis('y')

                if e.button('home') is not None and self.ctr.button_states['home']:
                    print("button pressed")
                    if self.servo_activated:
                        self.servo_stopper.call(Trigger.Request())
                        self.servo_activated = False
                    else:
                        self.servo_starter.call(Trigger.Request())
                        self.servo_activated = True
                    print(f"Activation of the moveit servo module: {self.servo_activated}")

                if e.button('a') is not None and self.ctr.button_states['a']:
                    # switch to the local coordinate mode
                    self.joint = 'tool0'
                elif e.button('b') is not None and self.ctr.button_states['b']:
                    # switch to the global coordinates
                    self.joint = 'base_link'

                e = self.ctr.event()

        # publish to the topic
        self.mess.header.stamp = self.get_clock().now().to_msg()
        self.mess.header.frame_id = self.joint
        self.target_topic.publish(self.mess)
        if publish_gripper:
            grip_msg = Float64()
            grip_msg.data = gripper_value
            self.gripper_topic.publish(grip_msg)


    def destroy_node(self) -> bool:
        del(self.ctr)   # close the files for the input devices
        return super(ControllerNode, self).destroy_node()


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
