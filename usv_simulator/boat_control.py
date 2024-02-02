#!/usr/bin/env python3

from usv_simulator.msg import UsvCommand
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node


class BoatController(Node):

    def __init__(self):
        super().__init__('boat_controller')
        self.publisher = self.create_publisher(UsvCommand, 'usv_command', 1000)
        self.subscription_pose = self.create_subscription(
            Twist,
            'twist_command',
            self.twist_callback,
            1000)
        self.command=UsvCommand()




    
    def twist_callback(self,msg):
        av,rot=msg.linear.x,msg.angular.z
        if rot>1:
            rot=1
        elif rot<-1:
            rot=-1

        if rot>0:
            self.command.right=80*av
            self.command.left=80*av-rot*80
        else:
            self.command.left=80*av
            self.command.right=80*av+rot*80
        self.publisher.publish(self.command)


def main(args=None):
    rclpy.init(args=args)

    boat_controller = BoatController()

    rclpy.spin(boat_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
