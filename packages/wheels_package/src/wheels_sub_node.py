#!/usr/bin/env python3

import os
import rospy
from datetime import datetime
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, BoolStamped
from std_msgs.msg import String


HOST = os.environ['VEHICLE_NAME']
cmd = f'/{HOST}/car_cmd_switch_node/cmd'
wheels_cmd_executed = f'/{HOST}/wheels_driver_node/wheels_cmd_executed'

class WheelsSubNode(DTROS):
    def __init__(self, node_name): 
        super(WheelsSubNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self.cmd_sub = rospy.Subscriber(
            cmd,
            Twist2DStamped,
            self.cmd_cb,
            queue_size=1
        )

        self.wheels_sub = rospy.Subscriber(
            wheels_cmd_executed,
            WheelsCmdStamped,
            self.wheels_cb,
            queue_size=1
        )



        self.time = datetime.now()

        self.wh = WheelsCmdStamped()
        self.tw = Twist2DStamped()

    def cmd_cb(self, twist):
        if self.tw.v != twist.v or self.tw.omega != twist.omega:
            self.tw = twist
            rospy.loginfo(f'Twist: v={twist.v}, omega={twist.omega}')
            rospy.loginfo(f'TIMEDIFF: {(datetime.now() - self.time).total_seconds()}')
            self.time = datetime.now()

    def wheels_cb(self, wheel):
        if self.wh.vel_left != wheel.vel_left or self.wh.vel_right != wheel.vel_right:
            self.wh = wheel
            rospy.loginfo(f'Wheel: left={wheel.vel_left}, right={wheel.vel_right}')

if __name__ == '__main__':
    node = WheelsSubNode(node_name='wheels_sub_node')
    rospy.spin()
