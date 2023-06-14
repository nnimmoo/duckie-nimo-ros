#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, BoolStamped
from std_msgs.msg import String, Bool


HOST = os.environ['VEHICLE_NAME']
cmd = f'/{HOST}/car_cmd_switch_node/cmd'
wheels_cmd_executed = f'/{HOST}/wheels_driver_node/wheels_cmd_executed'
duckiebot_detected= f'/{HOST}/duckiebot_detected'
duckie_detected= f'/{HOST}/duckie_detected'

class WheelsNode(DTROS):
    def __init__(self, node_name):
        super(WheelsNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # testing for all Twist2DStamped, WheelsCmdStamped
        self.duckie_detect = False
        self.duckiebot_detect = False
        self.duckie_detected_sub  = rospy.Subscriber(
            'duckie_detected', Bool, 
            self.duckie_dt_cb, 
            queue_size=1
            )
        self.duckiebot_detected_sub = rospy.Subscriber(
            'duckiebot_detected', Bool, 
            self.duckiebot_dt_cb, 
            queue_size=1
            )
        # CMD
        self.cmd_pub = rospy.Publisher(
            cmd,
            Twist2DStamped,
            queue_size=1
        )

        self.cmd_sub = rospy.Subscriber(
            cmd,
            Twist2DStamped,
            self.cmd_cb,
            queue_size=1
        )

        # WHEELS_CMD_EXECUTED
        self.wheels_pub = rospy.Publisher(
            wheels_cmd_executed,
            WheelsCmdStamped,
            queue_size=1
        )

        self.wheels_sub = rospy.Subscriber(
            wheels_cmd_executed,
            WheelsCmdStamped,
            self.wheels_cb,
            queue_size=1
        )
    

    def duckie_dt_cb(self, detected):
        print(f'duckie: {detected.data}')
        self.duckie_detect = detected.data

    def duckiebot_dt_cb(self,detected):
        # print(f'duckiebot: {detected.data}')
        self.duckiebot_detect = detected.data
        print(f'duckiebot:{self.duckiebot_detect}')
        
    def cmd_cb(self, twist):
        rospy.loginfo(f'Twist: v={twist.v}, omega={twist.omega}')

    def wheels_cb(self, wheel):
        rospy.loginfo(f'Wheel: left={wheel.vel_left}, right={wheel.vel_right}')

    def stop(self):
        wh = self.get_wheel(0,0)
        tw = self.get_twist(0,0)
        self.move(wheel=wh, twist=tw)
        print('I stopped')


    def run(self):

        rospy.Rate(0.5).sleep()
        wh = self.get_wheel(0.2, 0.2)
        tw = self.get_twist(0.25, 0)
        self.move(wheel=wh, twist=tw)

        # rospy.Rate(0.5).sleep()
        while not rospy.is_shutdown():
            if self.duckiebot_detect:
                self.stop()
                rospy.sleep(1)
                # Move wheels to the left
                # Publish left wheel command
               
                tw = self.get_twist(0.25, 0)
                wheels_cmd = self.get_wheel(0.2, 0.8)
                self.move(wheel=wheels_cmd,twist=tw)
                rospy.sleep(0.5)
                # rospy.Rate(1).sleep()

                # # Move forward for specified duration
                twist_cmd = self.get_twist(0.25, 0)  # Forward motion
                wheels_cmd = self.get_wheel(0.25, 0.2)
                self.move(wheel= wheels_cmd,twist=twist_cmd)

                # # Move wheels to the right
                # # Publish right wheel command
                tw = self.get_twist(0.25,0)
                wheels_cmd = self.get_wheel(0.8, 0.2)
                self.move(wheel=wheels_cmd,twist=tw)

                self.stop()        

    def get_wheel(self, l, r):
        wheel = WheelsCmdStamped()
        wheel.vel_left = l
        wheel.vel_right = r
        return wheel

    def get_twist(self, v, omega):
        twist = Twist2DStamped()
        twist.v = v
        twist.omega = omega
        return twist

    def move(self, wheel, twist):
        self.wheels_pub.publish(wheel)
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    print('started')
    node = WheelsNode(node_name='wheels_node')
    node.run()
    rospy.spin()