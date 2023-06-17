#!/usr/bin/env python3

import os
import rospy
import json
import yaml
import time
from std_srvs.srv import EmptyResponse, Empty
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped 
from std_msgs.msg import String, Bool

HOST = os.environ['VEHICLE_NAME']
cmd = f'/{HOST}/car_cmd_switch_node/cmd'
velocity = f'/{HOST}/kinematics_node/velocity'
wheels_cmd_executed = f'/{HOST}/wheels_driver_node/wheels_cmd_executed'
car_cmd = f'/{HOST}/joy_mapper_node/car_cmd'

class WheelsNode(DTROS):
    def __init__(self, node_name):
        super(WheelsNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # self.is_obstacle_detected = False
        self.wh = self.get_wheel(0.4, 0.4)
        self.tw = self.get_twist(0.4, 0)

        self.pub_cmd = rospy.Publisher(
            cmd,
            Twist2DStamped,
            queue_size=1,
        )

        self.pub_velocity = rospy.Publisher(
            velocity,
            Twist2DStamped,
            queue_size=1,
        )

        self.pub_wheels_cmd = rospy.Publisher(
            wheels_cmd_executed,
            WheelsCmdStamped,
            queue_size=1
        )

        self.sub_car_cmd = rospy.Subscriber(
            car_cmd,
            Twist2DStamped,
            self.car_cmd_cb
        )

        self.pub_car_cmd = rospy.Publisher(
            car_cmd,
            Twist2DStamped,
            queue_size=1
        )

        self.sub_detected = rospy.Subscriber(
            'duckiebot_detected',
            Bool,
            self.det_cb,
            queue_size=1
        )

        self.duckie_detected = rospy.Subscriber(
            'duckie_detected',
            Bool,
            self.duckie_cb,
            queue_size=1
        )
        self.duckie_detect = False
        self.detect = False

    def stop(self):
        wh = self.get_wheel(0,0)
        tw = self.get_twist(0,0)
        self.move(wheel=wh, twist=tw)

    def det_cb(self, b):
        # rospy.loginfo(f'detect: {b}')
        self.detect = b.data
    def duckie_cb(self,b):
        rospy.loginfo(f'detect: {b}')
        self.duckie_detect = b.data
    
        
    def run(self):
        rospy.sleep(2)
        wh = self.get_wheel(0.2, 0.2)
        tw = self.get_twist(0.25, 0)
        self.move(wh, tw)

        while not rospy.is_shutdown():
            if self.detect or self.duckie_detect:
                self.stop()
                rospy.sleep(1)
                self.avoid_car()
                rospy.sleep(1)
                self.stop()
                break
        rospy.sleep(2)
        self.stop()


    def get_wheel(self, left, right):
        wheel = WheelsCmdStamped()
        wheel.vel_left = left
        wheel.vel_right = right
        return wheel
    
   

    def avoid_car(self):
        # left
        tw = self.get_twist(0.3, 2)
        wh = self.get_wheel(0.1, 0.7)
        self.move(wh, tw)

        rospy.sleep(1)
        self.stop()

        # right 
        tw = self.get_twist(0.4, -3)
        wh = self.get_wheel(0.7, 0.1)
        self.move(wh, tw)

        rospy.sleep(1)
        self.stop()

        # right        
        tw = self.get_twist(0.3, -3.5)
        wh = self.get_wheel(0.7, 0.1)
        self.move(wh, tw)

        rospy.sleep(0.6)
        self.stop()

        # left        
        tw = self.get_twist(0.45, 4)
        wh = self.get_wheel(0.1, 0.7)
        self.move(wh, tw)

        rospy.sleep(1)
        self.stop()


    def get_twist(self, v, omega):
        twist = Twist2DStamped()
        twist.v = v
        twist.omega = omega
        return twist

    def move(self, wheel, twist):
        self.pub_wheels_cmd.publish(wheel)
        self.pub_cmd.publish(twist)
    
    def car_cmd_cb(self, twist):
        twist.v = self.trim(
            value=twist.v,
            low=-self._v_max.value, 
            high=self._v_max.value
        )

        twist.omega = self.trim(
            value=twist.omega,
            low=-self._omega_max.value,
            high=self._omega_max.value
        )

        # setting motor constants
        k_r = k_l = self._k

        # adjust k by gain and trim
        k_r_inv = (self._gain.value + self._trim.value) / k_r
        k_l_inv = (self._gain.value - self._trim.value) / k_l

        omega_r = (twist.v + 0.5 * twist.omega * self._baseline.value) / self._radius.value
        omega_l = (twist.v - 0.5 * twist.omega * self._baseline.value) / self._radius.value

        # conversion from motor rotation rate to duty cycle
        u_r = omega_r * k_r_inv
        u_l = omega_l * k_l_inv

        # limiting output
        u_r_lim = self.trim(
            value=u_r,
            low=-self._limit.value,
            high=self._limit.value
        )
        
        u_l_lim = self.trim(
            value=u_l,
            low=-self._limit.value,
            high=self._limit.value
        )

        # publishing wheel
        wh = self.get_wheel(l=u_l_lim, r=u_r_lim)
        wh.header.stamp = twist.header.stamp
        self.pub_wheels_cmd.publish(wh)

        # conversion from motor duty to motor rotation rate
        omega_r = wh.vel_right / k_r_inv
        omega_l = wh.vel_left / k_l_inv

        # velocity calculation
        v = (self._radius.value * omega_r + self._radius.value * omega_l) / 2.0
        omega = (self._radius.value * omega_r + self._radius.value * omega_l) / self._baseline.value 

        # publishing velocity
        tw = self.get_twist(v=v, omega=omega)
        tw.header = twist.header
        self.pub_velocity.publish(tw)

 

if __name__ == '__main__':
    print('started')
    node = WheelsNode(node_name='wheels_node')
    node.run()
    rospy.spin()