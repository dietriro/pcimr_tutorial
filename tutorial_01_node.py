#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from pcimr_simulation.srv import InitPos


class Node01:
    """
    A node including call of initilization service, move command publisher, position subscriber and scan information subscriber
    """

    def __init__(self):

        # call /initpos to initialize position
        rospy.wait_for_service('init_pos')
        rospy.loginfo('Requesting position initialization')
        try:
            # define service
            init_position_client = rospy.ServiceProxy('init_pos', InitPos)
            # call service
            resp = init_position_client.call(2,0)
            rospy.loginfo('Position initialization success')
        except rospy.ServiceException as e:
            rospy.logwarn('Position initialization failed')

        # Initialize Publisher
        self.move = rospy.Publisher('/move', String, queue_size=10)

        # Initialize Subscribers
        self.pos_sub = rospy.Subscriber('/robot_pos', Point, self.callback_pos)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)

    # callback of scan listener
    def callback_scan(self, LaserScan):
        self.LaserRange = LaserScan.ranges
        rospy.loginfo(f'Listener:Laser Range is:{self.LaserRange}')

    # callback of postion listener
    def callback_pos(self, Point):
        self.pos = [Point.x, Point.y]
        rospy.loginfo(f'Listener:robot position is:{self.pos}')

    # Accordding to situation to publish move command
    def run(self, rate: float = 1):
        n = 0
        while not rospy.is_shutdown():
            # Publish messages
            
            # a random walk at the first step in order to scan the world
            if n==0:
                self.move.publish('N')
            elif self.LaserRange[2]>1.0:
                self.move.publish('N')
            elif self.LaserRange[3]>1.0:
                self.move.publish('E')
            else:
                print('Reach the Goal')
                break
            n += 1

            if rate:
                rospy.sleep(1/rate)

if __name__ == "__main__":
    rospy.init_node('node01')

    node01 = Node01()
    node01.run(rate=1)
