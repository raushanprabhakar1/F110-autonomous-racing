#!/usr/bin/env python


import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan







def test(data):
    print(data.data)


def listen():

    rospy.init_node("test")

    rospy.Subscriber('farthest_point', Float64, test)
    rospy.spin()




listen()

