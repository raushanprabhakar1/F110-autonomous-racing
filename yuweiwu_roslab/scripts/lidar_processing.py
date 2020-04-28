#!/usr/bin/env python

"""
Created on Wed Jan 20 20:53:20 2020

@author: yuweiwu

Usage: This is the script to create the node lidar_processing
and three topic:closest_point, farthest_point and scan_range
"""
import rospy
#import math
import numpy as np
import std_msgs.msg
from sensor_msgs.msg import LaserScan
from yuweiwu_roslab.msg import scan_range


def lidar_processing():
    #initial the node
    rospy.init_node("lidar_processing", anonymous = True)

    #create topic closest_point
    closest_pub = rospy.Publisher('closest_point', std_msgs.msg.Float64, queue_size = 10)

    #create topic farthest_point
    farthest_pub = rospy.Publisher('farthest_point', std_msgs.msg.Float64, queue_size = 10)

    #create topic scan_range
    scan = rospy.Publisher('scan_range', scan_range, queue_size = 10)

    def callback(msg):
        #rate = rospy.Rate(1) # if want to control it

        data = scan_range()
        data.header = std_msgs.msg.Header(stamp = rospy.Time.now(), frame_id="base")

        #if not math.isnan(max(msg.ranges)) and not math.isinf(max(msg.ranges)):
        #we can use isnan and isinf to check the data if needed
        data.scan_max = np.float64(max(msg.ranges))

        #if not math.isnan(min(msg.ranges)) and not math.isinf(min(msg.ranges)):
        data.scan_min = np.float64(min(msg.ranges))
        
        # publish all
        closest_pub.publish(data.scan_min)
        farthest_pub.publish(data.scan_max)
        scan.publish(data)

    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()

if __name__ == "__main__":
    lidar_processing()
