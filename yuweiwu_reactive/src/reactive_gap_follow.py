#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
pre_angle = 0


class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber( lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.drive_pub = rospy.Publisher( drive_topic, AckermannDriveStamped, queue_size=1)
        
    @staticmethod
    def running_mean(x,N):
        cumsum = np.cumsum(np.insert(x,0,0))
        return (cumsum[N:]- cumsum[:-N]) / float(N)


    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)       """
        n = len(ranges)
        #valid = np.arange(270,810)
        #proc_ranges = ranges[valid]
        proc_ranges = self.running_mean(ranges,4)

        proc_ranges[proc_ranges < 1] = 0.0 

        #for i in range(n):
        #   proc_ranges[i] = sum(ranges[i-2:i+3])/5
        #   if ranges[i] >2.7 or i < 270 or i > 810:
        #       proc_ranges[i] = np.nan
           

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        start_i,end_i,best_start,best_end = 0,0,0,0

        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] >1.42:
                end_i += 1
            else:
                if end_i - start_i + 1 > best_end-best_start+1:
                    best_start,best_end = start_i,end_i
                
                start_i = i
                end_i = i

        if end_i - start_i + 1 > best_end-best_start+1:
            return start_i, end_i

        return best_start, best_end
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
       
        _max = np.nanmax(ranges[start_i:end_i])
        if _max != "nan":
            return ranges.index(_max)
        else: int(np.mean([start_i,end_i]))
        


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        global pre_angle
        ranges = data.ranges

        proc_ranges = self.preprocess_lidar(ranges)
        #print(proc_ranges)

        #Find closest point to LiDAR
        index = proc_ranges.index(np.nanmin(proc_ranges))
        min_distance = ranges[index]

        #Eliminate all points inside 'bubble' (set them to zero)
        r = 0.17
        l = ranges[index]
        #print(l)
        if l == 0:
            delta_a = math.asin(0)
        elif l >= r: 
            delta_a = math.asin(r/l)
        elif l < r:
            delta_a = math.asin(1)
            return
       
        angle_range = [data.angle_increment*index - delta_a, data.angle_increment*index + delta_a]

        for i in range(len(proc_ranges)):
            angle_point = data.angle_increment*i   
            if  angle_range[0] <= angle_point <= angle_range[1]:
                proc_ranges[i] = 0


        #Find max length gap 
        start_i, end_i = self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        best_index = self.find_best_point(start_i, end_i, proc_ranges)


        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        if min_distance > 0.48:
            angle = (best_index-540)*data.angle_increment*0.335/min_distance
        else:
            angle = (best_index-540)*data.angle_increment*1.05
        angle = np.clip(angle, -0.41, 0.41)


        new_angle = (pre_angle + angle)/2
        pre_angle = new_angle 
        

        if abs(angle) <= 5*math.pi/180:
            velocity  = 2.6
        elif abs(angle) <= 10*math.pi/180:
            velocity  = 2.4
        elif abs(angle) <= 15*math.pi/180:
            velocity = 2.2
        elif abs(angle) <= 20*math.pi/180:
            velocity = 2
        else:
            velocity = 1.8
        
        
        #print([angle,new_angle, min_distance])

        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "drive"
        drive_msg.drive.speed = velocity

        drive_msg.drive.steering_angle = new_angle 
        self.drive_pub.publish(drive_msg)
        return 

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
