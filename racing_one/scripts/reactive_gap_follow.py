#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        #drive_topic =  '/vesc/high_level/ackermann_cmd_mux/input/nav_1'

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
        proc_ranges = self.running_mean(ranges, 4)
        proc_ranges[proc_ranges < 1] = 0.0   

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """

        non_zeros= np.nonzero(free_space_ranges)
        non_zeros = non_zeros[0]
        #non_zeros = np.sort(non_zeros)  
        non_zeros_2 = np.copy(non_zeros)    
        non_zeros_2[:-1] = non_zeros[1:]
        non_zeros_2[-1] = 0

        result = non_zeros_2 - non_zeros

        out = np.where(result>1)
        # print(result)
        if out[0].shape[0] > 0:
            if out[0][0] +1 >= non_zeros.shape[0] - out[0][0] +1:
                return 0, non_zeros[out[0][0]]
            else:
                return non_zeros[out[0][0]+1], free_space_ranges.shape[0]-1
        else:
            return 0, free_space_ranges.shape[0]-1


    
    def find_best_point(self, start_i, end_i, ranges):

        return (start_i+end_i)//2


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        index = np.argmin(proc_ranges) #  proc_ranges.index(min(proc_ranges))
        min_distance = ranges[index]

        #Eliminate all points inside 'bubble' (set them to zero)
        r = 0.2
        l = ranges[index]
 
        if l == 0:
            delta_a = math.asin(0)
        elif l > r: 
            delta_a = math.asin(r/l)
        elif l < r:
            return
       

        angle_range = [data.angle_increment*index - delta_a, data.angle_increment*index + delta_a]
        #print(angle_range)
        for i in range(len(proc_ranges)):
            angle_point = data.angle_increment*i
            if  angle_range[0] <= angle_point <= angle_range[1]:
                proc_ranges[i] = 0


    
        #Find max length gap 
        start_i, end_i = self.find_max_gap(proc_ranges)
        #print([start_i, end_i])
        

        #Find the best point in the gap 
        best_index = self.find_best_point(start_i, end_i, proc_ranges)

        #print(best_index)

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        angle = (best_index-540)*data.angle_increment
        
        if abs(angle) <= 5*math.pi/180:
            velocity  = 4
        elif abs(angle) <= 10*math.pi/180:
            velocity  = 3.7
        elif abs(angle) <= 15*math.pi/180:
            velocity = 3.5
        elif abs(angle) <= 20*math.pi/180:
            velocity = 3
        else:
            velocity = 2.5

        

        angle = np.clip(angle, -0.43, 0.43)

        #print(angle)
        #print(angle)
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "drive"
        drive_msg.drive.speed = 2

        drive_msg.drive.steering_angle = angle
        self.drive_pub.publish(drive_msg)
        return 

   

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
