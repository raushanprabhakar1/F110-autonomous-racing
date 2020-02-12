#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 1.11
kd = 0.081
ki = 0.0000013

#kp = 2 #TODO
#kd = 0.08 #TODO
#ki = 0.000001 #TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_LEFT = 0.8
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
L = 0.37


class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        #drive_topic =  '/vesc/high_level/ackermann_cmd_mux/input/nav_1'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)


    def getRange(self, data, angle):

        index = (angle+90) * (len(data.ranges)/360)

        #index = max(angle*4-1,0)
        distance = data.ranges[index]

        return distance

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = -kp*error + ki*(integral+ error*servo_offset) + kd*(error-prev_error)/servo_offset
        
        #update
        prev_error = error

        integral = integral + error*servo_offset

        # update the velocity
        if  abs(angle) <= 5*math.pi/180:
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

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"

        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        error = DESIRED_DISTANCE_LEFT - leftDist
        return error

    def followRight(self, data, rightDist):
        error = DESIRED_DISTANCE_RIGHT - rightDist
        return error



    def lidar_callback(self, data):

        global servo_offset
        servo_offset = L*1.0/ VELOCITY
 
        a = self.getRange(data, 135)
        b = self.getRange(data, 180)

        #c = self.getRange(data, 60)
        #d = self.getRange(data, 0)
        theta = math.radians(45)
    
        angle1 = math.atan((a*math.cos(theta)-b)/a*math.sin(theta))
        Dist_t1 = b*math.cos(angle1) + L*math.sin(angle1)

            #angle1 = math.atan((c*math.cos(theta)-d)/c*math.sin(theta))
            #right_Dist_t1 = d*math.cos(angle1) + L*math.sin(angle1)   
            #Dist_t1 = self.getRange(data, 180)*math.cos(-angle1)  + L*math.sin(-angle1)


        #update the error

        leftDist = Dist_t1
        error = self.followLeft(data, leftDist)
        #error = self.followRight(data, rightDist)
        

        #send error to pid_control
        self.pid_control(error, VELOCITY)




def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
