#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, Header

# TODO: import ROS msg types and libraries

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0 
        
        # TODO: create ROS subscribers and publishers.
        rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)
        
        self.brake_bool = rospy.Publisher('brake_bool', Bool, queue_size = 1)
        self.brake = rospy.Publisher('brake', AckermannDriveStamped, queue_size = 1)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        bool_data = Bool()
        bool_data.data = False
        brake = AckermannDriveStamped()
        brake.header = Header(stamp = rospy.Time.now(), frame_id="brake")

        T = 0.4
        min_ttc = float("inf")
        for i in range(len(scan_msg.ranges)):
            distance = scan_msg.ranges[i]
            angle = scan_msg.angle_min + scan_msg.angle_increment*(i+1)
            ri_d = self.speed*math.cos(angle)
            if ri_d > 0.00001:
                ttc = distance*1.0/ri_d
                if ttc< min_ttc:
                    min_ttc = ttc
        if min_ttc <= T:
            bool_data.data = True
            brake.drive.speed = 0.0 
            self.brake.publish(brake)

        # TODO: publish brake message and publish controller bool
        
        self.brake_bool.publish(bool_data)
        


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
    
if __name__ == '__main__':
    main()
