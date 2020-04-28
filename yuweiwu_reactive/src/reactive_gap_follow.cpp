#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>

class ReactiveFollowGap {

private:
    ros::NodeHandle n;
    double speed;

public:
    ReactiveFollowGap() {
        n = ros::NodeHandle();
        speed = 0.0;
        // TODO: create ROS subscribers and publishers
        ros::Subscriber lidar_sub = n.subscribe("/scan", 100, &scan_callback::callback);
        ros::Publisher drive_pub  = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 100);  
    }



    void preprocess_lidar(const sensor_msgs::LaserScan::ranges &ranges) {
          /*Preprocess the LiDAR scan array. Expert implementation includes:
           1.Setting each value to the mean over some window
           2.Rejecting high values (eg. > 3m)
          */

        proc_ranges = ranges;
        return proc_ranges;

    }

    void find_max_gap(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        //Return the start index & end index of the max gap in free_space_ranges


        return 0;
    }
    void find_best_point(const start_i, const start_j, const sensor_msgs::LaserScan::ranges &ranges) {
        /*Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        */

        return 0;
    }




    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        /*Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        */


        //Find closest point to LiDAR
        sensor_msgs::LaserScan::ranges ranges;
        ranges = scan_msg::ranges
        proc_ranges = preprocess_lidar(ranges);
         
        //Eliminate all points inside 'bubble' (set them to zero) 



        //Find max length gap 



        //Find the best point in the gap 



        //Publish Drive message
        ackermann_msgs::AckermannDriveStamped acker_msg;
        acker_msg.
        chatter_pub.publish(acker_msg);



    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "FollowGap_node");
    ReactiveFollowGap rfgs;
    ros::spin();
    return 0;
}
