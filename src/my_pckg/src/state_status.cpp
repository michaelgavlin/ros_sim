#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "my_pckg/PoseSimple.h"

#include <tf/tf.h>
#include <cmath>

// Global publisher for sending calculated pose information.
ros::Publisher pub;

// Target position constants.
const double target_x = 2.0;
const double target_y = 1.0;

// Variables to store calculated angle and distance to target.
double angle_to_target_value;
double distance_to_target;

/**
 * Calculate the angle to the target location from the current position.
 * 
 * param target_x X-coordinate of the target location.
 * param target_y Y-coordinate of the target location.
 * param current_x X-coordinate of the current location.
 * param current_y Y-coordinate of the current location.
 * return Angle in radians to the target location from the current position.
 */
double calculate_angle_to_target(double target_x, double target_y, double current_x, double current_y) {
    double angle = atan2(target_y - current_y, target_x - current_x);
    return angle;
}

/**
 * Calculate the distance to the target location from the current position.
 * 
 * param target_x X-coordinate of the target location.
 * param target_y Y-coordinate of the target location.
 * param current_x X-coordinate of the current location.
 * param current_y Y-coordinate of the current location.
 * @eturn Euclidean distance to the target location from the current position.
 */
double dist_to_target(double target_x, double target_y, double current_x, double current_y) {
    double distance = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
    return distance;
}

/**
 * Callback function for odometry messages. Calculates the necessary pose 
 * information to reach a predefined target and publishes it.
 * 
 * param msg The received odometry message.
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    angle_to_target_value = calculate_angle_to_target(target_x, target_y, current_x, current_y);
    distance_to_target = dist_to_target(target_x, target_y, current_x, current_y);

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double angular_change = atan2(sin(angle_to_target_value - yaw), cos(angle_to_target_value - yaw));

    ROS_INFO("--- msg --- ");
    ROS_INFO("Distance to target: %f", distance_to_target);
    ROS_INFO("Pose: [Position: x: %f, y: %f]", current_x, current_y);
    ROS_INFO("Robot Orientation (Yaw): %f", yaw);
    ROS_INFO("Angle to Target: %f", angle_to_target_value);
    ROS_INFO("Required Angular Change: %f", angular_change);

    my_pckg::PoseSimple msg_;
    msg_.linear_x = distance_to_target;
    msg_.angular_z = angular_change;
    pub.publish(msg_);
}

/**
 * Main function to initialize the ROS node and set up the publisher and subscriber.
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_subscriber");
    ros::NodeHandle n;
     
    pub = n.advertise<my_pckg::PoseSimple>("/controller", 10);
    ros::Subscriber sub = n.subscribe("/odom", 1000, odomCallback);

    ros::spin(); // Keep the node running and listening for callbacks.

    return 0;
}
