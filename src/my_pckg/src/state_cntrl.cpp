#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <cmath>

// Global publisher
ros::Publisher vel_pub;

// Target position
const double target_x = 2.0;
const double target_y = 1.0;
double angle_to_target_value;
double distance_to_target;

void publish_movement(double distance, double angular_change) {
    const double max_linear_speed = 0.2; // Maximum linear speed
    const double linear_threshold = 0.05; // Threshold to stop linear movement
    const double angular_threshold = 0.1; // Threshold to stop rotation

    geometry_msgs::Twist movement_msg;

    // Linear movement control
    double linear_speed = distance; // Assuming 'distance' is proportional to desired linear speed
    linear_speed = std::min(linear_speed, max_linear_speed); // Clamp linear speed

    if (fabs(distance) > linear_threshold) {
        movement_msg.linear.x = linear_speed;
    } else {
        movement_msg.linear.x = 0; // Stop linear movement
    }

    // Angular movement control
    double angular_speed = angular_change * 3; // Proportional control for angular speed
    if (fabs(angular_change) > angular_threshold) {
        movement_msg.angular.z = angular_speed;
    } else {
        movement_msg.angular.z = 0; // Stop rotation
    }

    vel_pub.publish(movement_msg);
}


double calculate_angle_to_target(double target_x, double target_y, double current_x, double current_y) {
    // Angle of the vector between current and dest
    double angle = atan2(target_y - current_y, target_x - current_x);
    return angle; // Angle in radians
}

double dist_to_target(double target_x, double target_y, double current_x, double current_y) {
    // Distance location and destination
    double distance = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
    return distance;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // Calculate angle and distance to target
    angle_to_target_value = calculate_angle_to_target(target_x, target_y, current_x, current_y);
    distance_to_target = dist_to_target(target_x, target_y, current_x, current_y);

    // Extract the robot's current yaw orientation from the quaternion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // Yaw is the orientation around Z-axis

    // Calculate the required angular change
    double angular_change = atan2(sin(angle_to_target_value - yaw), cos(angle_to_target_value - yaw));

    ROS_INFO("Distance to target: %f", distance_to_target);
    ROS_INFO("Pose: [Position: x: %f, y: %f]", current_x, current_y);
    ROS_INFO("Robot Orientation (Yaw): %f", yaw);
    ROS_INFO("Angle to Target: %f", angle_to_target_value);
    ROS_INFO("Required Angular Change: %f", angular_change);

    publish_movement(distance_to_target,angular_change);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_subscriber");
    ros::NodeHandle n;

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber sub = n.subscribe("/odom", 1000, odomCallback);

    ros::spin();

    return 0;
}