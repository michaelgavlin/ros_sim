#include <tf/tf.h>
#include <cmath>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

// Global publisher and subscriber.
ros::Publisher pub;
ros::Subscriber sub;

// Global scale factor, use this to scale the location
double scale_factor = 10.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // scaling or transforming the odometry data
    nav_msgs::Odometry scaled_msg = *msg;
    
    // Scale the linear x and y components
    scaled_msg.pose.pose.position.x *= scale_factor;
    scaled_msg.pose.pose.position.y *= scale_factor;

    ROS_INFO("---"); 
    ROS_INFO("Received odometry: linear x: %f, linear y: %f", 
             msg->pose.pose.position.x, msg->pose.pose.position.y);
    ROS_INFO("Scaled odometry: linear x: %f, linear y: %f", 
             scaled_msg.pose.pose.position.x, scaled_msg.pose.pose.position.x);

    // Publish the modified message
    pub.publish(scaled_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "location_scaler_node");
    ros::NodeHandle n;
   
    // Subscribe to odometry messages and set up the publisher
    sub = n.subscribe("/odom", 1000, odomCallback);
    pub = n.advertise<nav_msgs::Odometry>("/odom_scaled", 1000); //publisher to move the robot to lat / long location

    ros::spin(); // Keep the node running and listening for callbacks.

    return 0;
}
