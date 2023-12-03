
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <nav_msgs/Odometry.h>

// nav_msgs/Odometry
// void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg){
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){

  ROS_INFO("--- start msg ---");

  ROS_INFO("Header: seq: %u, stamp: %u, frame_id: %s", 
          msg->header.seq, 
          msg->header.stamp.sec, // Assuming stamp is a time type
          msg->header.frame_id.c_str());

  ROS_INFO("Pose: [Position: x: %f, y: %f, z: %f], [Orientation: x: %f, y: %f, z: %f, w: %f]", 
          msg->pose.pose.position.x, 
          msg->pose.pose.position.y, 
          msg->pose.pose.position.z,
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);

  ROS_INFO("Twist: [Linear: x: %f, y: %f, z: %f], [Angular: x: %f, y: %f, z: %f]", 
          msg->twist.twist.linear.x, 
          msg->twist.twist.linear.y, 
          msg->twist.twist.linear.z,
          msg->twist.twist.angular.x,
          msg->twist.twist.angular.y,
          msg->twist.twist.angular.z);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "odom_subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/odom", 1000, chatterCallback);
  ros::spin();

  return 0;
}
