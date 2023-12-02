
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg){

    ROS_INFO("--- start msg ---");

    // Log header information
    ROS_INFO("Header: seq: [%u], stamp: secs: [%u], nsecs: [%u], frame_id: [%s]",
                msg->header.seq, msg->header.stamp.sec, msg->header.stamp.nsec, msg->header.frame_id.c_str());

    // Log orientation
    ROS_INFO("Orientation: x: [%f], y: [%f], z: [%f], w: [%f]",
                msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    // Log orientation covariance
    ROS_INFO("Orientation Covariance: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
                msg->orientation_covariance[0], msg->orientation_covariance[1], msg->orientation_covariance[2],
                msg->orientation_covariance[3], msg->orientation_covariance[4], msg->orientation_covariance[5],
                msg->orientation_covariance[6], msg->orientation_covariance[7], msg->orientation_covariance[8]);

    // Log angular velocity
    ROS_INFO("Angular Velocity: x: [%f], y: [%f], z: [%f]",
                msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // Log angular velocity covariance
    ROS_INFO("Angular Velocity Covariance: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
                msg->angular_velocity_covariance[0], msg->angular_velocity_covariance[1], msg->angular_velocity_covariance[2],
                msg->angular_velocity_covariance[3], msg->angular_velocity_covariance[4], msg->angular_velocity_covariance[5],
                msg->angular_velocity_covariance[6], msg->angular_velocity_covariance[7], msg->angular_velocity_covariance[8]);

    // Log linear acceleration
    ROS_INFO("Linear Acceleration: x: [%f], y: [%f], z: [%f]",
                msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    // Log linear acceleration covariance
    ROS_INFO("Linear Acceleration Covariance: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
                msg->linear_acceleration_covariance[0], msg->linear_acceleration_covariance[1], msg->linear_acceleration_covariance[2],
                msg->linear_acceleration_covariance[3], msg->linear_acceleration_covariance[4], msg->linear_acceleration_covariance[5],
                msg->linear_acceleration_covariance[6], msg->linear_acceleration_covariance[7], msg->linear_acceleration_covariance[8]);

}


int main(int argc, char **argv){
  ros::init(argc, argv, "imu_subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/imu", 1000, chatterCallback);
  ros::spin();

  return 0;
}
