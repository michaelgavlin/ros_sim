#include <cmath>
#include <tf/tf.h>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "my_pckg/PoseSimple.h"  // Include my custom message header.

ros::Publisher vel_pub; // Global publisher for sending movement commands.

/**
 * Publishes movement commands based on provided distance and angular change.
 *  
 * param distance: The desired distance to move. This is used to control linear motion.
 * param angular_change: The desired change in orientation. This is used to control rotational motion.
 */
void publish_movement(double distance, double angular_change) {
    const double max_linear_speed = 0.25; // Maximum linear speed.
    const double linear_threshold = 0.05; // Threshold below which linear movement is stopped.
    const double angular_threshold = 0.05; // Threshold below which rotational movement is stopped.

    // Manual tweak parameters
    const double linear_speed_scaling_factor = 0.01; // Scale the proportional linear speed
    const double angular_speed_scaling_factor = 1; // Scale the proportional angular speed

    geometry_msgs::Twist movement_msg;

    // Linear movement control with proportional speed and manual tweak
    double linear_speed;
    if (fabs(distance) > linear_threshold) {
        linear_speed = std::min(max_linear_speed * (fabs(distance) / linear_threshold) * linear_speed_scaling_factor, max_linear_speed);
        movement_msg.linear.x = distance > 0 ? linear_speed : -linear_speed; // Ensure correct direction
    } else {
        movement_msg.linear.x = 0; // Stop linear movement.
    }

    // Angular movement control with manual tweak
    double angular_speed = angular_change * angular_speed_scaling_factor; // Proportional control for angular speed with scaling.
    if (fabs(angular_change) > angular_threshold) {
        movement_msg.angular.z = angular_speed; // Set angular speed for rotation.
    } else {
        movement_msg.angular.z = 0; // Stop rotation.
    }

    ROS_INFO("Publish to robot: angular [%f] , linear [%f]", 
    movement_msg.angular.z, movement_msg.linear.x);

    vel_pub.publish(movement_msg); // Publish the movement command.
}

/**
 * Callback function for the controller topic subscriber.
 * Extracts linear and angular values from the PoseSimple message and calls publish_movement.
 * 
 * param msg: PoseSimple message containing linear and angular values.
 */
void controllerCallback(const my_pckg::PoseSimple& msg) {
    ROS_INFO("Linear X: %f, Angular Z: %f", msg.linear_x, msg.angular_z);
    publish_movement(msg.linear_x, msg.angular_z); // Call to publish movement commands based on received msg.
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_node"); // Initialize the ROS node.
    ros::NodeHandle n;

    // Subscribe to the /controller topic and advertise on /cmd_vel.
    ros::Subscriber sub = n.subscribe("/controller", 1000, controllerCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::spin(); // Keep the node running and listening for callbacks.

    return 0;
}
