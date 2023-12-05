#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "my_pckg/PoseSimple.h"  // Include my custom message header.

ros::Publisher movement_pub;  // Global publisher for movement commands
float global_min_distance = std::numeric_limits<float>::max();
float direction_parameter = 0.0; // New parameter for direction

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int num_readings = scan->ranges.size();
    global_min_distance = scan->range_max; // Reset to max range at each callback
    direction_parameter = 0.0; // Reset direction parameter

    // Define the angular window size (±30 degrees in radians)
    float window_size = 45 * 3.14 / 180.0; 

    for(int i = 0; i < num_readings; i++) {
        float angle = scan->angle_min + i * scan->angle_increment;

        // Check if the angle is within the ±2.5-degree window
        if (angle >= -window_size && angle <= window_size) {
            // If within the window, check if this is the closest object so far
            if(scan->ranges[i] < global_min_distance) {
                global_min_distance = scan->ranges[i];
                global_min_distance = global_min_distance; // Scale to meet gps map scaler 
                // Set direction_parameter based on the side of the object
                direction_parameter = (angle > 0) ? 1.0 : -1.0;
            }
        }
    }
    ROS_INFO("---------");
    ROS_INFO("Laser scan received with %d points in the ±2.5-degree window", num_readings);
    ROS_INFO("Closest obstacle in the window at: %f meters, Direction: %f", 
             global_min_distance, direction_parameter);

}

void controllerCallback(const my_pckg::PoseSimple& msg) {
    ROS_INFO("Received Linear X: %f, Angular Z: %f", msg.linear_x, msg.angular_z);

    // Check for division by zero
    if(global_min_distance == 0) {
        ROS_WARN("Min distance is zero. Avoiding division by zero.");
        return;
    }

    // Modify the message based on the direction parameter
    my_pckg::PoseSimple modified_msg = msg;

    // logic function - modify angle 
    float threashold = 0.5;
    if(global_min_distance < threashold) {
    // modified_msg.angular_z += (direction_parameter * 1/ (global_min_distance * proportion));
    float mag = 1;
    modified_msg.angular_z += - direction_parameter * (1 + threashold - global_min_distance) * 60 * 3.14 / 180 * mag;
    modified_msg.linear_x = 0.1;
    }

    // Publish the modified message
    movement_pub.publish(modified_msg);

    ROS_INFO("Published Modified Linear X: %f, Modified Angular Z: %f",
             modified_msg.linear_x, modified_msg.angular_z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_reader");
    ros::NodeHandle n;

    // Subscriber for laser scan data
    ros::Subscriber laser_sub = n.subscribe("/scan", 1000, scanCallback);

    // Subscriber for controller data
    ros::Subscriber controller_sub = n.subscribe("/final_destination", 1000, controllerCallback);

    // Publisher for controller commands (assuming the topic is /movement_commands)
    movement_pub = n.advertise<my_pckg::PoseSimple>("/controller", 1000);

    ros::spin();

    return 0;
}
