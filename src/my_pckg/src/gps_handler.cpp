#include <ros/ros.h>
#include <iostream>
#include "my_pckg/PoseSimple.h"
#include <nav_msgs/Odometry.h>
#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/NavSatFix.h>
#include "geographic_msgs/GeoPoint.h"


// Global publisher for sending calculated pose information.
ros::Publisher pub;
ros::Publisher pub_sim_gui;

// Global variables for target position.
double latitude = 32.072734;
double longitude = 34.787465;

class GeoToCartesianConverter {
public:
    GeoToCartesianConverter() {
        // Set the origin (reference point)
        originLatitude = 32.072734;
        originLongitude = 34.787465;
        // Convert origin to UTM
        GeographicLib::UTMUPS::Forward(originLatitude, originLongitude, zone, northp, originX, originY);
    }

    // Function to convert Cartesian coordinates back to latitude and longitude
    void cartesianToLatLong(double x, double y, double& latitude, double& longitude) {
        x += originX;
        y += originY;
        // Convert back from UTM coordinates to latitude and longitude
        GeographicLib::UTMUPS::Reverse(zone, northp, x, y, latitude, longitude);
    }

    // Calculate the destination relatinve to origin (Sarona)
    void destination_in_cartesian_calc(double& x, double& y) {
        int zone;
        bool northp;
        // Convert the received geolocation to UTM coordinates
        GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, x, y);
        // Translate based on the origin
        x -= originX;
        y -= originY;
    }

private:
    double originLatitude, originLongitude, originX, originY;
    int zone;
    bool northp;
};


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("--- msg --- "); 

    // GeoToCartesianConverter converter;
    GeoToCartesianConverter converter;

    // read msg from /odom topic with the current location relative to origin Sarona.
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;
    ROS_INFO("Current Position: [x: %f, y: %f]", current_x, current_y);

    // Convert current Cartesian coordinates to latitude and longitude
    double current_latitude, current_longitude;
    converter.cartesianToLatLong(current_x, current_y, current_latitude, current_longitude);
    ROS_INFO("Current GPS location: [lat: %f, long: %f]", current_latitude, current_longitude);

    // publish current location to simGUI
    sensor_msgs::NavSatFix msg_to_gui;
    msg_to_gui.latitude = current_latitude;
    msg_to_gui.longitude = current_longitude;
    pub_sim_gui.publish(msg_to_gui);
}


// Callback for handling user input of target location
void userCallback(const geographic_msgs::GeoPoint::ConstPtr& msg) {

    // GeoToCartesianConverter converter;
    GeoToCartesianConverter converter;

    latitude = msg->latitude;    // Correctly access latitude
    longitude = msg->longitude;  // Correctly access longitude
    ROS_INFO("Received GPS destination: [lat: %f, long: %f]", latitude, longitude);

    // Calculate destination coordination in relative to origin Sarona.
    double destination_x, destination_y;
    converter.destination_in_cartesian_calc(destination_x, destination_y);
    ROS_INFO("Converted Cartesian coordinates target: [ x: %f, y: %f]", destination_x, destination_y);

    // publish requested coordination in cartesian 
    my_pckg::PoseSimple requested_coordination_msg;
    requested_coordination_msg.linear_x = destination_x;
    requested_coordination_msg.linear_y = destination_y;
    pub.publish(requested_coordination_msg);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_handler_node");
    ros::NodeHandle n;

    pub = n.advertise<my_pckg::PoseSimple>("/target_destination", 10); //publisher to move the robot to x,y location
    pub_sim_gui = n.advertise<sensor_msgs::NavSatFix>("/robot_location", 10);
    ros::Subscriber sub = n.subscribe("/odom", 1000, odomCallback);
    ros::Subscriber user_sub = n.subscribe("/target_location", 1000, userCallback);

    ros::spin();

    return 0;
}
