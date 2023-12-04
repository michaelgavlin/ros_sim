#include <ros/ros.h>
#include <iostream>
#include "my_pckg/PoseSimple.h"
#include <nav_msgs/Odometry.h>
#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/NavSatFix.h>

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


    void convert() {
        double x, y;
        int zone;
        bool northp;

        // Convert the received geolocation to UTM coordinates
        GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, x, y);

        // Translate based on the origin
        x -= originX;
        y -= originY;

        ROS_INFO("Converted Cartesian coordinates target: [ x: %f, y: %f]", x, y);

        // publish requested coordination in cartesian relative to Sarona
        my_pckg::PoseSimple msg_;
        msg_.linear_x = x;
        msg_.linear_y = y;
        pub.publish(msg_);
    }

private:
    double originLatitude, originLongitude, originX, originY;
    int zone;
    bool northp;
};

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;
    GeoToCartesianConverter converter;

    ROS_INFO("--- msg --- ");
    ROS_INFO("Current Position: [x: %f, y: %f]", current_x, current_y);

    converter.convert();

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_handler_node");
    ros::NodeHandle n;

    pub = n.advertise<my_pckg::PoseSimple>("/target_destination", 10);
    pub_sim_gui = n.advertise<sensor_msgs::NavSatFix>("/robot_location", 10);
    ros::Subscriber sub = n.subscribe("/odom", 1000, odomCallback);

    ros::spin();

    return 0;
}
