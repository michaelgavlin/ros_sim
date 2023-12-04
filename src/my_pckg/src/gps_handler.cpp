#include <ros/ros.h>
#include <iostream>
#include "my_pckg/PoseSimple.h"
#include <GeographicLib/UTMUPS.hpp>

// Global publisher for sending calculated pose information.
ros::Publisher pub;

class GeoToCartesianConverter {
public:
    GeoToCartesianConverter() {
        // Set the origin (reference point)
        originLatitude = 32.072734;
        originLongitude = 34.787465;

        // Convert origin to UTM
        int zone;
        bool northp;
        GeographicLib::UTMUPS::Forward(originLatitude, originLongitude, zone, northp, originX, originY);
    }

    void convert() {

        int inputType;
        double x, y;
        std::cout << "-----------------------------------------------------------\n";
        std::cout << "Enter 1 for GPS coordinates, 2 for Cartesian coordinates:\n";
        std::cin >> inputType;

        if (inputType == 1) { // GPS Coordinates
            std::cout << "Sarona geographic location (origin):  32.072734 , 34.787465\n";
            std::cout << "for debug ref:                        32.072758 , 34.787702\n";
            std::cout << "----\n";
            
            double latitude, longitude;
            std::cout << "Enter latitude: ";
            std::cin >> latitude;
            std::cout << "Enter longitude: ";
            std::cin >> longitude;

            int zone;
            bool northp;

            // Convert the received geolocation to UTM coordinates
            GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, x, y);

            // Translate based on the origin
            x -= originX;
            y -= originY;

            std::cout << "Converted Cartesian coordinates: x = " << x << ", y = " << y << std::endl;
        } else if (inputType == 2) { // Cartesian Coordinates
            std::cout << "Enter Cartesian x: ";
            std::cin >> x;
            std::cout << "Enter Cartesian y: ";
            std::cin >> y;
        // Assuming Cartesian coordinates are already relative to the origin
        
        } else {
            std::cout << "Invalid input. Please enter 1 for GPS or 2 for Cartesian coordinates.\n";
            return;
        }

        // publish requested coordination in cartesian relative to Sarona
        my_pckg::PoseSimple msg_;
        msg_.linear_x = x;
        msg_.linear_y = y;
        pub.publish(msg_);

    }

private:
    double originLatitude, originLongitude, originX, originY;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_handler_node");
    ros::NodeHandle n;

    pub = n.advertise<my_pckg::PoseSimple>("/target_destination", 10);

    GeoToCartesianConverter converter;

    while (ros::ok()) {
        converter.convert();
    }

    return 0;
}
