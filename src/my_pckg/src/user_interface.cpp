#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include "std_msgs/String.h"
#include "geographic_msgs/GeoPoint.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "user_interface_talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geographic_msgs::GeoPoint>("/target_location", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        geographic_msgs::GeoPoint msg;

        double latitude, longitude;
        std::cout << "------------------------------------\n";
        std::cout << "for reference, Sarona: lat:[32.072734] long:[34.787465]\n";
        std::cout << "Enter GPS Latitude: ";
        std::cin >> latitude;
        std::cout << "Enter GPS Longitude: ";
        std::cin >> longitude;

        msg.latitude = latitude;
        msg.longitude = longitude;

        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
