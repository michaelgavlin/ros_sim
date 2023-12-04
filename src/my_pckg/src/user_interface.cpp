#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include "std_msgs/String.h"
#include "my_pckg/PoseSimple.h"
#include "geographic_msgs/GeoPoint.h"


// Simple user interface
int main(int argc, char **argv)
{
    ros::init(argc, argv, "user_interface_talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geographic_msgs::GeoPoint>("/target_location", 1000); //publisher to move the robot to lat / long location
    ros::Publisher debug_pub = n.advertise<my_pckg::PoseSimple>("/target_destination", 10); //publisher to move the robot to x,y location

    ros::Rate loop_rate(10);


    while (ros::ok())
    {
        geographic_msgs::GeoPoint msg;

        double latitude, longitude;
        int in;
        std::cout << "------------------------------------\n";
        std::cout << "Enter 1 for GPS input [lat/long] \n";
        std::cout << "Enter 2 for cartesian input (DEBUG) [x/y] \n";
        std::cin >> in;

        if (in==1){
            std::cout << "for reference, Sarona: lat:[32.072734] long:[34.787465]\n";
            std::cout << "Enter GPS Latitude: ";
            std::cin >> latitude;
            std::cout << "Enter GPS Longitude: ";
            std::cin >> longitude;

            msg.latitude = latitude;
            msg.longitude = longitude;

            chatter_pub.publish(msg);
            ros::spinOnce();
        }
        else if (in==2)
        {
            // publish requested coordination in cartesian 
            my_pckg::PoseSimple requested_coordination_msg;
            double destination_x , destination_y;
            std::cout << "Enter x destination [m]:";
            std::cin >> destination_x;
            std::cout << "Enter y destination [m]:";
            std::cin >> destination_y;

            requested_coordination_msg.linear_x = destination_x;
            requested_coordination_msg.linear_y = destination_y;
            debug_pub.publish(requested_coordination_msg);
            ros::spinOnce();
        }
    
    }

    return 0;
}
