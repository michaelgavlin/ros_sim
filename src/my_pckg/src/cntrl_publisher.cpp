#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;
    

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate rate(10);

    while (ros::ok()) {
        geometry_msgs::Twist msg;
        msg.linear.x = 5.0;
        msg.angular.z = 1.0;
        
        ROS_INFO("Linear X: %f, Angular Z: %f", msg.linear.x, msg.angular.z);

        pub.publish(msg);
        rate.sleep();
    }

    return 0;
}