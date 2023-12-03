
// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "geometry_msgs/Twist.h"
// #include <sstream>


// int main(int argc, char **argv)
// {

//   ros::init(argc, argv, "turtle_move");
//   ros::NodeHandle n;
//   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

//   ros::Rate loop_rate(10);

//   int count = 0;
//   while (ros::ok())
//   {

//     std_msgs::String msg;

//     std::stringstream ss;
//     ss << "hello world " << count;
//     msg.data = ss.str();

//     ROS_INFO("%s", msg.data.c_str());

//     chatter_pub.publish(msg);

//     ros::spinOnce();
//     loop_rate.sleep();
//     ++count;
//   }


//   return 0;
// }
// // %EndTag(FULLTEXT)%





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