#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
    //Creating the node called talker
    ros::init(argc, argv, "talker");
    //Interface for creating nodes,
    ros::NodeHandle n("~");
    //Creates a Publisher node on the  chatter topic
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    //Loops at 10hz
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        std_msgs::String msg;
        std::stringstream ss;
        //Creates the message to pubish
        ss << "hello world, count : " << count;
        //Converts the stringsteam into a msg
        msg.data = ss.str();
        //Ros debug
        ROS_INFO("%s", msg.data.c_str());
        //Publish the message
        chatter_pub.publish(msg);
        //Sleeps for remaining time
        loop_rate.sleep();
        ++count;
    }
    return 0;
}