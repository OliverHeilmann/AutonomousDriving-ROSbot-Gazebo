#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sstream>

void myCallback(const std_msgs::Int32::ConstPtr &msg)
{
    std::cout << "Chatter : " << msg->data << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tutorial_subscriber_node");
    ros::NodeHandle n("~");
    ros:: Subscriber sub = n.subscribe("/talker/chatter", 10, myCallback);
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}