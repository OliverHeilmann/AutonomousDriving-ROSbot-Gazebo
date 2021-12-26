#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

ros::Time simTime;
void myCallback(const rosgraph_msgs::Clock::ConstPtr &msg)
{
    simTime = msg->clock;
    std::cout << "Sim Time: " << msg->clock << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clock_time_subscriber_node");
    ros::NodeHandle n("~");
    ros::Subscriber sub = n.subscribe("/clock", 10, myCallback);
    ros::Rate loop_rate(50); // this is the RATE, not time!
    int counter = 25;
    while (ros::ok())
    {
        ++counter;
        ros::spinOnce();
        loop_rate.sleep();
        if (counter > 49)
        {
            counter = 0;
            std::cout << "Simulation Time: " << simTime << std::endl; 
        }
    }
}