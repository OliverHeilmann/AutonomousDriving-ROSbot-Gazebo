#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Duration.h>
#include <std_msgs/String.h>

ros::Time simTime;
ros::Time startTime;
std_msgs::Duration elapseTime;

ros::Publisher elapse_time_pub;

std::string state;

/* get sim time and store in global variable simTime */
void callback_clock(const rosgraph_msgs::Clock::ConstPtr &msg)
{
    //std::cout << "------Simu Time------\n" << msg->clock << std::endl;
    simTime = msg->clock;
}

/* setup callback as start, stop, reset, _, ... */
void callback_setup(const std_msgs::String &msg)
{
    // update pose values
    state = msg.data;

    if (state == "start")
    {
        // store current time as start time of run
        startTime = simTime;
    }
    else if (state == "stop")
    {
        // calculate elapsed time and publish result    
        elapseTime.data = simTime - startTime;

        std::cout << "[INFO]: TOTAL RUN TIME = " << elapseTime.data << std::endl;
        
        elapse_time_pub.publish(elapseTime);
    }
}

/* main sub and pub setup */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_clock_time");
    ros::NodeHandle n("~");
    
    // subscriptions
    ros::Subscriber sub = n.subscribe("/clock", 10, callback_clock);
    ros::Subscriber setup = n.subscribe("/cmd_setup", 1, callback_setup);

    // publishers
    elapse_time_pub = n.advertise<std_msgs::Duration>("/elapsed_time", 10);

    ros::Rate loop_rate(20); // this is the RATE, not time
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}