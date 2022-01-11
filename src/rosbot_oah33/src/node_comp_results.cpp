#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Duration.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include<cmath>

ros::Publisher elapse_time_pub, total_dist_pub;

ros::Time simTime;
ros::Time startTime;
std_msgs::Duration elapseTime;

float pX0, pY0, pX1, pY1;
std_msgs::Float64 total_dist;
bool isFirst = true;

std::string state;

/* Distance between two points */
float points2dist(float x0, float y0, float x1, float y1){
    return pow( pow(x1 - x0, 2) + pow(y1 - y0, 2), 0.5 );
}

/* get sim time and store in global variable simTime */
void callback_clock(const rosgraph_msgs::Clock::ConstPtr &msg)
{
    //std::cout << "------Simu Time------\n" << msg->clock << std::endl;
    simTime = msg->clock;
}

/* get sim time and store in global variable simTime */
void callback_pos(const nav_msgs::Odometry &msg)
{
    //std::cout << "------Odometry------\n" << msg << std::endl;

    if (!isFirst){
        // get current coords
        pX1 = msg.pose.pose.position.x;
        pY1 = msg.pose.pose.position.y;

        // get distance, add to total for new total distance
        total_dist.data += points2dist( pX0, pY0, pX1, pY1 );

    } else {isFirst = false;}

    // set previous coords to current (for next iteration)
    pX0 = msg.pose.pose.position.x;
    pY0 = msg.pose.pose.position.y;
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

        // reset total distance to 0 and then say next value is the first
        total_dist.data = 0;
        isFirst = true;
    }
    else if (state == "stop")
    {
        // calculate elapsed time and publish result    
        elapseTime.data = simTime - startTime;

        std::cout << "[INFO]: TOTAL RUN TIME [s]= " << elapseTime.data << std::endl;
        std::cout << "[INFO]: TOTAL DISTANCE [m]= " << total_dist << std::endl;
        
        elapse_time_pub.publish(elapseTime);
        total_dist_pub.publish(total_dist);
    }
}

/* main sub and pub setup */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_comp_results");
    ros::NodeHandle n("~");
    
    // subscriptions
    ros::Subscriber time = n.subscribe("/clock", 10, callback_clock);
    ros::Subscriber dist = n.subscribe("/odom", 10, callback_pos);
    ros::Subscriber setup = n.subscribe("/cmd_setup", 1, callback_setup);

    // publishers
    elapse_time_pub = n.advertise<std_msgs::Duration>("/results/elapsed_time", 10);
    total_dist_pub = n.advertise<std_msgs::Float64>("/results/total_dist", 10);

    ros::Rate loop_rate(20); // this is the RATE, not time
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}