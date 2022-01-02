#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include<cmath>

ros::Publisher lidar_headings;

std::string state;

float lidar_array[720];

/* callback for scan data */
void callback_scan(const sensor_msgs::LaserScan &msg){

    // 

    for ( n=0 ; n<lidar_array.size() ; n++ )
    {
        result += lidar_array[n];
    }
    cout << result;
}

/* setup callback as start, stop, reset, _, ... */
void callback_setup(const std_msgs::String &msg)
{
    // update pose values
    state = msg.data;

    if (state == "start")
    {

    }
}

/* main sub and pub setup */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_lidar");
    ros::NodeHandle n("~");
    
    // subscriptions
    ros::Subscriber time = n.subscribe("/scan", 720, callback_scan);
    ros::Subscriber setup = n.subscribe("/cmd_setup", 1, callback_setup);

    // publishers
    lidar_headings = n.advertise<std_msgs::Float64>("/heading/lidar", 10);

    ros::Rate loop_rate(20); // this is the RATE, not time
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}