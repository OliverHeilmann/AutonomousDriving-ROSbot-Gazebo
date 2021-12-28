#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <string>

float deg2rad = 3.14159265/180;
float rad2deg = 1/deg2rad;
ros::Publisher explore_pub;

float range_fl;
float range_fr;
float range_fl_max;
float range_fr_max;

geometry_msgs::Vector3 pose;

std::string state;

/* read the rpy message and genertae a transform to represent the orientation */
void fl_callback(const sensor_msgs::Range &msg)
{
    //std::cout << "------Front Left------\n" << msg << std::endl;

    range_fl = msg.range;
    range_fl = msg.max_range;
}

/* read the IMU message and generate a pose message like the physical ROSbot */
/* RHW Updated 24-11-20 with more accurate calculation of RPY */
void fr_callback(const sensor_msgs::Range &msg)
{
    //std::cout << "------Front Right------\n" << msg << std::endl;

    range_fr = msg.range;
    range_fl = msg.max_range;
}

/* read the rpy roll, pitch, yaw values [deg] */
void rpy_callback(const geometry_msgs::Vector3 &msg)
{
    //std::cout << "------IMU Degrees-----\n" << msg << std::endl;

    // update pose values
    pose = msg;

    // NOW DO THE BRAITENBERG APPROACH STUFF BELOW AND PUBLISH THE OUTPUTS IF START IS CALLED
    // ELSE DON'T DO IT BUT STILL UPDATE SENSOR READINGS

    // remember condition where rosbot is head on with obstacle
}

/* setup callback as start, stop, reset, _, ... */
void setup_callback(const std_msgs::String &msg)
{
    //std::cout << "------Setup State-----\n" << msg << std::endl;

    // update pose values
    state = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_braitenberg");
    ros::NodeHandle n("~");

    // callback functions
    ros::Subscriber range_fl = n.subscribe("/range/fl", 1, fl_callback);
    ros::Subscriber range_fr = n.subscribe("/range/fr", 1, fr_callback);
    ros::Subscriber pose_rpy = n.subscribe("/rpy", 1, rpy_callback);
    ros::Subscriber setup = n.subscribe("/cmd_setup", 1, setup_callback);

    // publisher node explore
    explore_pub = n.advertise<geometry_msgs::Vector3>("/explore", 10);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}