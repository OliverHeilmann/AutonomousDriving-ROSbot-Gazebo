#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <string>

float deg2rad = 3.14159265/180;
float rad2deg = 1/deg2rad;
ros::Publisher explore_pub;

float range_fl;
float range_fr;
float range_fl_max;
float range_fr_max;
float range_fl_min;
float range_fr_min;

geometry_msgs::Vector3 pose;

float trip_thresh = 0.95; // larger means avoidance measures will happen closer to when sensor reads its max range (deals with sensor noise)
float bberg_weight = 90; // weight for bberg sensor componenet [this says max movement is 90 deg]
std_msgs::Float64 dTheta_yaw;

float yaw_start = 0;
bool start_trigger = true;

std::string state;

/* read the rpy message and genertae a transform to represent the orientation */
void callback_fl(const sensor_msgs::Range &msg)
{
    //std::cout << "------Front Left------\n" << msg << std::endl;

    range_fl = msg.range;
    range_fl_max = msg.max_range;
    range_fl_min = msg.min_range;
}

/* read the IMU message and generate a pose message like the physical ROSbot */
/* RHW Updated 24-11-20 with more accurate calculation of RPY */
void callback_fr(const sensor_msgs::Range &msg)
{
    //std::cout << "------Front Right------\n" << msg << std::endl;

    range_fr = msg.range;
    range_fr_max = msg.max_range;
    range_fr_min = msg.min_range;
}

/* read the rpy roll, pitch, yaw values [deg] */
void callback_rpy(const geometry_msgs::Vector3 &msg)
{
    //std::cout << "------IMU Degrees-----\n" << msg << std::endl;

    // update pose values
    pose = msg;

    if (state == "start"){
        // std::cout << "[INFO]: node_braitenberg.cpp, callback_rpy recieved: " << state << std::endl;

        // store starting pose if first instance only
        if (start_trigger)
        {
            // print info to console if running as main
            std::cout << "[INFO]: Starting Heading[deg]: " << std::to_string(pose.z) << std::endl;
            
            yaw_start = pose.z;
            start_trigger = false;
        }

        // proxy sensors will read 0.9 when nothing is ahead, make sure we aren't avoiding nothing by
        // checking that current range is less than max val on either sensor... 
        if (range_fl < (trip_thresh * range_fl_max) && range_fr < (trip_thresh * range_fr_max) ) // both sensors firing!
        {
            // if both sensors detect an obstacle, we should check which direction is most 
            // favourable with bias calculation... (to stay pointing straight)

            // rosbot should turn left
            if (yaw_start - pose.z < 0)
            {
                dTheta_yaw.data = -bberg_weight * ( 1 - (range_fr-range_fr_min) / (range_fr_max-range_fr_min) );
            }
            // else rosbot should turn right
            else
            {
                dTheta_yaw.data = bberg_weight * ( 1 - (range_fl-range_fl_min) / (range_fl_max-range_fl_min) );
            }
        }
        // if left sensor triggered but right hasn't, turn right
        else if (range_fl < (trip_thresh * range_fl_max) && range_fr >= (trip_thresh * range_fr_max) )
        {
            dTheta_yaw.data = bberg_weight * ( 1 - (range_fl-range_fl_min) / (range_fl_max-range_fl_min) );
        }
        // if right sensor triggered but left hasn't, turn left
        else if (range_fr < (trip_thresh * range_fr_max) && range_fl >= (trip_thresh * range_fl_max) )
        {
            dTheta_yaw.data = -bberg_weight * ( 1 - (range_fr-range_fr_min) / (range_fr_max-range_fr_min) );
        }
        // otherwise go back to starting direction
        else
        {
            dTheta_yaw.data = pose.z - yaw_start;
        }

        // print info to console if running as main
        std::cout << "----------> delta theta [deg]: " << std::to_string(dTheta_yaw.data) << std::endl;

        // publish proposed yaw angle (in radians)
        dTheta_yaw.data = -dTheta_yaw.data * deg2rad; // note negative because ROSbot cmds are inversed!
        explore_pub.publish(dTheta_yaw);
    }
    else if (state == "stop")
    {
        //std::cout << "[INFO]: Stopping ROSbot... " << std::endl;
    }
    else if (state == "reset")
    {
        //std::cout << "[INFO]: Resetting ROSbot... " << std::endl;

        start_trigger = true; // reset trigger so next 'start' will store a new starting pose
    }
    else {
        //std::cout << "[INFO]: node_braitenberg.cpp, callback_rpy recieved unexpected arguements: " << state << std::endl;
    }
}

/* setup callback as start, stop, reset, _, ... */
void callback_setup(const std_msgs::String &msg)
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
    ros::Subscriber range_fl = n.subscribe("/range/fl", 1, callback_fl);
    ros::Subscriber range_fr = n.subscribe("/range/fr", 1, callback_fr);
    ros::Subscriber pose_rpy = n.subscribe("/rpy", 1, callback_rpy);
    ros::Subscriber setup = n.subscribe("/cmd_setup", 1, callback_setup);

    // publisher node explore
    explore_pub = n.advertise<std_msgs::Float64>("/explore", 10);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}