#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include<cmath>
#include<iostream>
#include <vector>

ros::Publisher lidar_array;

std_msgs::Float64MultiArray lidar_FMA;
std::string state;

/* Calculate next speed step size */
float var_speed(float curr_dist, float max_dist, float max_speed, float min_speed)
{
    // if current measurement is > than sensor max range, set curr to max
    if (curr_dist > max_dist){curr_dist = max_dist;}

    // control loop to find speed based on measured distance to object
    float speed = max_speed + - ((max_speed - min_speed) * (abs(curr_dist - max_dist) / max_dist));

    return speed;
}

/* callback for scan data */
void callback_scan(const sensor_msgs::LaserScan &msg){

    // only publish results when not stop setup cmd.
    if (state != "stop"){

        // get number of samples in lidar sample vector
        int sample_num = msg.ranges.size();

        // calculate scan coverage area in radians
        float angle_range = msg.angle_increment * sample_num;

        // how many segments should the lidar be broken into
        int parts = 10; // EVEN NUMBERS ONLY

        // how many samples per part (last segment may have less than the rest)
        int remainder = sample_num % parts;
        int samples_per_part = (sample_num - remainder) / parts;
        
        // minimumm acceptable range of obstacles [m]
        float min_obj_range = .9; //.8

        // make array of lidar headings 
        float lidar_headings[parts] = {};
        int lidar_movables[parts] = {};
        int step = -1;
        float el;
        float speed = 0.5; // set random speed to start

        int count = 0;

        // loop through all lidar samples in the recieved message vector data
        for (int i = 0; i < sample_num; i++)
        {
            // calculate the next speed value for ROSbot based on distance to next
            // object ahead...
            if (i == 0)
            {
                speed = var_speed(msg.ranges[i], msg.range_max, .8, 0.2);
            }

            // check if current i is the first or middle of a part, add heading to 
            // array if i is at the middle of the part.
            if ( i % samples_per_part == 0) { step++; }
            else if  (i % (samples_per_part / 2) == 0)
            {
                lidar_headings[step] = -(i - samples_per_part/2) * msg.angle_increment;
            }
            
            // check if result is negative (cannot search for array entries with neg numbers i c++!)
            if (i - samples_per_part / 2 < 0)
            {
                el = msg.ranges[sample_num - samples_per_part/2 + i];
            }
            else
            {
                el = msg.ranges[i - samples_per_part/2];
            }

            // if the element range is < user defined threshold, set to not available (==1)
            if (el < min_obj_range * (1 - abs( 0.85 * sin( (-(i - samples_per_part/2) * msg.angle_increment) / 2) )) )
            {
                lidar_movables[step] = 1;
            }
        }

        // clear message cache
        lidar_FMA.data.clear();
        
        for (int i = 0; i <= parts-1; i++){
            if (lidar_movables[i] != 1)
            {
                // correct for shortest path to achieve desired heading
                if (lidar_headings[i] < -M_PI)
                {
                    lidar_headings[i] += 2*M_PI;
                }
                else if (lidar_headings[i] > M_PI)
                {
                    lidar_headings[i] -= 2*M_PI;
                }

            } else {lidar_headings[i] = 999.;}

            // add el to ROS message vector
            lidar_FMA.data.push_back(-lidar_headings[i]); // -ve to flip coords for ROSbot config
        }

        // add suggested speed
        lidar_FMA.data.push_back(speed);

        // publish message
        lidar_array.publish(lidar_FMA);
    }
}

/* setup callback as start, stop, reset, _, ... */
void callback_setup(const std_msgs::String &msg)
{
    // update pose values
    state = msg.data;
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
    lidar_array = n.advertise<std_msgs::Float64MultiArray>("/heading/lidar", 10);

    ros::Rate loop_rate(20); // this is the RATE, not time
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}