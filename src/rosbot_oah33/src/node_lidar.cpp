#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include<cmath>
#include<iostream>
#include <vector>

ros::Publisher lidar_headings;

std::string state;

float lidar_array[720];

/* callback for scan data */
void callback_scan(const sensor_msgs::LaserScan &msg){

    // get number of samples in lidar sample vector
    int sample_num = msg.ranges.size();

    // calculate scan coverage area in radians
    float angle_range = msg.angle_increment * sample_num;

    // how many segments should the lidar be broken into
    int parts = 4; // EVEN NUMBERS ONLY

    // how many samples per part (last segment may have less than the rest)
    int remainder = sample_num % parts;
    int samples_per_part = (sample_num - remainder) / parts;

    std::cout<< "Remainder: " << std::to_string(remainder) << std::endl;
    std::cout<< "SPP: " << std::to_string(samples_per_part) << std::endl;
    std::cout<< "Samples: " << std::to_string(sample_num) << std::endl;

    // minimumm acceptable range of obstacles [m]
    float min_obj_range = 1.;

    // make 2D array of lidar headings 
    float lidar_headings[parts] = {};

    int step = 0;
    bool moveable = true;
    for (int i = 0; i <= sample_num-1; i++)
    {
        // if we are at a new segment, then store movable state and reset movable to default state
        // (which is considered as true)
        if ( i % samples_per_part == 0)
        {
            moveable = true;
        }
        // if at middle of part, store its heading in lidar_headings array
        else if  (i % (samples_per_part/ 2) == 0)
        {
            if (lidar_headings[step] != -999.)
            {
                lidar_headings[step] = (i - samples_per_part/2 ) * msg.angle_increment;
            }

            std::cout << std::to_string(i) << ") Heading: " << std::to_string( lidar_headings[step] ) << std::endl;
            //std::cout << std::to_string(i) << ") Step: " << std::to_string(step ) << std::endl;
            
            step++;
        }

        // if there is a range less than the minimum acceptable in this part, set movable to false
        // i.e. robot cannot move here...
        if (msg.ranges[i - samples_per_part/2] <= min_obj_range)
        {
            moveable = false;
            lidar_headings[step] = -999.;
        }

        /*
        else if ( i % (samples_per_part-1) == 0)
        {
            if (moveable == false)
            {
                lidar_headings[step] == -999.;
            }

            std::cout << std::to_string(lidar_headings[step]) << ", ";
        }
        */
    }

    for (int i = 0; i <= parts-1; i++){
        std::cout << std::to_string(lidar_headings[i]) << ", ";
    }
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