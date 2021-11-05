#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>


float deg2rad = 3.14159265/180;
float rad2deg = 1/deg2rad;
ros::Publisher pose_pub;

/* read the rpy message and genertae a transform to represent the orientation */
void rpy_callback(const geometry_msgs::Vector3 &rpy)
{
    static tf::Transform transform;
    static tf::Quaternion q;
    static tf::TransformBroadcaster br;
    q = tf::createQuaternionFromRPY(-rpy.x * deg2rad, -rpy.y * deg2rad, -rpy.z * deg2rad);

    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "orientation"));
}

/* read the IMU message and generate a pose message like the physical ROSbot */
/* RHW Updated 24-11-20 with more accurate calculation of RPY */
void imu_callback(const sensor_msgs::Imu &imu)
{
    static geometry_msgs::Vector3 rpy;
    double roll, pitch, yaw;

    tf::Quaternion q( imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    rpy.x = roll * rad2deg;
    rpy.y = pitch * rad2deg;
    rpy.z = yaw * rad2deg;

    if (pose_pub)
    {
        pose_pub.publish(rpy);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_controller");
    ros::NodeHandle n("~");

    ros::Subscriber pose_imu = n.subscribe("/imu", 1, imu_callback);
    ros::Subscriber pose_rpy = n.subscribe("/rpy", 1, rpy_callback);

    pose_pub = n.advertise<geometry_msgs::Vector3>("/rpy", 10);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

