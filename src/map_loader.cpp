//
// Created by bruce on 2022/3/29.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZI> Cloud;

/**
 * @brief It is quite necessary to warp the message publishing in a function.
 * First of all we cannot keep the temp pcl cloud in the main function to prevent it from taking up extra memory.
 * Besides, the publisher `mapPub` will keep the message itself because its `latch` is enabled. So, we do not have to
 * keep the instance of PointCloud2 message.
 * By doing this, we can ensure that the map_loader node will only take up a single times the size of the point cloud.
 * We can certainly clear the two point cloud instances in the main function, but warping them within a function is
 * undoubtedly the simplest and safest way.
 */
int pubMap(const string &mapPath, ros::Publisher &pub)
{
    ROS_INFO("Loading map");
    Cloud cloud;
    auto ret = pcl::io::loadPCDFile<pcl::PointXYZI>(mapPath, cloud);
    if (ret != 0) return ret;
    ROS_INFO("Map loaded");

    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg<pcl::PointXYZI>(cloud, cloudMsg);
    cloudMsg.header.stamp = ros::Time::now();
    cloudMsg.header.frame_id = "map";
    pub.publish(cloudMsg);
    ROS_INFO("Map published");

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_loader");
    ros::NodeHandle nh("~");

    auto mapPub = nh.advertise<sensor_msgs::PointCloud2>("/map_cloud", 1, true);

    string mapPath;
    nh.param<string>("map_path", mapPath, "");

    auto ret = pubMap(mapPath, mapPub);
    if (ret != 0) return ret;

    ros::spin();

    return 0;
}