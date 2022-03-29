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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_loader");
    ros::NodeHandle nh("~");

    auto mapPub = nh.advertise<sensor_msgs::PointCloud2>("/map_cloud", 1, true);

    string path;
    nh.param<string>("map_path", path, "");

    ROS_INFO("Loading map");
    Cloud cloud;
    auto ret = pcl::io::loadPCDFile<pcl::PointXYZI>(path, cloud);
    if (ret != 0) return ret;

    ROS_INFO("Map loaded");

    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg<pcl::PointXYZI>(cloud, cloudMsg);
    cloudMsg.header.stamp = ros::Time::now();
    cloudMsg.header.frame_id = "map";

    mapPub.publish(cloudMsg);
    ROS_INFO("Map published");

    ros::spin();

    return 0;
}