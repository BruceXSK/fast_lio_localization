//
// Created by bruce on 2022/3/29.
//

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include "pclomp/ndt_omp.h"

using namespace std;

typedef pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> NDT;
typedef pcl::PointCloud<pcl::PointXYZI> Cloud;

class Config
{
public:
    string odomFrame = "camera_init";
    struct
    {
        int numThreads = 4;
        int maximumIterations = 20;
        float voxelLeafSize = 2;
        float resolution = 1.0;
        double transformationEpsilon = 0.01;
        double stepSize = 0.1;
        double threshShift = 2;
        double threshRot = M_PI / 12;
    } ndt;

    explicit Config(ros::NodeHandle &nh) : _nh(nh)
    {
        _nh.getParam("odom_frame", odomFrame);

        _nh.getParam("ndt/num_threads", ndt.numThreads);
        _nh.getParam("ndt/maximum_iterations", ndt.maximumIterations);
        _nh.getParam("ndt/voxel_leaf_size", ndt.voxelLeafSize);
        _nh.getParam("ndt/transformation_epsilon", ndt.transformationEpsilon);
        _nh.getParam("ndt/step_size", ndt.stepSize);
        _nh.getParam("ndt/resolution", ndt.resolution);
        _nh.getParam("ndt/thresh_shift", ndt.threshShift);
        _nh.getParam("ndt/thresh_rot", ndt.threshRot);
    }

private:
    ros::NodeHandle &_nh;
};

class Localizer
{
public:
    explicit Localizer(ros::NodeHandle &nh) :
            _nh(nh), _cfg(nh), _mapPtr(new Cloud), _mapFilteredPtr(new Cloud)
    {
        _mapSub = _nh.subscribe("/map_cloud", 10, &Localizer::mapCallback, this);
        _initPoseSub = _nh.subscribe("/initialpose", 10, &Localizer::initPoseCallback, this);

        _pcSubPtr = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/velodyne_points", 1);
        _odomSubPtr = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/odom_lio", 1);
        _syncPtr = new message_filters::Synchronizer<ExactSyncPolicy>(
                ExactSyncPolicy(10), *_pcSubPtr, *_odomSubPtr
        );
        _syncPtr->registerCallback(boost::bind(&Localizer::syncCallback, this, _1, _2));

        _voxelGridFilter.setLeafSize(_cfg.ndt.voxelLeafSize, _cfg.ndt.voxelLeafSize, _cfg.ndt.voxelLeafSize);
        _ndt.setNumThreads(_cfg.ndt.numThreads);
        _ndt.setTransformationEpsilon(_cfg.ndt.transformationEpsilon);
        _ndt.setStepSize(_cfg.ndt.stepSize);
        _ndt.setResolution(_cfg.ndt.resolution);
        _ndt.setMaximumIterations(_cfg.ndt.maximumIterations);
    }

private:
    ros::NodeHandle &_nh;
    ros::Subscriber _mapSub;
    ros::Subscriber _initPoseSub;
    tf2_ros::TransformBroadcaster _br;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *_pcSubPtr;
    message_filters::Subscriber<nav_msgs::Odometry> *_odomSubPtr;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> ExactSyncPolicy;
    message_filters::Synchronizer<ExactSyncPolicy> *_syncPtr;

    NDT _ndt;
    pcl::VoxelGrid<pcl::PointXYZI> _voxelGridFilter;
    Config _cfg;
    Cloud::Ptr _mapPtr, _mapFilteredPtr;
    tf::Pose _odomPose;

    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        ROS_INFO("Get map");
        pcl::fromROSMsg<pcl::PointXYZI>(*msg, *_mapPtr);
        _voxelGridFilter.setInputCloud(_mapPtr);
        _voxelGridFilter.filter(*_mapFilteredPtr);
        _ndt.setInputTarget(_mapFilteredPtr);
    }

    void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        auto &q = msg->pose.pose.orientation;
        auto &p = msg->pose.pose.position;
        tf::Pose base2map(tf::Quaternion(q.x, q.y, q.z, q.w), tf::Vector3(p.x, p.y, p.z));
        auto t = base2map.inverseTimes(_odomPose);

        geometry_msgs::TransformStamped tfMsg;
        tfMsg.header.seq = msg->header.seq;
        tfMsg.header.stamp = msg->header.stamp;
        tfMsg.header.frame_id = msg->header.frame_id;
        tfMsg.child_frame_id = _cfg.odomFrame;
        tfMsg.transform.translation.x = t.getOrigin().x();
        tfMsg.transform.translation.y = t.getOrigin().y();
        tfMsg.transform.translation.z = t.getOrigin().z();
        tfMsg.transform.rotation.x = t.getRotation().x();
        tfMsg.transform.rotation.y = t.getRotation().y();
        tfMsg.transform.rotation.z = t.getRotation().z();
        tfMsg.transform.rotation.w = t.getRotation().w();

        _br.sendTransform(tfMsg);
        ROS_INFO("Initial pose set");
    }

    void syncCallback(const sensor_msgs::PointCloud2::ConstPtr &pc, const nav_msgs::Odometry::ConstPtr &odom)
    {
        tf::poseMsgToTF(odom->pose.pose, _odomPose);
        static tf::Pose lastNDTPose = _odomPose;

        tf::Pose currPose;
        tf::poseMsgToTF(odom->pose.pose, currPose);
        auto T = lastNDTPose.inverseTimes(currPose);

        if (hypot(T.getOrigin().x(), T.getOrigin().y()) > _cfg.ndt.threshShift or
            tf::getYaw(T.getRotation()) > _cfg.ndt.threshRot)
        {
            ROS_INFO("In");
        }

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fast_lio_localization");
    ros::NodeHandle nh("~");

    Localizer localizer(nh);

    ros::spin();

    return 0;
}