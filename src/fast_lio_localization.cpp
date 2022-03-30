//
// Created by bruce on 2022/3/29.
//

#include <chrono>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
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
        bool debug = false;
        int numThreads = 4;
        int maximumIterations = 20;
        float voxelLeafSize = 0.1;
        float resolution = 1.0;
        double transformationEpsilon = 0.01;
        double stepSize = 0.1;
        double threshShift = 2;
        double threshRot = M_PI / 12;
        double minScanRange = 1.0;
        double maxScanRange = 100;
    } ndt;

    explicit Config(ros::NodeHandle &nh) : _nh(nh)
    {
        _nh.getParam("odom_frame", odomFrame);

        _nh.getParam("ndt/debug", ndt.debug);
        _nh.getParam("ndt/num_threads", ndt.numThreads);
        _nh.getParam("ndt/maximum_iterations", ndt.maximumIterations);
        _nh.getParam("ndt/voxel_leaf_size", ndt.voxelLeafSize);
        _nh.getParam("ndt/transformation_epsilon", ndt.transformationEpsilon);
        _nh.getParam("ndt/step_size", ndt.stepSize);
        _nh.getParam("ndt/resolution", ndt.resolution);
        _nh.getParam("ndt/thresh_shift", ndt.threshShift);
        _nh.getParam("ndt/thresh_rot", ndt.threshRot);
        _nh.getParam("ndt/min_scan_range", ndt.minScanRange);
        _nh.getParam("ndt/max_scan_range", ndt.maxScanRange);
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

        _odomMap.setIdentity();
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
    tf::Pose _baseOdom, _odomMap;

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
        tf::Pose baseMap(tf::Quaternion(q.x, q.y, q.z, q.w), tf::Vector3(p.x, p.y, p.z));
        _odomMap = baseMap * _baseOdom.inverse();
        ROS_INFO("Initial pose set");
    }

    void syncCallback(const sensor_msgs::PointCloud2::ConstPtr &pcMsg, const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        static chrono::steady_clock::time_point t0, t1;
        tf::poseMsgToTF(odomMsg->pose.pose, _baseOdom);
        static tf::Pose lastNDTPose = _baseOdom;

        auto T = lastNDTPose.inverseTimes(_baseOdom);

        if (hypot(T.getOrigin().x(), T.getOrigin().y()) > _cfg.ndt.threshShift or
            tf::getYaw(T.getRotation()) > _cfg.ndt.threshRot)
        {
            Cloud::Ptr tmpCloudPtr(new Cloud);
            pcl::fromROSMsg(*pcMsg, *tmpCloudPtr);

            Cloud::Ptr scanCloudPtr(new Cloud);
            for (const auto &p: *tmpCloudPtr)
            {
                auto r = hypot(p.x, p.y);
                if (r > _cfg.ndt.minScanRange and r < _cfg.ndt.maxScanRange)
                    scanCloudPtr->push_back(p);
            }

            _ndt.setInputSource(scanCloudPtr);
            auto baseMap = _odomMap * _baseOdom;
            Eigen::Affine3d baseMapMat;
            tf::poseTFToEigen(baseMap, baseMapMat);

            Cloud::Ptr outputCloudPtr(new Cloud);
            if (_cfg.ndt.debug) t0 = chrono::steady_clock::now();
            _ndt.align(*outputCloudPtr, baseMapMat.matrix().cast<float>());
            if (_cfg.ndt.debug) t1 = chrono::steady_clock::now();

            auto tNDT = _ndt.getFinalTransformation();
            tf::Transform baseMapNDT;
            tf::poseEigenToTF(Eigen::Affine3d(tNDT.cast<double>()), baseMapNDT);
            _odomMap = baseMapNDT * _baseOdom.inverse();
            lastNDTPose = _baseOdom;

            if (_cfg.ndt.debug) ROS_INFO("NDT: %ldms", chrono::duration_cast<chrono::milliseconds>(t1 - t0).count());
            ROS_INFO("NDT Relocated");
        }
        publishTF();
    }

    void publishTF()
    {
        geometry_msgs::TransformStamped tfMsg;
        tfMsg.header.stamp = ros::Time::now();
        tfMsg.header.frame_id = "map";
        tfMsg.child_frame_id = _cfg.odomFrame;
        tfMsg.transform.translation.x = _odomMap.getOrigin().x();
        tfMsg.transform.translation.y = _odomMap.getOrigin().y();
        tfMsg.transform.translation.z = _odomMap.getOrigin().z();
        tfMsg.transform.rotation.x = _odomMap.getRotation().x();
        tfMsg.transform.rotation.y = _odomMap.getRotation().y();
        tfMsg.transform.rotation.z = _odomMap.getRotation().z();
        tfMsg.transform.rotation.w = _odomMap.getRotation().w();

        _br.sendTransform(tfMsg);
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