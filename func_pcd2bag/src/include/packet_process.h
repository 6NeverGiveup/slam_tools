//sys
#include <iostream>
#include <string>
#include <vector>
#include <inttypes.h>
//ros
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h> // pcl::PointCloud ->pcl::PointCloud2 -> sensor_msgs::PointCloud2
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//defination
struct PointXYZIRT
    {
        PCL_ADD_POINT4D;                // point xyz
        std::uint16_t intensity;        // i
        std::uint16_t ring;                  // xian id
        double timestamp;               // timestamp
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // enable to use the new() method
    } EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint16_t, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))
using namespace std;
