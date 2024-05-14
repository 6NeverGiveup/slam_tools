//sys
#include <iostream>
#include <string>
#include <vector>
#include <inttypes.h>
//#include <filesystem>
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
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include<experimental/filesystem>

//using fs=std::experimental::filesystem;

//defination
struct PointXYZIRT_OUT
    {
        PCL_ADD_POINT4D;                
        std::uint16_t intensity;        
        std::uint16_t ring;                 
        double timestamp;               
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    } EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT_OUT,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint16_t, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))
//444428 
struct PointXYZIRT_IN_1
    {
        PCL_ADD_POINT4D;                
        float intensity;        
        std::uint16_t ring;                 
        double timestamp;               
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    } EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT_IN_1,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))
//444228
struct PointXYZIRT_IN_2
    {
        PCL_ADD_POINT4D;                
        std::uint16_t intensity;        
        std::uint16_t ring;                 
        double timestamp;               
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    } EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT_IN_2,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint16_t, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))

//444128
struct PointXYZIRT_IN_3
    {
        PCL_ADD_POINT4D;                
        std::uint8_t intensity;        
        std::uint16_t ring;                 
        double timestamp;               
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    } EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT_IN_3,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))






using namespace std;
//using fs = experimental::filesystem;
