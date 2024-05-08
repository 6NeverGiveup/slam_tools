#include <iostream>
#include <string>
#include <vector>
#include "json.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
using json = nlohmann::json;
// 自定义点云类型PointXYZIRT接受pcd点云

struct PointXYZIRT
{
    PCL_ADD_POINT4D;                // point xyz
    PCL_ADD_INTENSITY;              // i
    std::uint16_t ring;             // xian id
    double timestamp;               // timestamp
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // enable to use the new() method
} EIGEN_ALIGN16;
// 注册自定义点云
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))

struct PoseResult
{
    double posetime;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    float q_x;
    float q_y;
    float q_z;
    float q_w;
};

struct PointTypePose
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};

using namespace std;
using namespace pcl;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("pc_viewer"));

pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, PointTypePose *transformIn)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
#pragma omp parallel for num_threads(15)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

int ReadPose(std::string file_path, std::vector<PoseResult> &Info)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "ReadPose 无法打开JSON文件" << std::endl;
        return -1;
    }
    std::cout << "pose data file is : " << file_path << std::endl;
    json jsonData;
    file >> jsonData;
    // pose data may less than ptr data
    int index = 0;
    for (json::iterator it = jsonData.begin(); it != jsonData.end(); ++it)
    {
        double pose_time = std::stod(it.key());
        // std::cout << "pose_time is "  << pose_time << std::endl;
        PoseResult tmp_pose;
        tmp_pose.posetime = pose_time;
        json tmp_data = jsonData[it.key()];
        tmp_pose.x = tmp_data["position"][0];
        tmp_pose.y = tmp_data["position"][1];
        tmp_pose.z = tmp_data["position"][2];
        tmp_pose.q_x = tmp_data["orientation"][0];
        tmp_pose.q_y = tmp_data["orientation"][1];
        tmp_pose.q_z = tmp_data["orientation"][2];
        tmp_pose.q_w = tmp_data["orientation"][3];
        Info.push_back(tmp_pose);
        index++;
    }
    std::cout << "pose read done !" << std::endl;
    std::cout << "----------------------------------------------------------------------" << std::endl;
    return 1;
}

Eigen::Vector3d Quaterniond2Euler(const float x, const float y, const float z, const float w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;

    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    return euler;
}

void visualizeCloud(pcl::PointCloud<PointXYZI>::Ptr &cloud)
{
    viewer->removePointCloud("realtime pcl");
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    pcl::PointCloud<PointXYZI>::Ptr cloud_for_view(new pcl::PointCloud<PointXYZI>);
    cloud_for_view->points.resize(cloud->points.size());
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        if (isnan(cloud->points[i].x))
        {
            continue;
        }
        cloud_for_view->points[i].x = cloud->points[i].x;
        cloud_for_view->points[i].y = cloud->points[i].y;
        cloud_for_view->points[i].z = cloud->points[i].z;
        cloud_for_view->points[i].intensity = static_cast<int>(cloud->points[i].intensity);
        // std::cout <<   "data: " << "x: " << cloud->points[i].x << " y: " << cloud->points[i].y << " z: " << cloud->points[i].z <<
        // " intensity: " << cloud->points[i].intensity <<  " ring: " << cloud->points[i].ring << " timestamp: "  <<  cloud->points[i].timestamp << std::endl;
    }
    // std::cout << "cloud_for_view size is " << cloud_for_view->size() << std::endl;
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(cloud_for_view, "intensity");
    viewer->addPointCloud(cloud_for_view, handler, "realtime pcl");
    viewer->updatePointCloud(cloud_for_view, handler, "realtime pcl");
    viewer->spinOnce();
}

void visual_map(pcl::PointCloud<PointXYZI>::Ptr &cloud)
{
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(cloud, "x");
    viewer->addPointCloud(cloud, handler, "map");
    viewer->updatePointCloud(cloud, handler, "map");
    viewer->spinOnce();
}

int main(int argc, char **argv)
{
    string PCD_PATH = "/home/jeff/slam/deskewed_cloud_pcd/";
    DIR *dir;
    struct dirent *ptr;
    std::vector<std::string> pcd_file_list;
    const char *p = PCD_PATH.c_str();
    dir = opendir(p);
    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
            continue;
        if (strcmp(ptr->d_name, "pcd") == 0)
            continue;
        pcd_file_list.push_back(ptr->d_name);
    }
    closedir(dir);
    std::sort(pcd_file_list.begin(), pcd_file_list.end());
    std::cout << "file_list size is : " << pcd_file_list.size() << std::endl;

    string MAP_PCD_PATH = "/home/jeff/slam/CornerMap.pcd";
    pcl::PointCloud<PointXYZI>::Ptr MAP_PCD(new pcl::PointCloud<PointXYZI>);
    if (pcl::io::loadPCDFile<PointXYZI>(MAP_PCD_PATH, *MAP_PCD) == -1)
    {
        PCL_ERROR("Couldn't read the pcd file!\n");
        return (-1);
    }
    visual_map(MAP_PCD);

    string pose_path = "/home/jeff/slam/pose_data.json";
    std::vector<PoseResult> Info;
    ReadPose(pose_path, Info);
    std::cout << " Info.size() is : " << Info.size() << std::endl;
    for (int num_index = 0; num_index < pcd_file_list.size(); num_index++)
    {
        string pcd_fileName = pcd_file_list[num_index];
        string pcd_filePath = PCD_PATH + pcd_fileName;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudin(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<PointXYZI>::Ptr pcd_pcl_pointcloud(new pcl::PointCloud<PointXYZI>);
        for (size_t i = 0; i < Info.size(); i++)
        {
            double cloud_time = std::stod(pcd_fileName);
            if (fabs(Info[i].posetime - cloud_time) < 0.05)
            {
                if (pcl::io::loadPCDFile<PointXYZI>(pcd_filePath, *pcd_pcl_pointcloud) == -1)
                {
                    PCL_ERROR("Couldn't read the pcd file!\n");
                    return (-1);
                }
                PointTypePose tmp_pose;
                Eigen::Vector3d euler = Quaterniond2Euler(Info[i].q_x, Info[i].q_y, Info[i].q_z, Info[i].q_w);
                tmp_pose.x = Info[i].x;
                tmp_pose.y = Info[i].y;
                tmp_pose.z = Info[i].z;
                tmp_pose.roll = euler[2];
                tmp_pose.pitch = euler[1];
                tmp_pose.yaw = euler[0];
                *cloudin = *transformPointCloud(pcd_pcl_pointcloud, &tmp_pose);
                // std::ofstream out("../time.txt", std::ios::app | std::ios::out);
                // if (!out)
                // {
                //     std::cerr << "time.txt Error opening file!" << std::endl;
                //     return 0;
                // }
                // out << "num_index is : " << num_index << " "
                //     << "timestamp is : " << setprecision(19) << pcd_fileName << std::endl;

                // out.close();
                visualizeCloud(cloudin);
            }
        }
    }
    return 1;
}
