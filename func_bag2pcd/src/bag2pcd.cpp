#include "bag2pcd.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "bag2pcd");
    if (argc != 4)
    {
        std::cout << "\033[31mNumber of params is invalid!" << argc << "\033[0m" << std::endl;
        return -1;
    }
    std::string InputBag = argv[1];
    std::string OutputPath = argv[2];
    int flag = argv[3];
    // if (!fs::exists(InputBag)){
    //     std::cout << "\033[31m InputBag is invalid! \033[0m" << std::endl;
    //     return -1;
    // }
    // if (!fs::exists(OutputPath)){
    //     std::cout << "\033[31m OutputPath is invalid! \033[0m" << std::endl;
    //     return -1;
    // }
    std::cout << "InputBag is: \033[32m " << InputBag << " \033[0m " << std::endl;
    std::cout << "OutputPath is: \033[32m " << OutputPath << " \033[0m " << std::endl;
    ROS_INFO("Ready to parse pcds.");
    rosbag::Bag in_bag;
    in_bag.open(InputBag, rosbag::bagmode::Read);
    rosbag::View view(in_bag, rosbag::TypeQuery("sensor_msgs/PointCloud2"));
    if (flag == 1) // 444428
    {
        for (rosbag::MessageInstance const m : view)
        {
            sensor_msgs::PointCloud2::Ptr pt_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (pt_msg != nullptr)
            {
                pcl::PointCloud<PointXYZIRT_IN_1>::Ptr Cloud_in(new pcl::PointCloud<PointXYZIRT_IN_1>);
                pcl::fromROSMsg(*pt_msg, *Cloud_in);
                pcl::PointCloud<PointXYZIRT_OUT>::Ptr Cloud_out(new pcl::PointCloud<PointXYZIRT_OUT>);
                Cloud_out->height = Cloud_in->height;
                Cloud_out->width = Cloud_in->width;
                Cloud_out->resize(Cloud_out->height * Cloud_out->width);
                for (size_t i = 0; i < Cloud_in->size(); i++)
                {
                    Cloud_out->points[i].x = Cloud_in->points[i].x;
                    Cloud_out->points[i].y = Cloud_in->points[i].y;
                    Cloud_out->points[i].z = Cloud_in->points[i].z;
                    Cloud_out->points[i].intensity = (std::uint16_t)Cloud_in->points[i].intensity;
                    Cloud_out->points[i].ring = Cloud_in->points[i].ring;
                    Cloud_out->points[i].timestamp = Cloud_in->points[i].timestamp;
                }
                std::ostringstream os;
                os << pt_msg->header.stamp;
                std::string time_name = os.str();
                std::string pcd_filename = OutputPath + time_name + ".pcd";
                pcl::io::savePCDFileBinary(pcd_filename, *Cloud_out);
            }
        }
    }
    else if (flag == 2) // 444228
    {
        for (rosbag::MessageInstance const m : view)
        {
            sensor_msgs::PointCloud2::Ptr pt_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (pt_msg != nullptr)
            {
                pcl::PointCloud<PointXYZIRT_IN_2>::Ptr Cloud_in(new pcl::PointCloud<PointXYZIRT_IN_2>);
                pcl::fromROSMsg(*pt_msg, *Cloud_in);
                pcl::PointCloud<PointXYZIRT_OUT>::Ptr Cloud_out(new pcl::PointCloud<PointXYZIRT_OUT>);
                Cloud_out->height = Cloud_in->height;
                Cloud_out->width = Cloud_in->width;
                Cloud_out->resize(Cloud_out->height * Cloud_out->width);
                for (size_t i = 0; i < Cloud_in->size(); i++)
                {
                    Cloud_out->points[i].x = Cloud_in->points[i].x;
                    Cloud_out->points[i].y = Cloud_in->points[i].y;
                    Cloud_out->points[i].z = Cloud_in->points[i].z;
                    Cloud_out->points[i].intensity = (std::uint16_t)Cloud_in->points[i].intensity;
                    Cloud_out->points[i].ring = Cloud_in->points[i].ring;
                    Cloud_out->points[i].timestamp = Cloud_in->points[i].timestamp;
                }
                std::ostringstream os;
                os << pt_msg->header.stamp;
                std::string time_name = os.str();
                std::string pcd_filename = OutputPath + time_name + ".pcd";
                pcl::io::savePCDFileBinary(pcd_filename, *Cloud_out);
            }
        }
    }
    else if (flag == 3) // 444128
    {
        for (rosbag::MessageInstance const m : view)
        {
            sensor_msgs::PointCloud2::Ptr pt_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (pt_msg != nullptr)
            {
                pcl::PointCloud<PointXYZIRT_IN_3>::Ptr Cloud_in(new pcl::PointCloud<PointXYZIRT_IN_3>);
                pcl::fromROSMsg(*pt_msg, *Cloud_in);
                pcl::PointCloud<PointXYZIRT_OUT>::Ptr Cloud_out(new pcl::PointCloud<PointXYZIRT_OUT>);
                Cloud_out->height = Cloud_in->height;
                Cloud_out->width = Cloud_in->width;
                Cloud_out->resize(Cloud_out->height * Cloud_out->width);
                for (size_t i = 0; i < Cloud_in->size(); i++)
                {
                    Cloud_out->points[i].x = Cloud_in->points[i].x;
                    Cloud_out->points[i].y = Cloud_in->points[i].y;
                    Cloud_out->points[i].z = Cloud_in->points[i].z;
                    Cloud_out->points[i].intensity = (std::uint16_t)Cloud_in->points[i].intensity;
                    Cloud_out->points[i].ring = Cloud_in->points[i].ring;
                    Cloud_out->points[i].timestamp = Cloud_in->points[i].timestamp;
                }
                std::ostringstream os;
                os << pt_msg->header.stamp;
                std::string time_name = os.str();
                std::string pcd_filename = OutputPath + time_name + ".pcd";
                pcl::io::savePCDFileBinary(pcd_filename, *Cloud_out);
            }
        }
    }
    else
    {
        std::cout << "Flag num Error, check std" << std::endl;
    }
    in_bag.close();
    ROS_INFO("FUNC Parse pcds completed.");
    return 1;
}
