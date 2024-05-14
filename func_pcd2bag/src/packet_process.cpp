#include "packet_process.h"

ros::Time timesamp2rostime(string timesamp){
  std::string sec_string = timesamp.substr(0,10);
  std::string nsec_string = timesamp.substr(11,9);
  while(nsec_string.length() < 9){
      nsec_string += "0";
  }
  return ros::Time(std::stoi(sec_string),std::stoi(nsec_string));
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "packet_process");
    string PCD_PATH = argv[1];
    string INS_BAG = argv[2];
    string OUTPUT_BAG_PATH = argv[3];
    if (PCD_PATH.empty()){
        std::cout << "\033[31m PCD PATH is invalid! \033[0m" << std::endl;
        return -1;
    }
    if (INS_BAG.empty()){
        std::cout << "\033[31m INS_BAG is invalid! \033[0m" << std::endl;
        return -1;
    }
    if (OUTPUT_BAG_PATH.empty()){
        std::cout << "\033[31m OUTPUT_BAG_PATH is invalid! \033[0m" << std::endl;
        return -1;
    }
    std::cout << "PCD_PATH is: " << PCD_PATH << std::endl;
    std::cout << "INS_BAG is: " << INS_BAG << std::endl;
    std::cout << "OUTPUT_BAG_PATH is: " << OUTPUT_BAG_PATH << std::endl;
    std::cout << "Ready to combine data" << std::endl;
    rosbag::Bag output_bag;
    output_bag.open(OUTPUT_BAG_PATH + "merged.bag", rosbag::bagmode::Write);

    DIR *dir;
    struct dirent *ptr;
    std::vector<std::string> pcd_file_list;
    const char *p = PCD_PATH.c_str();
    dir = opendir(p);
    while ((ptr = readdir(dir)) != NULL){
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
            continue;
        if (strcmp(ptr->d_name, "pcd") == 0)
            continue;
        pcd_file_list.push_back(ptr->d_name);
    }
    closedir(dir);
    std::sort(pcd_file_list.begin(), pcd_file_list.end());

    for (int num_index = 0; num_index < pcd_file_list.size(); num_index++){
        string pcd_fileName = pcd_file_list[num_index];
        string pcd_filePath = PCD_PATH + pcd_fileName;
        pcl::PointCloud<PointXYZIRT>::Ptr pcd_pcl_pointcloud(new pcl::PointCloud<PointXYZIRT>);
        if (pcl::io::loadPCDFile<PointXYZIRT>(pcd_filePath, *pcd_pcl_pointcloud) == -1){
            PCL_ERROR("Couldn't read the pcd file!\n");
            return -1;
        }
        sensor_msgs::PointCloud2 sm_pc2;
        pcl::toROSMsg(*pcd_pcl_pointcloud, sm_pc2);
        sm_pc2.header.stamp = timesamp2rostime(pcd_file_list[num_index]);
        sm_pc2.header.frame_id = "lidar";
        output_bag.write("/points_raw", sm_pc2.header.stamp, sm_pc2);
    }
    rosbag::Bag gps_imu_bag;
    gps_imu_bag.open(INS_BAG, rosbag::bagmode::Read);
    sensor_msgs::Imu::Ptr imu_msg;
    sensor_msgs::NavSatFix::Ptr gps_msg;
    for (rosbag::MessageInstance const m : rosbag::View(gps_imu_bag)){
        imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg != nullptr)
            output_bag.write("/imu/data", imu_msg->header.stamp, imu_msg);
        gps_msg = m.instantiate<sensor_msgs::NavSatFix>();
        if (gps_msg != nullptr)
            output_bag.write("/gps/data", gps_msg->header.stamp, gps_msg);
    }
    gps_imu_bag.close();
    output_bag.close();
    ROS_INFO("Data merge completed.");
    return 1;
}

