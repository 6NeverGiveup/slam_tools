#include "eigen3/Eigen/Eigen"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include<iostream> 

//? func 1
static void Q_to_EulerAngle(const double x,const double y,const double z,const double w, double& roll, double& pitch, double& yaw)
{
/*
输入：x,y,z,w　为四元数
输出：roll，pitch，yaw欧拉角
*/
// roll (x-axis rotation)
    double sinr_cosp = +2.0 * (w * x + y * z);
    double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

// yaw (z-axis rotation)
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
//    return yaw;
}

//? func 2 (same to 1)
static Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;

    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    return euler;
}

static Eigen::Quaterniond Euler_to_Quaternion(const double yaw, const double pitch, const double roll) // yaw (Z), pitch (Y), roll (X)
{
    //? Degree to radius:
    // double tmp_yaw = yaw * M_PI / 180;
    // double tmp_pitch = pitch * M_PI / 180;
    // double tmp_roll = roll * M_PI / 180;

    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Quaterniond q;
    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;
    return q;
}

static Eigen::Matrix3d Q_to_matrix(const double x,const double y,const double z,const double w) // yaw (Z), pitch (Y), roll (X)
{
    //?四元数到旋转矩阵
    //下面的变量名称自拟      
    Eigen::Quaterniond q_odom_curr_tmp;//声明一个Eigen类的四元数
    //此处进行赋值，使用其他语句以及合理的常数也可
    q_odom_curr_tmp.x() = x;
    q_odom_curr_tmp.y() = y;
    q_odom_curr_tmp.z() = z;
    q_odom_curr_tmp.w() = w;
    Eigen::Matrix3d R_odom_curr_tmp;//声明一个Eigen类的3*3的旋转矩阵
    //四元数转为旋转矩阵--先归一化再转为旋转矩阵
    R_odom_curr_tmp= q_odom_curr_tmp.normalized().toRotationMatrix();  
    // std::cout<<"q_ori:"<< std::endl; 
    // std::cout<< R_odom_curr_tmp <<std::endl;
    // std::cout<<"q_inverse:"<< std::endl; 
    // std::cout<< R_odom_curr_tmp.inverse() <<std::endl;
    return R_odom_curr_tmp;
}

static Eigen::Quaterniond matrix_to_Q(const Eigen::Matrix3d &R_odom_curr_now)
{
    Eigen::Matrix3d tmp = R_odom_curr_now;
    Eigen::Quaterniond q_odom_curr_now = Eigen::Quaterniond(tmp);//旋转矩阵转为四元数
    q_odom_curr_now.normalize();//转为四元数之后，需要进行归一化
    // std::cout << "q_odom_curr_now is : " << q_odom_curr_now.w() << " " << q_odom_curr_now.x() << " " << q_odom_curr_now.y() << " " << q_odom_curr_now.z() << std::endl;
    return q_odom_curr_now;
}

static Eigen::Vector3d matrix_to_Euler(const Eigen::Matrix3d &R_odom_curr_now)
{
    Eigen::Vector3d euler = R_odom_curr_now.eulerAngles(2, 1, 0);
    return euler;
}

static Eigen::Matrix3d Euler_to_matrix(const double& roll, const double& pitch, const double& yaw) // yaw (Z), pitch (Y), roll (X)
{
    //?欧拉角到旋转矩阵
    Eigen::Vector3d rpy_raw;
    rpy_raw << roll, pitch, yaw;
    // std::cout<<"rpy_raw:"<< std::endl;
    // std::cout<< rpy_raw <<std::endl;
    // rpy_raw = rpy_raw * M_PI / 180;
    Eigen::Matrix3d R_AB;

    R_AB = Eigen::AngleAxisd(rpy_raw[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy_raw[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy_raw[0], Eigen::Vector3d::UnitX());
    // std::cout<<"R_AB :"<< std::endl; 
    // std::cout<< R_AB<<std::endl;
    return R_AB;
}

