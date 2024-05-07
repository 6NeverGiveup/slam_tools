#include "rotation_transfrom.h"

using namespace std;
#define if_quater_2_matrix 1
#define if_matrix_2_quater 1
#define if_euler_2_matrix 1
#define if_matrix_2_euler 1
#define if_quater_2_euler 1
#define if_euler_2_quater 1

int main(int argc, char const *argv[])
{ 
    // 以下赋值随便给的，可以自定义修改
    Eigen::Matrix3d test_matrix_in;
    test_matrix_in << -0.999804, -0.0147122, -0.0132582,
            0.0149837,  -0.999675, -0.0206149,
            -0.0129506, -0.0208096,     0.9997;
    Eigen::Quaterniond test_Quater_in;
    test_Quater_in.x() = -0.006552894582320790626;
    test_Quater_in.y() = -0.01035718801478957216;
    test_Quater_in.z() = 0.9998973256027507972;
    test_Quater_in.w() = 0.007424720970217765607;
    Eigen::Vector3d test_euler_in;
    test_euler_in[0] = -0.0208128;
    test_euler_in[1] = 0.012951;
    test_euler_in[2] = -3.12661;

    Eigen::Matrix3d test_matrix_out;
    Eigen::Quaterniond test_Quater_out;
    Eigen::Vector3d test_euler_out;

#if if_quater_2_matrix
    test_matrix_out = Q_to_matrix(test_Quater_in.x(), test_Quater_in.y(), test_Quater_in.z(), test_Quater_in.w());
    std::cout << "quater_2_matrix func test_matrix_out is: " << endl <<  test_matrix_out << std::endl;
    std::cout << "----------------------------------------------------------" << std::endl;
#endif

#if if_matrix_2_quater
    test_Quater_out = matrix_to_Q(test_matrix_in);
    std::cout << "matrix_2_quater func test_Quater_out is: " 
    << endl << " x : " << test_Quater_out.x()
    << endl << " y : " << test_Quater_out.y()
    << endl << " z : " << test_Quater_out.z() 
    << endl << " w : " << test_Quater_out.w() 
    << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
#endif

#if if_euler_2_matrix
    test_matrix_out = Euler_to_matrix(test_euler_in[0], test_euler_in[1], test_euler_in[2]);
    std::cout << "Euler_to_matrix func test_matrix_out is: " << endl << test_matrix_out << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
#endif

#if if_matrix_2_euler
    test_euler_out = matrix_to_Euler(test_matrix_in);
    cout << "matrix_2_euler result is:" << endl;
    cout << "r = " << test_euler_out[2] << endl;
    cout << "p = " << test_euler_out[1] << endl;
    cout << "y = " << test_euler_out[0] << endl;
    std::cout << "---------------------------------------------------------" << std::endl;
#endif

#if if_quater_2_euler
    double r, p, y;
    Q_to_EulerAngle(test_Quater_in.x(), test_Quater_in.y(), test_Quater_in.z(), test_Quater_in.w(), r, p, y);
    std::cout << "first Q_to_EulerAngle rpy result is : " << r << " " << p << " " << y << std::endl;
    test_euler_out = Quaterniond2Euler(test_Quater_in.x(), test_Quater_in.y(), test_Quater_in.z(), test_Quater_in.w());
    cout << "second rpy Quaterniond2Euler result is:" << endl;
    cout << "r = " << test_euler_out[2] << endl;
    cout << "p = " << test_euler_out[1] << endl;
    cout << "y = " << test_euler_out[0] << endl;
    std::cout << "---------------------------------------------------------" << std::endl;     
#endif

#if if_euler_2_quater
    test_Quater_out = Euler_to_Quaternion(test_euler_in[2], test_euler_in[1], test_euler_in[0]);
    std::cout << "euler_2_quater func test_Quater_out is: " 
    << endl << " x : " << test_Quater_out.x()
    << endl << " y : " << test_Quater_out.y()
    << endl << " z : " << test_Quater_out.z() 
    << endl << " w : " << test_Quater_out.w() 
    << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
#endif
    return 1;
}
