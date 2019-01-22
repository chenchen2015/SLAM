#include <iostream>
#include <cmath>
using namespace std; 

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

int main( int argc, char** argv )
{
    // construct a rotation matrix that does
    // 90 degree rotation on Z axis
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1))
                            .toRotationMatrix();

    Sophus::SO3d SO3_R(R);    // construct from rotation matrix
    Eigen::Quaterniond q(R);  // construct from quaternion
    Sophus::SO3d SO3_q(q);
    // show the constructed SO3 objects
    cout << "SO(3) from matrix:\n" << SO3_R.params() << endl;
    cout << "SO(3) from quaternion:\n" << SO3_q.params() << endl;

    // use logarithmic map to get the Lie algebra representation
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    // hat converts to matrix representation of Lie algebra element]
    // https://github.com/strasdat/Sophus/blob/master/sophus/so3.hpp#L661
    cout << "so3 hat =\n" << Sophus::SO3d::hat(so3) << endl;
    // vee maps to the jvector representation of Lie algebra
    cout << "so3 hat vee = "
         << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

    // // 增量扰动模型的更新
    // Eigen::Vector3d update_so3(1e-4, 0, 0); //假设更新量为这么多
    // Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3)*SO3_R;
    // cout<<"SO3 updated = "<<SO3_updated<<endl;
    
    // /********************萌萌的分割线*****************************/
    // cout<<"************我是分割线*************"<<endl;
    // // 对SE(3)操作大同小异
    // Eigen::Vector3d t(1,0,0);           // 沿X轴平移1
    // Sophus::SE3 SE3_Rt(R, t);           // 从R,t构造SE(3)
    // Sophus::SE3 SE3_qt(q,t);            // 从q,t构造SE(3)
    // cout<<"SE3 from R,t= "<<endl<<SE3_Rt<<endl;
    // cout<<"SE3 from q,t= "<<endl<<SE3_qt<<endl;
    // // 李代数se(3) 是一个六维向量，方便起见先typedef一下
    // typedef Eigen::Matrix<double,6,1> Vector6d;
    // Vector6d se3 = SE3_Rt.log();
    // cout<<"se3 = "<<se3.transpose()<<endl;
    // // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
    // // 同样的，有hat和vee两个算符
    // cout<<"se3 hat = "<<endl<<Sophus::SE3::hat(se3)<<endl;
    // cout<<"se3 hat vee = "<<Sophus::SE3::vee( Sophus::SE3::hat(se3) ).transpose()<<endl;
    
    // // 最后，演示一下更新
    // Vector6d update_se3; //更新量
    // update_se3.setZero();
    // update_se3(0,0) = 1e-4d;
    // Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3)*SE3_Rt;
    // cout<<"SE3 updated = "<<endl<<SE3_updated.matrix()<<endl;
    
    return 0;
}