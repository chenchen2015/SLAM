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
    cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;
    cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl;

    // use logarithmic map to get the Lie algebra representation
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    // hat converts to matrix representation of Lie algebra element
    // https://github.com/strasdat/Sophus/blob/master/sophus/so3.hpp#L661
    cout << "so3 hat =\n" << Sophus::SO3d::hat(so3) << endl;
    // vee maps to the jvector representation of Lie algebra
    // https://github.com/strasdat/Sophus/blob/master/sophus/so3.hpp#L742
    cout << "so3 hat vee = "
         << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

    // update the increment disturbance model
    // exp() https://github.com/strasdat/Sophus/blob/master/sophus/so3.hpp#L568
    Eigen::Vector3d update_so3(1e-4, 0, 0);
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;

    cout << "\n************ SE3 group tests *************\n" << endl;
    // SE3 shares similar API with SO3
    Eigen::Vector3d t(1, 0, 0);   // translate 1 unit along X
    Sophus::SE3d SE3_Rt(R, t);    // construct SE(3) from R, t
    Sophus::SE3d SE3_qt(q, t);    // construct SE(3) from q, t
    cout << "SE3 from R, t = " << endl << SE3_Rt.matrix() << endl;
    cout << "\nSE3 from q, t = " << endl << SE3_qt.matrix() << endl;
    // se(3) Lie algebra elements are represented in a Vector6d
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;
    // in Sophus, se(3) has the translation on the left and 
    // rotation on the right
    // and similarly, se(3) also has the hat and vee method
    cout << "se3 hat = " << endl << Sophus::SE3d::hat(se3) << endl;
    cout << "se3 hat vee = "
         << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

    // updating the se(3) object is similar to so(3)
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4d;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

    return 0;
}