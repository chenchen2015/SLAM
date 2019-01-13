#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
using namespace std;
// Eigen
#include <eigen3/Eigen/Core>
// g2o
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
// OpenCV
#include <opencv2/core/core.hpp>

// curve function
inline double func(const Eigen::Vector3d& abc, double x){
    return std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
}
inline double func(double a, double b, double c, double x) {
    return std::exp(a * _x * _x + b * _x + c);
}

// curve fitting vertex interface
class ICurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
  public:
    // hack to align memory for efficiency
    // http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() { _estimate << 0, 0, 0; }

    virtual void oplusImpl(const double *update) {
        _estimate += Eigen::Vector3d(update);
    }
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}
};

// curve fitting edge interface
class ICurveFittingEdge
    : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // constructor
    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}
    void computeError() {
        const CurveFittingVertex *v =
            static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - func(abc, _x);
    }
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}

  public:
    double _x;
};


int main(int argc, char** argv){
    // Basic g2o example
    // solving curve fitting problem with graph
    // using nonlinear optimization methods
    double a = 1.0, b = 2.0, c = 1.0; // parameters
    int N = 200;                      // number of points
    double w_sigma = 0.25;            // sigma of noise
    cv::RNG rng;                      // random number generator
    double abc[3] = {0, 0, 0};        // estimated parameters
    vector<double> xData, yData;      // measured data
    // generate observations
    cout << "Generating observations... " << endl;
    for (int i = 0; i < N; ++i) {
        double x = i / double(N);
        xData.push_back(x);
        yData.push_back(fun(x));
    }
}
