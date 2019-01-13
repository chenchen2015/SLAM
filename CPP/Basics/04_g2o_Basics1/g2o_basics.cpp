#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
using namespace std;
using ClockT = chrono::high_resolution_clock;
using DurationMS = chrono::duration<double, std::milli>;
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
inline double func(const Eigen::Vector3d &abc, double x) {
    return std::exp(abc(0, 0) * x * x + abc(1, 0) * x + abc(2, 0));
}
inline double func(double a, double b, double c, double x) {
    return std::exp(a * x * x + b * x + c);
}

// curve fitting vertex interface
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
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
class CurveFittingEdge
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

// solver type
enum SolverTypes { Levenberg, GaussNewton, Dogleg };

int main(int argc, char **argv) {
    // Basic g2o example
    // solving curve fitting problem with graph
    // using nonlinear optimization methods
    double a = 1.0, b = 2.0, c = 1.0; // parameters
    int N = 200;                      // number of points
    int nIter = 100;                  // number of iterations of the solver
    double wSigma = 0.25;             // sigma of noise
    cv::RNG rng;                      // random number generator
    double abc[3] = {0, 0, 0};        // estimated parameters
    vector<double> xData, yData;      // measured data
    const SolverTypes solverType = SolverTypes::Levenberg;
    // generate observations
    cout << "Generating observations... " << endl;
    for (int i = 0; i < N; ++i) {
        double x = i / double(N);
        xData.push_back(x);
        yData.push_back(func(a, b, c, x) + rng.gaussian(wSigma));
    }
    // configure g2o
    using Block = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
    auto pLinearSolver =
        std::make_unique<g2o::LinearSolverDense<Block::PoseMatrixType>>();
    auto pSolver = std::make_unique<Block>(std::move(pLinearSolver));
    g2o::OptimizationAlgorithmLevenberg *pSolverAlgo =
        new g2o::OptimizationAlgorithmLevenberg(std::move(pSolver));
    // std::unique_ptr<g2o::OptimizationAlgorithm> pSolverAlgo;
    // switch (solverType) {
    // case SolverTypes::Levenberg:
    //     auto pSolverAlgo =
    //     std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
    //         std::move(pSolver));
    //     break;
    // case SolverTypes::GaussNewton:
    //     auto pSolverAlgo =
    //     std::make_unique<g2o::OptimizationAlgorithmGaussNewton>(
    //         std::move(pSolver));
    //     break;
    // case SolverTypes::Dogleg:
    //     auto pSolverAlgo =
    //     std::make_unique<g2o::OptimizationAlgorithmDogleg>(
    //         std::move(pSolver));
    //     break;
    // }
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(pSolverAlgo);
    optimizer.setVerbose(true);
    // configure graph
    auto pVert = std::make_unique<CurveFittingVertex>();
    pVert->setEstimate(Eigen::Vector3d(0, 0, 0)); // initial guess
    pVert->setId(0);
    optimizer.addVertex(pVert.get());
    // add edges
    for (int i = 0; i < N; ++i) {
        auto pEdge = std::make_unique<CurveFittingEdge>(xData[i]);
        pEdge->setId(i);
        pEdge->setVertex(0, pVert.get());
        pEdge->setMeasurement(yData[i]);
        pEdge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1.0f /
                              (wSigma * wSigma));
        optimizer.addEdge(pEdge.release());
    }
    // start optimiztion
    auto t0 = ClockT::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    DurationMS timeUsed = ClockT::now() - t0;
    cout << "Solver spent " << timeUsed.count() << " ms." << endl;
    // output result
    cout << "Estimated parameters: " << pVert->estimate().transpose() << endl;

    return EXIT_SUCCESS;
}
