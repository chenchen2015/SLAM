#include <chrono>
#include <iostream>
using namespace std;
using clockT = chrono::high_resolution_clock;
using durationMS = chrono::duration<double, std::milli>;
// OpenCV
#include <opencv2/core/core.hpp>
// Ceres
#include <ceres/ceres.h>

// Basic ceres example
// solving curve fitting problem
// using nonlinear optimization methods
double a = 1.0, b = 2.0, c = 1.0; // parameters
int N = 200;                      // number of points
double w_sigma = 0.25;            // sigma of noise
cv::RNG rng;                      // random number generator

// Cost function
struct CostFunction
{
    // data
    const double _x, _y;

    // constructor
    CostFunction(double x, double y) : _x(x), _y(y) {}

    // residual
    template <typename T>
    bool operator()(const T *const abc, T *residual) const
    {
        // example nonlinear function
        // y = exp(ax^2 + bx + c) + w
        // where w ~ N(0, sigma) is the additive noise
        // thus, the residual is
        // y_hat - exp(ax^2 + bx + c)
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) +
                                         abc[1] * T(_x) + abc[2]);
        return true;
    }
};
using CeresAutoDiffCostFunction =
    ceres::AutoDiffCostFunction<CostFunction, 1, 3>;

// Curve function
inline double fun(double x) {
    return exp(a 
    
    
    *x * x + b * x + c) + rng.gaussian(w_sigma);
}

int main(int argc, char **argv)
{
    double abc[3] = {0, 0, 0};   // estimated parameters
    vector<double> xData, yData; // measured data
    // generate observations
    cout << "Generating observations... " << endl;
    for (int i = 0; i < N; ++i)
    {
        double x = i / double(N);
        xData.push_back(x);
        yData.push_back(fun(x));
    }
    // configure ceres to solve nonlinear least squares problem
    ceres::Problem problem;
    for (int i = 0; i < N; ++i)
    {
        problem.AddResidualBlock(
            new CeresAutoDiffCostFunction(new CostFunction(xData[i], yData[i])),
            nullptr, // kernel function, not used
            abc      // estimated parameters
        );
    }
    // configure solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    // show output to cout
    options.minimizer_progress_to_stdout = true;

    // summary for optimizer
    ceres::Solver::Summary summary;
    auto t0 = clockT::now();
    // start optimization
    ceres::Solve(options, &problem, &summary);
    durationMS timeUsed = clockT::now() - t0;
    cout << "Time used by the solver: " << timeUsed.count() << " ms" << endl;

    // output results
    cout << summary.BriefReport() << endl;
    cout << "Estimated parameters: ";
    for (const auto &x : abc)
        cout << x << " ,";
    cout << endl;

    return EXIT_SUCCESS;
}
