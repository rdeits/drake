#include <mex.h>
#include <Eigen/Core>
#include "splineGeneration.h"
#include "drakeUtil.h"
#include <iostream>
#include <limits>

using namespace std;
using namespace Eigen;

const int GRID_STEPS = 10;
const int Q_SIZE = 2;

Matrix<double, Q_SIZE, Q_SIZE> computeQ(double t0, double t1) {
  Matrix<double, Q_SIZE, Q_SIZE> Q;
  Q << std::pow(2.0, 2.0) * (t1 - t0), 6.0 * std::pow(t1 - t0, 2.0),
      6.0 * std::pow(t1 - t0, 2.0), std::pow(6.0, 2.0) / 3.0 * std::pow(t1 - t0, 3.0);
  return Q;
}

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {
  string usage = "[coefs, ts, objective_value] = twoWaypointCubicSplineFreeKnotTimesmex.cpp(t0, tf, xs, xd0, xdf)";
  if (nrhs != 5)
    mexErrMsgIdAndTxt("Drake:twoWaypointCubicSplineFreeKnotTimesmex.cpp:WrongNumberOfInputs", usage.c_str());
  if (nlhs < 2 || nlhs > 3)
    mexErrMsgIdAndTxt("Drake:twoWaypointCubicSplineFreeKnotTimesmex.cpp:WrongNumberOfOutputs", usage.c_str());

  double t0 = mxGetPrSafe(prhs[0])[0];
  double tf = mxGetPrSafe(prhs[1])[0];
  MatrixXd xs = matlabToEigen<Dynamic, Dynamic>(prhs[2]);
  auto xd0 = matlabToEigen<Dynamic, 1>(prhs[3]);
  auto xdf = matlabToEigen<Dynamic, 1>(prhs[4]);

  mwSize ndof = xs.rows();
  mwSize num_segments = 3;
  mwSize num_coeffs_per_segment = 4;
  mwSize dims[] = {ndof, num_segments, num_coeffs_per_segment};
  plhs[0] = mxCreateNumericArray(num_segments, dims, mxDOUBLE_CLASS, mxREAL);

  std::vector<double> segment_times;
  segment_times.resize(num_segments + 1);
  segment_times[0] = t0;
//  segment_times[1] = 1.0;
//  segment_times[2] = 2.0;
  segment_times[num_segments] = tf;
  std::vector<double> best_segment_times = segment_times;
  double t_step = (tf - t0) / GRID_STEPS;
  double min_objective_value = numeric_limits<double>::infinity();

  for (int t1_index = 0; t1_index < GRID_STEPS; t1_index++) {
    segment_times[1] = t0 + t1_index * t_step;
    for (int t2_index = t1_index; t2_index < GRID_STEPS; t2_index++) {
      segment_times[2] = t0 + t2_index * t_step;

      bool valid_solution = true;
      double objective_value = 0.0;
      for (int dof = 0; dof < ndof && valid_solution; dof++) {
        try {
          PiecewisePolynomial spline = twoWaypointCubicSpline(segment_times, xs(dof, 0), xd0[dof], xs(dof, 3), xdf[dof], xs(dof, 1), xs(dof, 2));
          // objective value using Q matrices:
          for (int i = 0; i < num_segments; i++) {
            auto Q = computeQ(spline.getStartTime(i), spline.getEndTime(i));
            auto coeffs_part = spline.getPolynomial(i).getCoefficients().bottomRows<Q_SIZE>();
            objective_value += (coeffs_part.transpose() * Q * coeffs_part).value();
          }

          // objective value using PiecewisePolynomial methods:
//          PiecewisePolynomial acceleration_squared = spline.derivative(2);
//          acceleration_squared *= acceleration_squared;
//          PiecewisePolynomial acceleration_squared_integral = acceleration_squared.integral();
//          objective_value += acceleration_squared_integral.value(spline.getEndTime()) - acceleration_squared_integral.value(spline.getStartTime());
        }
        catch (ConstraintMatrixSingularError& e) {
          valid_solution = false;
        }
      }

      if (valid_solution && objective_value < min_objective_value) {
        best_segment_times[1] = segment_times[1];
        best_segment_times[2] = segment_times[2];
        min_objective_value = objective_value;
      }
    }
  }

  for (mwSize dof = 0; dof < ndof; dof++) {
    PiecewisePolynomial spline = twoWaypointCubicSpline(best_segment_times, xs(dof, 0), xd0[dof], xs(dof, 3), xdf[dof], xs(dof, 1), xs(dof, 2));
    for (mwSize segment_index = 0; segment_index < spline.getNumberOfSegments(); segment_index++) {
      for (mwSize coefficient_index = 0; coefficient_index < num_coeffs_per_segment; coefficient_index++) {
        mwSize sub[] = {dof, segment_index, num_coeffs_per_segment - coefficient_index - 1}; // Matlab's reverse coefficient indexing...
        *(mxGetPr(plhs[0]) + sub2ind(3, dims, sub)) = spline.getPolynomial(segment_index).getCoefficients()[coefficient_index];
      }
    }
  }
  plhs[1] = stdVectorToMatlab(best_segment_times);

  if (nlhs > 2)
    plhs[2] = mxCreateDoubleScalar(min_objective_value);
}
