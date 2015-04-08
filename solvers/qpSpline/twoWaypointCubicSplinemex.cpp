#include <mex.h>
#include <Eigen/Core>
#include "splineGeneration.h"
#include "drakeUtil.h"
#include <iostream>

using namespace std;
using namespace Eigen;

const int Q_SIZE = 2;

Matrix<double, Q_SIZE, Q_SIZE> computeQ(double t0, double t1) {
  Matrix<double, Q_SIZE, Q_SIZE> Q;
  Q << std::pow(2.0, 2.0) * (t1 - t0), 6.0 * std::pow(t1 - t0, 2.0),
      6.0 * std::pow(t1 - t0, 2.0), std::pow(6.0, 2.0) / 3.0 * std::pow(t1 - t0, 3.0);
  return Q;
}

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {
  string usage = "[coefs, objval] = twoWaypointCubicSplinemex(ts, xs, xd0, xdf)";
  if (nrhs != 4)
    mexErrMsgIdAndTxt("Drake:twoWaypointCubicSplinemex:WrongNumberOfInputs", usage.c_str());
  if (nlhs > 2)
    mexErrMsgIdAndTxt("Drake:twoWaypointCubicSplinemex:WrongNumberOfOutputs", usage.c_str());

  const std::vector<double> segment_times = matlabToStdVector(prhs[0]);
  MatrixXd xs = matlabToEigen<Dynamic, Dynamic>(prhs[1]);
  auto xd0 = matlabToEigen<Dynamic, 1>(prhs[2]);
  auto xdf = matlabToEigen<Dynamic, 1>(prhs[3]);

  mwSize ndof = xs.rows();
  mwSize num_segments = 3;
  mwSize num_coeffs_per_segment = 4;
  mwSize dims[] = {ndof, num_segments, num_coeffs_per_segment};
  plhs[0] = mxCreateNumericArray(num_segments, dims, mxDOUBLE_CLASS, mxREAL);
  double objective_value = 0.0;
  for (mwSize dof = 0; dof < ndof; dof++) {
    PiecewisePolynomial spline = twoWaypointCubicSpline(segment_times, xs(dof, 0), xd0[dof], xs(dof, 3), xdf[dof], xs(dof, 1), xs(dof, 2));
    for (int i = 0; i < num_segments; i++) {
      auto Q = computeQ(spline.getStartTime(i), spline.getEndTime(i));
      auto coeffs_part = spline.getPolynomial(i).getCoefficients().bottomRows<Q_SIZE>();
      objective_value += (coeffs_part.transpose() * Q * coeffs_part).value();
    }

    for (mwSize segment_index = 0; segment_index < spline.getNumberOfSegments(); segment_index++) {
      for (mwSize coefficient_index = 0; coefficient_index < num_coeffs_per_segment; coefficient_index++) {
        mwSize sub[] = {dof, segment_index, num_coeffs_per_segment - coefficient_index - 1}; // Matlab's reverse coefficient indexing...
        *(mxGetPr(plhs[0]) + sub2ind(3, dims, sub)) = spline.getPolynomial(segment_index).getCoefficients()[coefficient_index];
      }
    }
  }

  if (nlhs > 1) {
    plhs[1] = mxCreateDoubleScalar(objective_value);
  }
}
