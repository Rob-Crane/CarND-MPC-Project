#include "helpers.h"

#include <cmath>

#include "Eigen-3.3/Eigen/QR"

using std::string;
using Eigen::VectorXd;

string helpers::hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

CppAD::AD<double> helpers::polyeval(const VectorXd &coeffs, CppAD::AD<double> x) {
    CppAD::AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

CppAD::AD<double> helpers::polytangent(const VectorXd &coeffs, CppAD::AD<double> x) {
  VectorXd d_coeffs = coeffs;         // Make a copy.
  d_coeffs[coeffs.size() - 1] = 0.0;  // Highest order term is 0.
  int i = coeffs.size() - 2;
  while (i >= 0) {
    d_coeffs[i] = (i + 1) * coeffs[i + 1];
    --i;
  }
  CppAD::AD<double> d_poly = polyeval(d_coeffs, x);  // Compute first derivative.
  return CppAD::atan(d_poly);               // Find desired angle.
}

VectorXd helpers::polyfit(const VectorXd &xvals, const VectorXd &yvals,
                          int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; ++i) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);

  return result;
}

helpers::CoordMatrix helpers::vehicleFrameTransform(helpers::CoordMatrix pts,
                                                    double px, double py,
                                                    double psi) {
  pts.row(0).array() -= px;
  pts.row(1).array() -= py;

  // Apply rotation to put waypoints in
  double cos_psi = std::cos(psi);
  double sin_psi = std::sin(psi);
  Eigen::Matrix2d rotate;
  rotate << cos_psi, sin_psi, -sin_psi, cos_psi;
  return rotate * pts;
}
