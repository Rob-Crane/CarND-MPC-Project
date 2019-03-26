#ifndef HELPERS_H
#define HELPERS_H

#include <string>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cmath>


namespace helpers {

using std::pow;
using CppAD::pow;
using CppAD::AD;
using std::atan;
using CppAD::atan;
using Eigen::VectorXd;
using Eigen::Dynamic;
using Eigen::Matrix;
using std::string;

using CoordMatrix = Matrix<double, 2, Dynamic>;
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions to fit and evaluate polynomials.
//

// Transform a set of points to the vehicle reference frame.
CoordMatrix vehicleFrameTransform(CoordMatrix pts,
                                 double px, double py, double psi);

// Evaluate a polynomial.
AD<double> polyeval(const VectorXd& coeffs, AD<double> x);

// Get the tangent angle of a polynomial at x.
//double polytangent(const VectorXd& coeffs, double x);
AD<double> polytangent(const VectorXd& coeffs, AD<double> x);

// Fit a polynomial.
// Adapted from:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(const VectorXd& xvals, const VectorXd& yvals, int order);

// For converting back and forth between radians and degrees.
static constexpr double pi() { return M_PI; }
static constexpr double deg2rad(double x) { return x * pi() / 180; }
static constexpr double rad2deg(double x) { return x * 180 / pi(); }

}  // helpers

#endif  // HELPERS_H
