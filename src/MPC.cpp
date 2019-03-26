#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

#include "helpers.h"

using CppAD::AD;
using Eigen::VectorXd;

namespace MPC {

// Set number of steps and time step.
constexpr size_t N = 15;
constexpr double dt = 0.1;

constexpr size_t n_state = 6;
constexpr size_t n_actuator = 2;
constexpr size_t n_vars = N * n_state + (N - 1) * n_actuator;
constexpr size_t n_constraints = N * n_state;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
constexpr double Lf = 2.67;

// Reference velocity.
constexpr double ref_v = 20.0;

// Set convenient offsets.
constexpr size_t x_offset = 0;
constexpr size_t y_offset = x_offset + N;
constexpr size_t psi_offset = y_offset + N;
constexpr size_t v_offset = psi_offset + N;
constexpr size_t cte_offset = v_offset + N;
constexpr size_t epsi_offset = cte_offset + N;
constexpr size_t delta_offset = epsi_offset + N;
constexpr size_t a_offset = delta_offset + N - 1;

using Dvector = std::vector<double>;
constexpr double cte_gain = 200;
constexpr double epsi_gain = 700;
constexpr double vref_gain = 0.2;
constexpr double delta_diff_gain = 70.0;
constexpr double adiff_gain = 10.0;

class FG_eval {
 public:
  using ADvector = std::vector<AD<double>>;
  // Fitted polynomial coefficients
  VectorXd coeffs;
  FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

  void operator()(ADvector& fg, const ADvector& vars) {

    // Add costs.
    fg[0] = 0.0;
    for (size_t t = 0; t < N; ++t) {
      AD<double> cte = vars[cte_offset + t];
      fg[0] += cte_gain * cte * cte; 
      AD<double> epsi = vars[epsi_offset + t];
      fg[0] += epsi_gain * epsi * epsi;
      AD<double> vel_diff = vars[v_offset + t] - ref_v;
      fg[0] += vref_gain * vel_diff * vel_diff;
    }

    for (size_t t = 0; t < N-1; ++t) {
      AD<double> delta0 = vars[delta_offset + t - 1];
      fg[0] + delta0 * delta0;
      AD<double> a0 = vars[a_offset + t - 1];
      fg[0] + a0 * a0;
    }
    for (size_t t = 0; t < N-2; ++t) {
      AD<double> delta0 = vars[delta_offset + t - 1];
      AD<double> delta1 = vars[delta_offset + t];
      AD<double> delta_diff = delta1 - delta0;
      fg[0] += delta_diff_gain * delta_diff * delta_diff;
      AD<double> a0 = vars[a_offset + t - 1];
      AD<double> a1 = vars[a_offset + t];
      AD<double> a_diff = a1 - a0;
      fg[0] += adiff_gain * a_diff * a_diff;
    }

    // Set initial values.
    fg[1 + x_offset] = vars[x_offset];
    fg[1 + y_offset] = vars[y_offset];
    fg[1 + psi_offset] = vars[psi_offset];
    fg[1 + v_offset] = vars[v_offset];
    fg[1 + cte_offset] = vars[cte_offset];
    fg[1 + epsi_offset] = vars[epsi_offset];

     //Set constraints.
     //Set values for remainder of rollout.
    for (size_t t = 1; t < N; ++t) {
 
      // The state at time t.
      AD<double> x0 = vars[x_offset + t - 1];
      AD<double> x1 = vars[x_offset + t];

      AD<double> y0 = vars[y_offset + t - 1];
      AD<double> y1 = vars[y_offset + t];

      AD<double> psi0 = vars[psi_offset + t - 1];
      AD<double> psi1 = vars[psi_offset + t];

      AD<double> v0 = vars[v_offset + t - 1];
      AD<double> v1 = vars[v_offset + t];

      AD<double> cte0 = vars[cte_offset + t - 1];
      AD<double> cte1 = vars[cte_offset + t];

      AD<double> epsi0 = vars[epsi_offset + t - 1];
      AD<double> epsi1 = vars[epsi_offset + t];

      AD<double> delta0 = vars[delta_offset + t - 1];
      AD<double> a0 = vars[a_offset + t - 1];

      AD<double> f0 = helpers::polyeval(coeffs, x0);
      AD<double> psides0 = helpers::polytangent(coeffs, x0);

      // x update.
      fg[1 + x_offset + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      // y update.
      fg[1 + y_offset + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      // psi update.
      fg[1 + psi_offset + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      // v update.
      fg[1 + v_offset + t] = v1 - (v0 + a0 * dt);
      // CTE update.
      fg[1 + cte_offset + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      // epsi update.
      fg[1 + epsi_offset + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }

  }
};
//
// MPC class definition implementation.
//
MPController::MPController() {}
MPController::~MPController() {}

MPCResult MPController::Solve(const VectorXd& state, const VectorXd& coeffs) {
  bool ok = true;

  // Initial value of the independent variables.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }
  vars[x_offset] = state[0];
  vars[y_offset] = state[1];;
  vars[psi_offset] = state[2];;
  vars[v_offset] = state[3];;
  vars[cte_offset] = state[4];;
  vars[epsi_offset] = state[5];;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for (size_t i = 0; i < delta_offset; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  for (size_t i = delta_offset; i < a_offset; ++i) {
    vars_lowerbound[i] = helpers::deg2rad(-25);
    vars_upperbound[i] = helpers::deg2rad(25);
  }
  for (size_t i = a_offset; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  //// Lower and upper limits for the constraints
  //// Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;
  }
  constraints_lowerbound[x_offset] = state[0];
  constraints_upperbound[x_offset] = state[0];
  constraints_lowerbound[y_offset] = state[1];
  constraints_upperbound[y_offset] = state[1];
  constraints_lowerbound[psi_offset] = state[2];
  constraints_upperbound[psi_offset] = state[2];
  constraints_lowerbound[v_offset] = state[3];
  constraints_upperbound[v_offset] = state[3];
  constraints_lowerbound[cte_offset] = state[4];
  constraints_upperbound[cte_offset] = state[4];
  constraints_lowerbound[epsi_offset] = state[5];
  constraints_upperbound[epsi_offset] = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  if (!ok) {
      std::cerr<<"SOLVE NOT OK"<<std::endl;
  }

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  std::vector<double>::const_iterator solution_begin = solution.x.cbegin();
  std::vector<double> x_vals(solution_begin + x_offset,
                             solution_begin + y_offset);
  std::vector<double> y_vals(solution_begin + y_offset,
                             solution_begin + psi_offset);
  double steer = solution.x[delta_offset];
  double throttle = solution.x[a_offset];

  return MPCResult(std::move(x_vals), std::move(y_vals), steer, throttle);
}
}  // MPC
