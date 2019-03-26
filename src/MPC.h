#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
namespace MPC {

struct MPCResult {
  MPCResult(std::vector<double>&& x_vals,
            std::vector<double>&& y_vals,
            double steer,
            double throttle) : mpc_x_vals(x_vals),
                               mpc_y_vals(y_vals),
                               steer_value(steer),
                               throttle_value(throttle) {}
  std::vector<double> mpc_x_vals;
  std::vector<double> mpc_y_vals;
  double steer_value;
  double throttle_value;
};

class MPController {
 public:
  MPController();

  virtual ~MPController();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  MPCResult Solve(const Eigen::VectorXd &state, const Eigen::VectorXd &coeffs);
};
}  // MPC

#endif  // MPC_H
