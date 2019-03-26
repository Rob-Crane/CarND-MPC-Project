#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::RowVectorXd;

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC::MPController mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = helpers::hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v0 = j[1]["speed"];

          RowVectorXd ptsx_global(
              Eigen::Map<Eigen::RowVectorXd>(ptsx.data(), ptsx.size()));
          RowVectorXd ptsy_global(
              Eigen::Map<Eigen::RowVectorXd>(ptsy.data(), ptsy.size()));
          helpers::CoordMatrix pts(2, ptsx.size());
          pts << ptsx_global, ptsy_global;
          helpers::CoordMatrix local_pts =
              helpers::vehicleFrameTransform(pts, px, py, psi);
          VectorXd x_local = local_pts.row(0);
          VectorXd y_local = local_pts.row(1);

          VectorXd coeffs = helpers::polyfit(x_local, y_local, 3);
          // Compute additional state variables.
          double cte0 = coeffs[0];
          double epsi0 = -coeffs[1];

          VectorXd state(6);
          constexpr double latency = 0.1;
          const double Lf = 2.67;
          double delta = -double(j[1]["steering_angle"]);
          double a = j[1]["throttle"];
          double x1 = latency * v0;
          double psi1 = latency * v0 * delta / Lf;
          double v1 = v0 + latency * a;
          double cte1 = cte0 + v0 * sin(epsi0) * latency;
          double epsi1 = epsi0 + v0 * delta / Lf * latency;
          state << x1,
                   0.0,
                   psi1,
                   v1,
                   cte1,
                   epsi1;

          MPC::MPCResult mpc_result = mpc.Solve(state, coeffs);

          json msgJson;
          msgJson["steering_angle"] = - mpc_result.steer_value;
          msgJson["throttle"] = mpc_result.throttle_value;

          // Display the MPC predicted trajectory
          msgJson["mpc_x"] = mpc_result.mpc_x_vals;
          msgJson["mpc_y"] = mpc_result.mpc_y_vals;
          // Display the waypoints/reference line
          vector<double> next_x_vals(x_local.data(),
                                     x_local.data() + x_local.size());
          vector<double> next_y_vals(y_local.data(),
                                     y_local.data() + y_local.size());
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(unsigned(latency * 1000)));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
