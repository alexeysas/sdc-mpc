#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// optimazied method to convert arrays of coordinates to car coordinates, 
// assumed that vectors are equal size - no special valdiation
void ConvertToVehicle(double vx, double vy, vector<double> &x, vector<double> &y, double psi)
{
      double cos_psi = cos(psi);
      double sin_psi = sin(psi);
  
       for (unsigned int i = 0; i < x.size(); ++i) {
          double xn = x[i] - vx;
          double yn = y[i] - vy;
          x[i] = xn*cos_psi + yn*sin_psi;
          y[i] = yn*cos_psi - xn*sin_psi; 
       }
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
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
          double v = j[1]["speed"]; 
          
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // Convert speed to m/s as we have latency measured in seconds and we need to predict
          // car state based on latency and kinematic model
          v = v * MhpToMs; 
          
          // Get current values - used for prediction of car state
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          // Convert waipoints to to vehicle coordiantes,
          // Too main reasons to do this:
          // 1. Need to draw waypoints later and simulator accepts them in vehicle coordinates
          // 2. It is easier to calculate CTE in vehicle coordinates - just (y - f(x))
          ConvertToVehicle(px, py, ptsx, ptsy, psi);
       
          // Convert vehiclae coordinates to vehicle coordinats - pretty easy :)
          px = 0;
          py = 0;
          psi = 0;
       
          // Convert to Eighen
          Eigen::VectorXd ptsxe  = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
          Eigen::VectorXd ptsye  = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());
          
          // Fit polynomial 
          auto coeffs =  polyfit(ptsxe, ptsye, 3);
                                
          // Predict real car position using linematic model based on latency (old version)
          //double px_predicted = v * latency;
          //double py_predicted = 0;
          //double psi_predicted = -v * steer_value * latency / Lf;
          //double v_predicted = v + throttle_value * latency;
          
          // Calculate predicted cte and epsi based on the predicted values
          //double cte = py_predicted - polyeval(coeffs, px_predicted);
          //double epsi = psi_predicted - atan(coeffs[1] + 2 * coeffs[2] * px_predicted + 3 * coeffs[3] * px_predicted * px_predicted);

          // Improvements according to the review
          double cte = polyeval(coeffs, px); //Having all state variables zeros - simplified expression
          double epsi = -atan(coeffs[1]); //Having all state variables zeros - simplified expression

          //  change of sign because turning left is negative sign in simulator but positive yaw for MPC
          double delta = -steer_value; 
          psi = 0;
          double px_predicted = px + v * cos(psi) * latency; 
          double py_predicted  = py + v * sin(psi) * latency;
          
          cte = cte + v * sin(epsi) * latency;
          epsi = epsi + v * delta * latency / Lf;
          double psi_predicted  = psi +  v * delta * latency / Lf;
          double v_predicted = v + throttle_value * latency;

          // Create state vector and pass it to Solver 
          Eigen::VectorXd state(6);
          state << px_predicted, py_predicted, psi_predicted, v_predicted, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);

          // Get steer angle and throttle_value from Solver 
          steer_value = vars[0];
          throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / deg2rad(25) * Lf;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (unsigned int i = 0; i < (vars.size() - 2) / 2; ++i) {
            mpc_x_vals.push_back(vars[i * 2 + 2]);
            mpc_y_vals.push_back(vars[i * 2 + 3]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          // Show 25 points of referece line
          unsigned int N = 25;
          for (unsigned int i = 0; i < N; ++i) {
            double x = 0 + i * 2.25; 
            next_x_vals.push_back(x);
            next_y_vals.push_back(polyeval(coeffs, x));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
