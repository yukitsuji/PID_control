#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  double Kp = 0.071;
  // double Kp = 0.05;
  double Ki = 0.0584992;
  // double Ki = 0.05;
  double Kd = 0.0680043;
  // double Kd = 0.05;
  pid.Init(Kp, Ki, Kd);

  double dKp = 0.003;
  double dKi = 0.003;
  double dKd = 0.003;
  std::vector<double> dp = {dKp, dKi, dKd};

  bool is_twiddled = true; //TODO: Please set false if optimize hyper parameter.
  Twiddle tw;
  tw.Init(dp);
  double dp_thresh = 0.0005;

  int try_road = 0;

  h.onMessage([&pid, &tw, &dp_thresh, &is_twiddled, &dp, &try_road](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event


    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          bool crash = false;
          if (std::abs(cte) > 5.0) {
            crash = true;
          }

          if (try_road == 3000) {
            crash = true;
          }

          try_road += 1;

          // Calculate optimized PID parameter.
          if (!is_twiddled) {
            if (crash){
              std::cout << "Kp = " << pid.Kp << ", Ki = " << pid.Ki << ", Kd = " << pid.Kd << std::endl;
              double error = pid.TotalError() / try_road;
              std::cout << "error " << error << " try " << try_road << std::endl;
              std::vector<double> p = {pid.Kp, pid.Ki, pid.Kd};
              p = tw.UpdateParam(error, p, try_road);
              pid.Init(p[0], p[1], p[2]);
              // reset environment
              try_road = 0;
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              return;
            }

            if ((tw.dp_sum() < dp_thresh)) {
              is_twiddled = true;
            }
          }

          // Update error for Calculating steering angle.
          pid.UpdateError(cte);

          /*
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          */
          // Execute PID control.
          // std::cout << "Kp = " << pid.Kp << ", Ki = " << pid.Ki << ", Kd = " << pid.Kd << std::endl;
          steer_value = pid.Predict();

          // Restrice steering value
          if (steer_value < -1.0) {
            steer_value = -1.0;
          } else if (steer_value > 1.0) {
            steer_value = 1.0;
          }

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.7;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    // std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
