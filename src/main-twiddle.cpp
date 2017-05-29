#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>

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

std::string slurp(std::ifstream& in) {
  // Read entire file into a string.
  std::stringstream sstr;
  sstr << in.rdbuf();
  return sstr.str();
}


int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.

  pid.Init(0.2, 0.004, 3.0);
  // Rate of change of parameters.
  double dp [3] = {0.2, 0.004, 1.0};
  // Number of runs before a change to parameter is initiated.
  int sample_size = 50;
  double err = 0;
  double besterr = -1;
  // Iterator to track current number of moves.
  int it = 0;
  // Iterator to track whether we are adjusting p, i, or d.
  int pi = 0;
  // Twiddle reference: https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/1397890f-71c5-4d83-aae7-0a031eedd1f2/concepts/34d4a65f-44d9-462f-b246-c2e653a19c1d
  // If cycle == 0, checks the upper limit (line 154 in twiddle reference).
  // If cycle == 1, checks the lower limit (line 161 in twiddle reference).
  int cycle = 0;

  const char* paramspath = "parameters.json";

  std::ifstream iparamsfile (paramspath);
  if (iparamsfile.is_open()) {
    std::string params_json( (std::istreambuf_iterator<char>(iparamsfile) ),
                             (std::istreambuf_iterator<char>()    ) );
    std::cout << "Loads following parameters from file:" << std::endl;
    std::cout << params_json << std::endl;

    json params = json::parse(params_json);
    sample_size = (int)params["sample_size"];
    besterr = (double)params["besterr"];
    pid.Kp = (double)params["p"];
    pid.Ki = (double)params["i"];
    pid.Kd = (double)params["d"];
    dp[0] = (double)params["dp"];
    dp[1] = (double)params["di"];
    dp[2] = (double)params["dd"];
    it = (int)params["it"];
    pi = (int)params["pi"];
    cycle = (int)params["cycle"];
    iparamsfile.close();
  }

  h.onMessage([&pid, &dp, &sample_size, &err, &besterr, &it, &pi, &cycle, &paramspath](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          err += cte*cte;
          std::cout << (it % sample_size);
          if (it % sample_size == 0 && it > 0) {
            if (it == sample_size) {
              // First time, set besterr and then update P coefficient.
              // Line 146 (outside loop) to 152 (first time) in twiddle reference.
              besterr = err;
              pid.Kp += dp[0];
              pi = 1;
              cycle = 1;
              // goto 1.
            }
            else {
              if (cycle == 0) {
                // 2 (line 162 in twiddle reference):
                if (err < besterr) {
                  besterr = err;
                  dp[pi] *= 1.1;
                }
                else {
                  switch(pi) {
                    case 0:
                      pid.Kp += dp[0];
                      break;
                    case 1:
                      pid.Ki += dp[1];
                      break;
                    case 2:
                      pid.Kd += dp[2];
                      break;
                  }
                  dp[pi] *= 0.9;
                }

                // Move to next parameter.
                pi = (pi + 1) % 3;

                // Line 152 (second to n time) in twiddle reference.
                switch(pi) {
                  case 0:
                    pid.Kp += dp[0];
                    break;
                  case 1:
                    pid.Ki += dp[1];
                    break;
                  case 2:
                    pid.Kd += dp[2];
                    break;
                }
              }
              else if (cycle == 1) {
                // 1 (line 155 in twiddle reference):
                if (err < besterr) {
                  besterr = err;
                  dp[pi] *= 1.1;
                }
                else {
                  switch(pi) {
                    case 0:
                      pid.Kp -= 2 * dp[0];
                      break;
                    case 1:
                      pid.Ki -= 2 * dp[1];
                      break;
                    case 2:
                      pid.Kd -= 2 * dp[2];
                      break;
                  }
                }
                // goto 2.
              }
              cycle = (cycle + 1) % 2;

            }
            err = 0;
          }
          ++it;
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          // These variables are needed so we may continue from previous training.
          std::cout << "Err: " << err << " Best: " << besterr << std::endl;
          std::cout << "p: " << pid.Kp << " i: " << pid.Ki << " d: " << pid.Kd << std::endl;
          std::cout << "dp: " << dp[0] << " di: " << dp[1] << " dd: " << dp[2] << std::endl;
          std::cout << "iteration: " << it << "param cycle: " << cycle << std::endl;

          // Store current parameters.
          json params = {
            {"sample_size", sample_size},
            {"besterr", besterr},
            {"p", pid.Kp}, {"i", pid.Ki}, {"d", pid.Kd},
            {"dp", dp[0]}, {"di", dp[1]}, {"dd", dp[2]},
            {"it", it}, {"pi", pi}, {"cycle", cycle}
          };

          std::ofstream paramsfile;
          paramsfile.open(paramspath);
          paramsfile << params.dump();
          paramsfile.close();

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["speed"] = speed;
          msgJson["throttle"] = 0.8;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
    std::cout << "Connected!!!" << std::endl;
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
