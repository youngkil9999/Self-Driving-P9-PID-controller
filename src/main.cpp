#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <time.h>

using namespace std;

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
  // TODO: Initialize the pid variable.

//  pid.Init(0.2, 0.001, 0.02);
//  pid.Init(0.1, 0.001, 0.01);
//  pid.Init(0.221081, 0.000150046, 0.0117859); speed 0.2

    pid.Init(0.252628,0.00077475,2.16025);

  cout << "PRINT DUPLICATE CHECK" <<endl;

  cout<<pid.Kp<<' '<<pid.Ki<<' '<<pid.Kd<<endl;

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          double dt;
          double curr_time;
          double prev_time;
          double output;
          double best_err;

          if (pid.TWIDDLE == 0){

            pid.UpdateError(cte);

//              pid.num_step > 200 + pid.weight ||
//              (cte > 0.5 || cte < -0.5)&& pid.err > 0
            if (pid.num_step > 6000 + pid.weight && pid.err > 0 ) {

                pid.err = pid.TotalError(pid.err);

                cout << "Twiddle 0 " << endl;

                pid.best_err = pid.err;
                cout<<" Total Error is :" << pid.err << ", num step is : " << pid.num_step <<  endl;
                cout<< "best err is : " << pid.best_err << "Time period : " << pid.num_step + pid.weight << endl;
                cout << "Kp : " << pid.Kp << " Ki : " << pid.Ki << " Kd : " << pid.Kd << endl;
                cout << "dKp : " << pid.dKp << " dKi : " << pid.dKi << " dKd : " << pid.dKd << endl;
                cout << "p_error : " << pid.p_error << " i_error : " << pid.i_error << " d_error : " << pid.d_error << endl;

                pid.TWIDDLE = 1;
                pid.err = 0;
                pid.d_error = 0;
                pid.i_error = 0;
                pid.p_error = 0;
                pid.num_step = 0;
                pid.numOfstep = 0;

                pid.Restart(ws);

            }
          } else if(pid.TWIDDLE == 1){
//STEP for comparing TWIDDLE 0 error with TWIDDLE 1 error

//            cout << "Twiddle 1" <<endl;

            pid.twiddle(cte);

          } else if (pid.TWIDDLE ==2){

//            cout << "Twiddle 2" << endl;

          }

          output = - pid.Kp * pid.p_error - pid.Ki * pid.i_error - pid.Kd * pid.d_error ;

//          cout << output << endl;

          if (output < -1) {
                output = -1;
          }

          else if (output > 1) {
                output = 1;
          }

          steer_value = output;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);


          if (pid.sum == 1) {
//            cout << " sum is 1 so initialized" <<endl;
//            cout << "best_error is  " << pid.best_err <<endl;

            pid.sum = 0;
            pid.err = 0;
            pid.d_error = 0;
            pid.i_error = 0;
            pid.p_error = 0;
            pid.num_step = 0;
            pid.numOfstep = 0;
            pid.Restart(ws);

          } else if (pid.sum ==2){

              pid.sum = 0;
              pid.TWIDDLE = 0;
              pid.err = 0;
              pid.d_error = 0;
              pid.i_error = 0;
              pid.p_error = 0;
              pid.num_step = 0;
              pid.numOfstep = 0;
              pid.Restart(ws);
          }
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

