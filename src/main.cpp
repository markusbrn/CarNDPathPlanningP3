#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <ctime>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "assist.h"
#include "veh.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  static auto start_sim = std::chrono::steady_clock::now();
  static auto start_timing = std::chrono::steady_clock::now();

  //define ego vehicle
  static Vehicle ego;
  ego.lane = 1;
  ego.lane_cmd = 1;
  ego.state = "KL";
  ego.vel_cmd = 0.;
  ego.vel_max = 49.5*0.44704;
  ego.vel_inc_max = 5*0.02;
  ego.vel_inc = ego.vel_inc_max;
  Eigen::VectorXd state_init(6);
  ego.state_vector = state_init;
  Eigen::VectorXd sd_init(3);
  ego.si = sd_init;
  ego.sf = sd_init;
  ego.di = sd_init;
  ego.df = sd_init;

  // map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;


  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	//Load up map waypoints
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
        	auto act = std::chrono::steady_clock::now();
        	auto dt = act - start_timing;
        	double dt_sec = double((std::chrono::duration_cast<std::chrono::microseconds>(dt)).count())/CLOCKS_PER_SEC;
        	//cout << "Time: " << dt_sec << endl;
        	auto time = act - start_sim;
        	double time_sec = double((std::chrono::duration_cast<std::chrono::microseconds>(time)).count())/CLOCKS_PER_SEC;
        	start_timing = std::chrono::steady_clock::now();
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	ego.x_act = j[1]["x"];
          	ego.y_act = j[1]["y"];
          	ego.s_act = j[1]["s"];
          	ego.d_act = j[1]["d"];
          	ego.yaw_act = deg2rad(double(j[1]["yaw"]));
          	ego.speed_act = double(j[1]["speed"]) / 2.24;
          	if(ego.d_act < 4) {
          		ego.lane = 0;
          	} else if(4 <= ego.d_act && ego.d_act < 8) {
          		ego.lane = 1;
          	} else {
          		ego.lane = 2;
          	}

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	ego.previous_path_x.clear();
          	ego.previous_path_y.clear();
          	for(unsigned int i=0; i<previous_path_x.size(); i++) {
          		ego.previous_path_x.push_back(previous_path_x[i]);
          		ego.previous_path_y.push_back(previous_path_y[i]);
          	}
          	// Previous path's end s and d values 
          	ego.end_path_s = j[1]["end_path_s"];
          	ego.end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	vector<Vehicle> predictions;
          	for(unsigned int i=0; i<sensor_fusion.size(); i++) {
          		if(fabs(ego.s_act-double(sensor_fusion[i][5])) < 50) {
          			Vehicle car;
          			car.x_act = sensor_fusion[i][1];
          			car.y_act = sensor_fusion[i][2];
          			car.yaw_act = ego.yaw_act;
          			double vx = sensor_fusion[i][3];
          			double vy = sensor_fusion[i][4];
          			car.speed_act = sqrt(pow(vx,2)+pow(vy,2)) / 2.24;
          			car.s_act = sensor_fusion[i][5];
          			car.d_act = sensor_fusion[i][6];

          			if(car.d_act < 4) {
          				car.lane = 0;
          			} else if(4 <= car.d_act && car.d_act < 8) {
          				cout << car.d_act <<endl;
          				car.lane = 1;
          			} else {
          				car.lane = 2;
          			}

          			//generate location predictions for current vehicle
          			car.predict(map_waypoints_s, map_waypoints_x, map_waypoints_y);
          			predictions.push_back(car);
          		}
          	}

          	ego.choose_next_state(predictions, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	// END

          	json msgJson;
          	msgJson["next_x"] = ego.next_path_x;
          	msgJson["next_y"] = ego.next_path_y;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
