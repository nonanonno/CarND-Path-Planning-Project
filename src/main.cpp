#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "drive.h"
#include "json.hpp"
#include "spline.h"
#include "util.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

using namespace std;

// for convenience
using json = nlohmann::json;

class Sensor {};

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d
	// Waypoint map to read from
	Map map("../data/highway_map.csv");

	int lane = 1;

	double ref_vel_mps = 0; // mps
	constexpr double DT = 0.02;
	constexpr double ACC = 0.1; // mps;
	constexpr double SPEED_LIMIT = mph2mps(49.5);

	h.onMessage([&map, &lane, &ref_vel_mps](uWS::WebSocket<uWS::SERVER> ws,
	                                        char *data, size_t length,
	                                        uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket
		// message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		// auto sdata = string(data).substr(0, length);
		// cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = io::hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Main car's localization Data
					const drive::Telemetry tele(j[1]);

					auto prev_size =
					    tele.previous_path_x.size();
					auto this_car_s = tele.s;

					if (prev_size > 0) {
						this_car_s = tele.end_path_s;
					}

					std::vector<double> ptsx, ptsy;

					bool too_close = true;
					for (auto dir : {0, -1, 1}) {
						auto next_lane = lane + dir;
						if (next_lane < 0 ||
						    next_lane > 2) {
							continue;
						}
						auto collision =
						    drive::checkForwardCollide(
						        this_car_s, next_lane,
						        DT * prev_size,
						        tele.sensor_fusion);
						collision |=
						    drive::checkBackwardCollide(
						        this_car_s, next_lane,
						        DT * prev_size,
						        tele.sensor_fusion);
						// std::cout << next_lane << ":
						// " << collision << ", ";

						if (!collision) {
							lane = next_lane;
							too_close = false;
							break;
						}
					}

					if (too_close) {
						auto forward_car =
						    drive::findForwardCar(
						        lane, tele.s,
						        tele.sensor_fusion);
						// std::cout <<
						// forward_car.dump() <<
						// std::endl;
						if (forward_car.id != -1 &&
						    (ref_vel_mps >
						         forward_car
						             .speed_mps() ||
						     (forward_car.s -
						      this_car_s) < 5)) {
							std::cout
							    << "ref:"
							    << ref_vel_mps
							    << "mps, forward:"
							    << forward_car
							           .speed_mps()
							    << "mps, "
							       "this_car_s: "
							    << this_car_s
							    << "m, "
							       "forward_car_s: "
							    << forward_car.s
							    << "m" << std::endl;
							ref_vel_mps -= ACC;
						}
					} else if (ref_vel_mps < SPEED_LIMIT) {
						ref_vel_mps += ACC;
					}

					vector<double> next_x_vals;
					vector<double> next_y_vals;
					drive::createTrajectory(
					    tele, map, lane, ref_vel_mps,
					    next_x_vals, next_y_vals);
					// TODO: define a path made up of (x,y)
					// points that the car will visit
					// sequentially every .02 seconds

					auto msg = drive::createMessage(
					    next_x_vals, next_y_vals);
					// this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(),
					        uWS::OpCode::TEXT);
				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(),
				        uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req,
	                   char *data, size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection(
	    [&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
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
