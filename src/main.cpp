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

class Sensor
{
};

int main()
{
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d
	// Waypoint map to read from
	Map map("../data/highway_map.csv");

	int lane = 1;

	double ref_vel = 0; // mph

	h.onMessage([&map, &lane, &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
										uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{

			auto s = io::hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry")
				{
					// j[1] is the data JSON object

					// Main car's localization Data
					const drive::Telemetry tele(j[1]);

					auto prev_size = tele.previous_path_x.size();
					auto car_s = tele.s;

					if (prev_size > 0)
					{
						car_s = tele.end_path_s;
					}

					std::vector<double> ptsx, ptsy;

					auto ref_x = tele.x;
					auto ref_y = tele.y;
					auto ref_yaw = tele.yaw;

					if (prev_size < 2)
					{
						auto prev_car_x = tele.x - cos(tele.yaw);
						auto prev_car_y = tele.y - sin(tele.yaw);
						ptsx.push_back(prev_car_x);
						ptsx.push_back(tele.x);
						ptsy.push_back(prev_car_y);
						ptsy.push_back(tele.y);
					}
					else
					{
						ref_x = tele.previous_path_x[prev_size - 1];
						ref_y = tele.previous_path_y[prev_size - 1];

						double ref_x_prev = tele.previous_path_x[prev_size - 2];
						double ref_y_prev = tele.previous_path_y[prev_size - 2];
						ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

						ptsx.push_back(ref_x_prev);
						ptsx.push_back(ref_x);
						ptsy.push_back(ref_y_prev);
						ptsy.push_back(ref_y);
					}

					// create 3 trajectories
					bool too_close = true;
					for (auto dir : {0, -1, 1})
					{
						auto next_lane = lane + dir;
						if (next_lane < 0 || next_lane > 2)
						{
							continue;
						}
						auto collision = drive::collide(car_s, tele.s, min(ref_vel, tele.speed), next_lane, 0.02 * prev_size, tele.sensor_fusion);
						std::cout << next_lane << ": " << collision << ", ";

						if (!collision)
						{
							lane = next_lane;
							too_close = false;
							break;
						}
					}
					std::cout << ", " << ref_vel << ", " << tele.speed << std::endl;

					if (too_close)
					{
						auto fvel = drive::forwardCar(lane, tele.s, tele.sensor_fusion);
						std::cout << fvel << std::endl;
						if (ref_vel > fvel)
						{
							ref_vel -= 0.224;
						}
					}
					else if (ref_vel < 49.5)
					{
						ref_vel += 0.224;
					}

					auto next_wp0 = map.getXY(car_s + 30, 2 + 4 * lane);
					auto next_wp1 = map.getXY(car_s + 60, 2 + 4 * lane);
					auto next_wp2 = map.getXY(car_s + 90, 2 + 4 * lane);

					ptsx.push_back(next_wp0[0]);
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);

					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);

					for (int i = 0; i < ptsx.size(); i++)
					{
						// shift car reference angle to 0 degree
						auto shift_x = ptsx[i] - ref_x;
						auto shift_y = ptsy[i] - ref_y;
						auto tmp = rotate(shift_x, shift_y, -ref_yaw);
						ptsx[i] = tmp[0];
						ptsy[i] = tmp[1];
					}

					tk::spline s;
					s.set_points(ptsx, ptsy);

					vector<double> next_x_vals;
					vector<double> next_y_vals;
					// less than 50
					/*
					for (int i = 0; i < tele.previous_path_x.size(); i++)
					{
						next_x_vals.push_back(tele.previous_path_x[i]);
						next_y_vals.push_back(tele.previous_path_y[i]);
					}

					double target_x = 30.0;
					double target_y = s(target_x);
					double target_dist = sqrt(target_x * target_x + target_y * target_y);
					double x_add_on = 0;

					for (int i = 1; i <= 50 - tele.previous_path_x.size(); i++)
					{
						double N = (target_dist / (0.02 * ref_vel / 2.24));
						double x_point = x_add_on + (target_x) / N;
						double y_point = s(x_point);
						x_add_on = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						auto tmp = rotate(x_ref, y_ref, ref_yaw);

						x_point = tmp[0];
						y_point = tmp[1];

						x_point += ref_x;
						y_point += ref_y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
					}
					*/
					drive::createTrajectory(tele, map, lane, ref_vel, next_x_vals, next_y_vals);
					// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

					auto msg = drive::createMessage(next_x_vals, next_y_vals);
					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
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

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
						   char *message, size_t length) {
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
