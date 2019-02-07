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

	double ref_vel = 0;

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

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					std::vector<double> ptsx, ptsy;

					auto ref_x = tele.x;
					auto ref_y = tele.y;
					auto ref_yaw = deg2rad(tele.yaw);

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
					bool too_close = false;

					for (int i = 0; i < tele.sensor_fusion.size(); i++)
					{
						float d = tele.sensor_fusion[i].d;
						if (d < 2 + 4 * lane + 2 && d > 2 + 4 * lane - 2)
						{
							double vx = tele.sensor_fusion[i].vx;
							double vy = tele.sensor_fusion[i].vy;
							double check_speed = sqrt(vx * vx + vy * vy);
							double check_car_s = tele.sensor_fusion[i].s;
							check_car_s += (double)prev_size * 0.02 * check_speed;
							std::cout << check_car_s << ", ";
							if (check_car_s > car_s && check_car_s - car_s < 30)
							{
								too_close = true;
								if (lane > 0)
								{
									lane = 0;
								}
							}
						}
					}

					if (too_close)
					{
						ref_vel -= 0.224;
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

						ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
						ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
					}

					tk::spline s;
					s.set_points(ptsx, ptsy);

					// less than 50
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

						x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
						y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

						x_point += ref_x;
						y_point += ref_y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
					}

					// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

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
