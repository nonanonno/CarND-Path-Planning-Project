#ifndef DRIVE_H
#define DRIVE_H
#include "json.hpp"
#include "spline.h"
#include "util.h"
#include <iostream>
#include <vector>

namespace drive
{
struct Telemetry
{
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;
	std::vector<double> previous_path_x;
	std::vector<double> previous_path_y;
	double end_path_s;
	double end_path_d;
	std::vector<Car> sensor_fusion;

	Telemetry(const nlohmann::json &obj);
};

std::string createMessage(const std::vector<double> &next_x, const std::vector<double> &next_y);

static bool collide(double target_s, double car_s, double car_v, int lane, double dt, const std::vector<Car> &sensor)
{
	std::cout << "dt:" << dt << std::endl;
	for (auto &car : sensor)
	{
		auto d = car.d;
		if (2 + 4 * lane - 2 < d && d < 2 + 4 * lane + 2)
		{
			auto check_speed = sqrt(car.vx * car.vx + car.vy * car.vy);
			double check_s = car.s + check_speed * dt;
			if (car.s > car_s && car.s - car_s < 30)
			{
				return true;
			}
			if (check_s - target_s > -10 && check_s - target_s < 30)
			{
				std::cout << "col:" << check_s << "," << check_speed << "," << target_s << "," << car_s << std::endl;
				return true;
			}
			// if (target_s < check_s)
			// { // forward
			// 	if (check_s - (target_s + 10) < 0)
			// 	{
			// 		return true;
			// 	}
			// }
			// else
			// {
			// 	if ((target_s - 5) - check_s < 0)
			// 	{
			// 		return true;
			// 	}
			// }
		}
	}
	return false;
}

static double forwardCar(int lane, double car_s, const std::vector<Car> &sensor)
{
	double f_vel = -1;
	double f_s = 10000;

	for (auto &car : sensor)
	{
		auto d = car.d;
		if (d < 2 + 4 * lane + 2 && d > 2 + 4 * lane - 2)
		{
			auto speed = sqrt(car.vx * car.vx + car.vy * car.vy) * 2;
			std::cout << "(" << car.vx << ", " << car.vy << "), ";
			if (car.s > car_s && f_s > car.s)
			{
				f_s = car.s;
				f_vel = speed;
			}
		}
	}
	return f_vel;
}
// v : mph
static void createTrajectory(const Telemetry &tele, const Map &map, int lane, float v, std::vector<double> &next_x_vals, std::vector<double> &next_y_vals)
{
	std::vector<double> ptsx;
	std::vector<double> ptsy;
	auto prev_size = tele.previous_path_x.size();
	if (prev_size < 2)
	{
		auto prev_x = tele.x - cos(tele.yaw);
		auto prev_y = tele.y - sin(tele.yaw);
		ptsx.push_back(prev_x);
		ptsy.push_back(prev_y);
		ptsx.push_back(tele.x);
		ptsy.push_back(tele.y);
	}
	else
	{
		ptsx.push_back(tele.previous_path_x[prev_size - 2]);
		ptsy.push_back(tele.previous_path_y[prev_size - 2]);
		ptsx.push_back(tele.previous_path_x[prev_size - 1]);
		ptsy.push_back(tele.previous_path_y[prev_size - 1]);
	}
	auto ref_x = ptsx[1];
	auto ref_y = ptsy[1];
	auto car_s = prev_size > 0 ? tele.end_path_s : tele.s;

	std::array<double, 2> wp[3];
	wp[0] = map.getXY(car_s + 30, 2 + 4 * lane);
	wp[1] = map.getXY(car_s + 60, 2 + 4 * lane);
	wp[2] = map.getXY(car_s + 90, 2 + 4 * lane);
	for (auto w : wp)
	{
		ptsx.push_back(w[0]);
		ptsy.push_back(w[1]);
	}
	for (int i = 0; i < ptsx.size(); i++)
	{
		auto tmp = rotate(ptsx[i] - ref_x, ptsy[i] - ref_y, -tele.yaw);
		ptsx[i] = tmp[0];
		ptsy[i] = tmp[1];
	}

	tk::spline s;
	s.set_points(ptsx, ptsy);
	next_x_vals = tele.previous_path_x;
	next_y_vals = tele.previous_path_y;
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x * target_x + target_y * target_y);
	double x_add_on = 0;

	for (int i = 1; i <= 50 - tele.previous_path_x.size(); i++)
	{
		double N = (target_dist / (0.02 * v / 2.24));
		double x_point = x_add_on + (target_x) / N;
		double y_point = s(x_point);
		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		auto tmp = rotate(x_ref, y_ref, tele.yaw);

		x_point = tmp[0];
		y_point = tmp[1];

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}
}
} // namespace drive

#endif