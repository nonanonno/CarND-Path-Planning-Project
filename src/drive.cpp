#include "drive.h"
#include <cstdint>
#include <iostream>

namespace drive {

Telemetry::Telemetry(const nlohmann::json &obj)
    : x(obj["x"]), y(obj["y"]), s(obj["s"]), d(obj["d"]),
      yaw(deg2rad(obj["yaw"])), speed(obj["speed"]),
      previous_path_x(obj["previous_path_x"].size()),
      previous_path_y(obj["previous_path_y"].size()),
      end_path_s(obj["end_path_s"]), end_path_d(obj["end_path_d"]),
      sensor_fusion(obj["sensor_fusion"].size()) {
	for (size_t i = 0; i < previous_path_x.size(); i++) {
		previous_path_x[i] = obj["previous_path_x"][i];
		previous_path_y[i] = obj["previous_path_y"][i];
	}
	for (size_t i = 0; i < sensor_fusion.size(); i++) {
		auto item = obj["sensor_fusion"][i];
		sensor_fusion[i] = {
		    item[0], item[1], item[2], item[3],
		    item[4], item[5], item[6],
		};
	}
}

std::string createMessage(const std::vector<double> &next_x,
                          const std::vector<double> &next_y) {
	nlohmann::json j;
	j["next_x"] = next_x;
	j["next_y"] = next_y;
	return "42[\"control\"," + j.dump() + "]";
}

bool checkForwardCollide(double target_s, int target_lane, double target_time,
                         const std::vector<Car> &sensor) {
	for (auto &car : sensor) {
		auto d = car.d;
		if (target_lane == car.lane()) {
			auto check_speed_mps = car.speed_mps();
			double check_s = car.s + check_speed_mps * target_time;
			if (check_s > target_s && check_s - target_s < 40) {
				return true;
			}
		}
	}
	return false;
}

bool checkBackwardCollide(double target_s, int target_lane, double target_time,
                          const std::vector<Car> &sensor) {
	for (auto &car : sensor) {
		if (target_lane == car.lane()) {
			auto check_s = car.s + car.speed_mps() * target_time;
			if (check_s <= target_s && check_s - target_s > -10) {
				return true;
			}
		}
	}
}

Car findForwardCar(int lane, double this_car_s,
                   const std::vector<Car> &sensor) {
	double f_s = 10000;
	Car f_car;
	f_car.id = -1;
	for (auto &car : sensor) {
		auto d = car.d;
		if (lane == car.lane()) {
			auto speed_mps = car.speed_mps();
			if (car.s > this_car_s && f_s > car.s) {
				f_s = car.s;
				f_car = car;
			}
		}
	}
	return f_car;
}

void createTrajectory(const Telemetry &tele, const Map &map, int lane, float v,
                      std::vector<double> &next_x_vals,
                      std::vector<double> &next_y_vals) {
	std::vector<double> ptsx;
	std::vector<double> ptsy;
	auto prev_size = tele.previous_path_x.size();
	if (prev_size < 2) {
		auto prev_x = tele.x - cos(tele.yaw);
		auto prev_y = tele.y - sin(tele.yaw);
		ptsx.push_back(prev_x);
		ptsy.push_back(prev_y);
		ptsx.push_back(tele.x);
		ptsy.push_back(tele.y);
	} else {
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
	for (auto w : wp) {
		ptsx.push_back(w[0]);
		ptsy.push_back(w[1]);
	}
	for (int i = 0; i < ptsx.size(); i++) {
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
	double N = (target_dist / (0.02 * v));

	for (int i = 1; i <= 50 - tele.previous_path_x.size(); i++) {

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