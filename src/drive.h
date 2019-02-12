#ifndef DRIVE_H
#define DRIVE_H
#include "json.hpp"
#include "spline.h"
#include "util.h"
#include <iostream>
#include <vector>

namespace drive {
struct Telemetry {
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

std::string createMessage(const std::vector<double> &next_x,
                          const std::vector<double> &next_y);
bool checkForwardCollide(double target_s, int target_lane, double target_time,
                         const std::vector<Car> &sensor);

bool checkBackwardCollide(double target_s, int target_lane, double target_time,
                          const std::vector<Car> &sensor);

bool checkBackwardCollide(double target_s, int target_lane, double target_time,
                          const std::vector<Car> &sensor);

Car findForwardCar(int lane, double this_car_s, const std::vector<Car> &sensor);

void createTrajectory(const Telemetry &tele, const Map &map, int lane, float v,
                      std::vector<double> &next_x_vals,
                      std::vector<double> &next_y_vals);
} // namespace drive

#endif