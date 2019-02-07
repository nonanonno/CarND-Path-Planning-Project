#ifndef DRIVE_H
#define DRIVE_H
#include "json.hpp"
#include "util.h"

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
} // namespace drive

#endif