#include "drive.h"
#include <cstdint>
#include <iostream>

namespace drive
{

Telemetry::Telemetry(const nlohmann::json &obj) : x(obj["x"]), y(obj["y"]), s(obj["s"]), d(obj["d"]), yaw(obj["yaw"]), speed(obj["speed"]), previous_path_x(obj["previous_path_x"].size()), previous_path_y(obj["previous_path_y"].size()), end_path_s(obj["end_path_s"]), end_path_d(obj["end_path_d"]), sensor_fusion(obj["sensor_fusion"].size())
{
	for (size_t i = 0; i < previous_path_x.size(); i++)
	{
		previous_path_x[i] = obj["previous_path_x"][i];
		previous_path_y[i] = obj["previous_path_y"][i];
	}
	for (size_t i = 0; i < sensor_fusion.size(); i++)
	{
		auto item = obj["sensor_fusion"][i];
		sensor_fusion[i] = {
			item[0],
			item[1],
			item[2],
			item[3],
			item[4],
			item[5],
			item[6],
		};
	}
}

} // namespace drive