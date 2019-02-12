#ifndef UTIL_H
#define UTIL_H
#include "json.hpp"
#include <array>
#include <cmath>
#include <vector>

constexpr inline double pi() { return M_PI; }
constexpr inline double deg2rad(double x) { return x * pi() / 180; }
constexpr inline double rad2deg(double x) { return x * 180 / pi(); }
constexpr inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
constexpr inline std::array<double, 2> rotate(double x, double y, double theta)
{
	return {
		x * cos(theta) - y * sin(theta),
		x * sin(theta) + y * cos(theta),
	};
}

namespace io
{

std::string hasData(std::string s);
}

class Map
{
	std::vector<double> maps_x;
	std::vector<double> maps_y;
	std::vector<double> maps_s;
	std::vector<double> maps_dx;
	std::vector<double> maps_dy;
	double max_s = 6945.554;

  public:
	Map(const std::string &filename);
	int getClosestWaypoint(double x, double y) const;
	int getNextWaypoint(double x, double y, double theta) const;
	std::array<double, 2> getFrenet(double x, double y, double theta) const;
	std::array<double, 2> getXY(double s, double d) const;
};

struct Car
{
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
	inline std::string dump() const
	{
		std::stringstream ss;
		ss << id << ": " << x << ", " << y << ", " << vx << ", " << vy << ", " << s << ", " << d;
		return ss.str();
	}
};

#endif