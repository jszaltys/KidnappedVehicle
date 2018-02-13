#ifndef __HELPER_FUNCTIONS_H__
#define __HELPER_FUNCTIONS_H__

#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include "map.h"

static const double PI = 3.1415926535897932384626433832795;

/*
 * Struct representing one position/control measurement.
 */
struct control_s 
{
	double velocity;	// Velocity [m/s]
	double yawrate;		// Yaw rate [rad/s]
};
/*
 * Struct representing one ground truth position.
 */
struct ground_truth 
{
	double x;		// Global vehicle x position [m]
	double y;		// Global vehicle y position
	double theta;	// Global vehicle yaw [rad]
};
/*
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs 
{
	LandmarkObs(){}
	LandmarkObs(const double& _x, const double& _y, const int &_id) :x(_x), y(_y), id(_id) {}

	int id;			    // Id of matching landmark in the map.
	double x;			// Local (vehicle coordinates) x position of landmark observation [m]
	double y;			// Local (vehicle coordinates) y position of landmark observation [m]
};
/*
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(const double &x1, const double &y1, const double &x2,const  double &y2)
{
	return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline std::vector<double> getError(const double &gt_x, const double &gt_y, const double &gt_theta, const double &pf_x, const double &pf_y, const double &pf_theta)
{
	std::vector<double> error = { std::fabs(pf_x - gt_x) ,std::fabs(pf_y - gt_y) ,std::fmod(std::fabs(pf_theta - gt_theta),2.0* PI) };
	if (error[2] > PI)
		error[2] = 2.0 * PI - error[2];
	return error;
}
/* Reads map data from a file.
 * @param filename Name of file containing map data.
 * @output True if opening and reading file was successful
 */
inline bool read_map_data(const std::string &filename, Map& map) 
{
	// Get file of map:
	std::ifstream in_file_map(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_map) 
		return false;

	// Declare single line of map file:
	std::string line_map;

	// Run over each single line:
	while(getline(in_file_map, line_map))
	{
		std::istringstream iss_map(line_map);

		// Declare single_landmark:
		Map::single_landmark_s single_landmark_temp;
		
		// Set values
		iss_map >> single_landmark_temp.x_f;
		iss_map >> single_landmark_temp.y_f;
		iss_map >> single_landmark_temp.id_i;

		
		// Add to landmark list of map:
		map.landmark_list.push_back(single_landmark_temp);
	}
	return true;
}

/* Reads control data from a file.
 * @param filename Name of file containing control measurements.
 * @output True if opening and reading file was successful
 */
inline bool read_control_data(const std::string &filename, std::vector<control_s>& position_meas) 
{
	// Get file of position measurements:
	std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_pos) 
		return false;

	// Declare single line of position measurement file:
	std::string line_pos;

	// Run over each single line:
	while(getline(in_file_pos, line_pos))
	{
		std::istringstream iss_pos(line_pos);

		// Declare single control measurement:
		control_s meas;

		// Set values
		iss_pos >> meas.velocity;
		iss_pos >> meas.yawrate;

		// Add to list of control measurements:
		position_meas.push_back(meas);
	}
	return true;
}
/* Reads ground truth data from a file.
 * @param filename Name of file containing ground truth.
 * @output True if opening and reading file was successful
 */
inline bool read_gt_data(const std::string &filename, std::vector<ground_truth>& gt)
{
	// Get file of position measurements:
	std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_pos) 
		return false;

	// Declare single line of position measurement file:
	std::string line_pos;

	// Run over each single line:
	while(getline(in_file_pos, line_pos))
	{
		std::istringstream iss_pos(line_pos);

		// Declare single ground truth:
		ground_truth single_gt; 

		// Set values
		iss_pos >> single_gt.x;
		iss_pos >> single_gt.y;
		iss_pos >> single_gt.theta;

		// Add to list of control measurements and ground truth:
		gt.push_back(single_gt);
	}
	return true;
}
/* Reads landmark observation data from a file.
 * @param filename Name of file containing landmark observation measurements.
 * @output True if opening and reading file was successful
 */
inline bool read_landmark_data(const std::string &filename, std::vector<LandmarkObs>& observations) 
{
	// Get file of landmark measurements:
	std::ifstream in_file_obs(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_obs) 
		return false;

	// Declare single line of landmark measurement file:
	std::string line_obs;

	// Run over each single line:
	while(getline(in_file_obs, line_obs))
	{
		std::istringstream iss_obs(line_obs);

		// Declare single landmark measurement:
		LandmarkObs meas;

		// Set values
		iss_obs >> meas.x;
		iss_obs >> meas.y;

		// Add to list of control measurements:
		observations.push_back(meas);
	}
	return true;
}
inline double bivariate_normal(const double &x, const double & y, const double & mu_x, const double & mu_y, const double & sig_x, const double & sig_y) 
{
	return std::exp(-((x-mu_x)*(x-mu_x)/(2.0*sig_x*sig_x) + (y-mu_y)*(y-mu_y)/(2.0*sig_y*sig_y))) / (2.0*PI*sig_x*sig_y);
}
struct String2Int
{
	int operator()(const std::string &str) const
	{
		return std::stoi(str, std::string::size_type());
	}
};
struct String2Float
{
	double operator()(const std::string &str) const
	{
		return std::stod(str, std::string::size_type());
	}
};
struct String2Array
{
	std::vector<double> operator()(const std::string&str)
	{
		std::vector<double> tmp;
		bool separate_flag = false;

		std::string tmp_str(str);
		std::string accumulator = "";
		std::string::iterator it;
		std::replace(tmp_str.begin(), tmp_str.end(), ',', ' ');

		for (it = tmp_str.begin(); it < tmp_str.end(); ++it)
		{
			if (*it == ' ')
			{
				tmp.push_back(String2Float()(accumulator));
				separate_flag = true;
				accumulator.clear();
			}

			if (separate_flag != false)
				accumulator += *it;
			else
				accumulator += *it;
		}
		tmp.push_back(String2Float()(accumulator));

		return tmp;
	}
};
template<typename Type>
std::ostream& operator<<(std::ostream& os, const std::vector<Type>& vec)
{
	std::string tmp = "[";
	for (unsigned int i = 0; i < vec.size(); ++i)
	{
		if (i % 10 == 0 && i != 0)
			tmp += "\n ";
		if (i == vec.size() - 1)
			tmp += std::to_string(vec[i]) + "]";
		else
			tmp += std::to_string(vec[i]) + ", ";
	}

	return os << tmp;
}

#endif /* __HELPER_FUNCTIONS_H__ */
