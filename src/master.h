#ifndef __MASTER_H__
#define __MASTER_H__

#include <uWS/uWS.h>
#include "json.hpp"
#include "helper_functions.h"
#include "particle_filter.h"
#include "config.h"

class Master
{
public:
	Master();	


	void run					 ();
private:

	std::string hasData			 (const std::string &s);
	void read_cfg				 (const std::string &cfg_path);
	
	
	uWS::Hub					 h;

	Map						     map;
	Config 						 cfg;
	ParticleFilter				 pf;

	std::fstream				 in;
	std::istringstream			 iss;
	std::string					 buff;

	double				 		delta_t;				// Time elapsed between measurements [sec]
	double				 		sensor_range;			// Sensor range [m]
	unsigned int 			 	particles_numb;

	std::vector<double>	 		sigma_pos;				// GPS measurement uncertainty [x [m], y [m], theta [rad]]
	std::vector<double>	 		sigma_landmark;			// Landmark measurement uncertainty [x [m], y [m]]

	unsigned int			 	port;
};


#endif // __MASTER_H__ 