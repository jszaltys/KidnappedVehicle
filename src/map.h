#ifndef __MAP_H__
#define __MAP_H__

struct Map 
{
	struct single_landmark_s
	{
		single_landmark_s() : id_i(0), x_f(0.0), y_f(0.0) {}
	
		unsigned int id_i ; // Landmark ID
		double x_f;			// Landmark x-position in the map (global coordinates)
		double y_f;			// Landmark y-position in the map (global coordinates)
	};

	std::vector<single_landmark_s> landmark_list ; // List of landmarks in the map
};

#endif /* __MAP_H__ */
