#ifndef __PARTICLE_FILTER_H__
#define __PARTICLE_FILTER_H__

#include "libs.h"
#include "helper_functions.h"

struct Particle 
{
	Particle() : id(0), x(0.0), y(0.0), theta(0.0), weight(0.0) {}
	Particle& operator=(const Particle& particle);
	Particle(const Particle& particle);

	int							id;
	double						x;
	double						y;
	double						theta;
	double						weight;

	std::vector<int>			associations;
	std::vector<double>			sense_x;
	std::vector<double>			sense_y;
};


class ParticleFilter
{
public:
	// Constructor
	// @param M Number of particles
	ParticleFilter() : num_particles(0), is_initialized(false) {}

	// Destructor
	~ParticleFilter() {}
	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
	void init(const unsigned int &particles_numb,const double &x, const double &y, const double &theta, const std::vector<double>& std);
	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction(const double & delta_t, const std::vector<double>&std_pos, const double & velocity, const double & yaw_rate);
	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
	 *   standard deviation of bearing [rad]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */
	void updateWeights(const double &sensor_range,const std::vector<double> &std_landmark, const std::vector<LandmarkObs> &observations,const Map &map_landmarks);
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample();
	/**
	 * dataAssociation Finds which observations correspond to which landmarks (likely by using
	 *   a nearest-neighbors data association).
	 * @param predicted Vector of predicted landmark observations
	 * @param observations Vector of landmark observations
	 */
	void dataAssociation(const std::vector<LandmarkObs> &predicted, std::vector<LandmarkObs>& observations);
	/*
	 * Set a particles list of associations, along with the associations calculated world x,y coordinates
	 * This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected
	 */
	Particle SetAssociations(Particle &particle,const std::vector<int>&associations,const std::vector<double> &sense_x,const std::vector<double> &sense_y);
	/**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
	bool initialized() const;
	
	Particle 	get_best_particle();
	
	std::string getAssociations	(const Particle &best);
	std::string getSenseX		(const Particle &best);
	std::string getSenseY		(const Particle &best);

	
	// Set of current particles
	std::vector<Particle>	particles;
private:
	// Number of particles to draw
	unsigned int			num_particles;

	// Flag, if filter is initialized
	bool					is_initialized;

	// Vector of weights of all particles
	std::vector<double>		weights;
	
	std::random_device		rd;
	std::mt19937	 		gen;
	
	typedef std::vector<Map::single_landmark_s> Landmark_list;
};
#endif /* PARTICLE_FILTER_H_ */
