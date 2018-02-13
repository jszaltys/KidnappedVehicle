#include "particle_filter.h"

void ParticleFilter::init(const unsigned int &particles_numb, const double &x, const double &y,const double &theta, const std::vector<double>& std) 
{
	num_particles = particles_numb;
	weights.resize(num_particles);
	particles.resize(num_particles);

	gen = std::mt19937(rd());

	std::normal_distribution<double> dist_x(x,		   std[0]);
	std::normal_distribution<double> dist_y(y,		   std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	for (unsigned int i = 0; i < num_particles; ++i) 
	{
		particles[i].id		= i;
		particles[i].x		= dist_x(gen);
		particles[i].y		= dist_y(gen);
		particles[i].theta	= dist_theta(gen);
		particles[i].weight = 1.0;	
		weights[i] 			= 1.0;
	}
	is_initialized = true;
}
void ParticleFilter::prediction(const double & delta_t, const std::vector<double>&std_pos, const double & velocity, const double & yaw_rate) 
{
	double x = 0.0, y = 0.0, theta = 0.0;

	for (unsigned int i = 0; i < num_particles; ++i) 
	{
		if (std::fabs(yaw_rate) > 0.001) 
		{
			x 		= particles[i].x 	  + (velocity / yaw_rate) * (std::sin(particles[i].theta  + yaw_rate * delta_t) - std::sin(particles[i].theta));
			y 		= particles[i].y 	  + (velocity / yaw_rate) * (std::cos(particles[i].theta) - std::cos(particles[i].theta + yaw_rate * delta_t));
			theta 	= particles[i].theta  + yaw_rate  * delta_t;
		}
		else 
		{
			x	   = particles[i].x		  + velocity * delta_t * std::cos(particles[i].theta);
			y	   = particles[i].y		  + velocity * delta_t * std::sin(particles[i].theta);
			theta  = particles[i].theta   + yaw_rate * delta_t;
		}

		std::normal_distribution<double> dist_x(x,	   std_pos[0]);
		std::normal_distribution<double> dist_y(y,	   std_pos[1]);
		std::normal_distribution<double> dist_theta(theta, std_pos[2]);
		
		particles[i].x		   = dist_x(gen);
		particles[i].y	       = dist_y(gen);
		particles[i].theta     = dist_theta(gen);
	}
}
void ParticleFilter::updateWeights(const double &sensor_range, const std::vector<double> &std_landmark, const std::vector<LandmarkObs> &observations,const Map &map_landmarks)
{
	double prob 	= 0.0;
	double min_dist = 0.0;
	int id_min 		= 0;
	
	for (unsigned int i = 0; i < num_particles; ++i)
	{
		std::vector<LandmarkObs> transform_obs(observations.size());
		std::vector<LandmarkObs> closest_land;
			
		for (unsigned int j = 0; j < observations.size(); ++j)
		{
			const double trans_obs_x = observations[j].x * std::cos(particles[i].theta) - observations[j].y * std::sin(particles[i].theta) + particles[i].x;
			const double trans_obs_y = observations[j].x * std::sin(particles[i].theta) + observations[j].y * std::cos(particles[i].theta) + particles[i].y;

			transform_obs[j] = LandmarkObs(trans_obs_x,trans_obs_y,-1);		
		}
	
		for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j) 
		{
			const double landmark_part_dist = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
			if (landmark_part_dist < sensor_range) 	
			{
				LandmarkObs pred_landmark;
				pred_landmark.id = map_landmarks.landmark_list[j].id_i;
				pred_landmark.x = map_landmarks.landmark_list[j].x_f;
				pred_landmark.y = map_landmarks.landmark_list[j].y_f;

				closest_land.push_back(pred_landmark);
			}						
		}
		
		dataAssociation(closest_land, transform_obs);
		prob = 1.0;
		for (unsigned int j = 0; j < closest_land.size(); ++j)
		{
			id_min 		= -1;
			min_dist 	= 99999;

			for (unsigned int k = 0; k < transform_obs.size(); ++k) 
			{	
				const double m_dist = dist(closest_land[j].x, closest_land[j].y, transform_obs[k].x, transform_obs[k].y);

				if (m_dist < min_dist)
				{
					min_dist = m_dist;
					id_min = k;
				}
			}
			if (id_min != -1)
				prob *= bivariate_normal(closest_land[j].x, closest_land[j].y, transform_obs[id_min].x, transform_obs[id_min].y, std_landmark[0], std_landmark[1]);
		}

		weights[i] 			= prob;
		particles[i].weight = prob;	
	}
}
void ParticleFilter::resample() 
{
	std::random_device 				rd_wts;
	std::mt19937 					generator_wts(rd_wts());
	
	std::vector<Particle>			new_particles(num_particles);
	std::discrete_distribution<int> index(weights.begin(), weights.end());
	

	for (unsigned int i = 0; i < num_particles; ++i)
		new_particles[i] = particles[index(generator_wts)];

	particles = new_particles;
}
Particle ParticleFilter::get_best_particle()
{
	double highest_weight = -1.0;
	Particle best_particle;

	for (unsigned int i = 0; i < num_particles; ++i)
	{
		if (particles[i].weight > highest_weight)
		{
			highest_weight = particles[i].weight;
			best_particle = particles[i];
		}
	}
	return best_particle;
}
bool ParticleFilter::initialized() const
{
	return is_initialized;
}
void ParticleFilter::dataAssociation(const std::vector<LandmarkObs> &predicted, std::vector<LandmarkObs>& observations) 
{
	double 		 m_dist 			= 0.0;
	int 		 closest_landmark 	= 0;
	double 		 min_dist  			= 0.0;
	for (unsigned int i = 0; i < observations.size(); ++i)
	{
		min_dist 		 = 99999;
		closest_landmark = -1;
		for (unsigned int j = 0; j < predicted.size(); ++j) 
		{
			m_dist = dist(observations[i].x,observations[i].y, predicted[j].x, predicted[j].y);
			if (m_dist < min_dist)
			{
				min_dist = m_dist;
				closest_landmark = predicted[j].id;
			}
		}
		observations[i].id = closest_landmark;
	}
}
std::string ParticleFilter::getAssociations(const Particle &best)
{
	std::stringstream ss;
    std::copy(best.associations.begin(), best.associations.end(), std::ostream_iterator<int>(ss, " "));
	std:: string s = ss.str();
    s = s.substr(0, s.length()-1);  
    return s;
}
Particle ParticleFilter::SetAssociations(Particle &particle,const std::vector<int> &associations,const std::vector<double> &sense_x,const std::vector<double> &sense_y)
{
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations	= associations;
 	particle.sense_x	 	= sense_x;
 	particle.sense_y 		= sense_y;

 	return particle;
}
std::string ParticleFilter::getSenseX(const Particle &best)
{
	std::stringstream ss;
    std::copy( best.sense_x.begin(), best.sense_x.end(),  std::ostream_iterator<double>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  
    return s;
}
std::string ParticleFilter::getSenseY(const Particle &best)
{
	std::stringstream ss;
    std::copy( best.sense_y.begin(), best.sense_y.end(),  std::ostream_iterator<double>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  
    return s;
}
Particle & Particle::operator=(const Particle & particle)
{
	this->associations	= particle.associations;
	this->sense_x		= particle.sense_x;
	this->sense_y		= particle.sense_y;
	this->id			= particle.id;
	this->x				= particle.x;
	this->y				= particle.y;
	this->theta			= particle.theta;
	this->weight		= particle.weight;

	return *this;
}
Particle::Particle(const Particle & particle)
{
	this->associations 	= particle.associations;
	this->sense_x 		= particle.sense_x;
	this->sense_y 		= particle.sense_y;
	this->id 			= particle.id;
	this->x 			= particle.x;
	this->y 			= particle.y;
	this->theta 		= particle.theta;
	this->weight 		= particle.weight;
}