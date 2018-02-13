#include "master.h"

Master::Master():delta_t(0.0),sensor_range(0.0),particles_numb(0),port(0){}

std::string Master::hasData(const std::string& s)
{
	auto found_null = s.find("null");
	auto b1			= s.find_first_of("[");
	auto b2			= s.find_first_of("]");

	if (found_null != std::string::npos)
		return "";

	else if (b1 != std::string::npos && b2 != std::string::npos)
		return s.substr(b1, b2 - b1 + 1);

	return "";
}
void Master::read_cfg(const std::string &cfg_path)
{
	cfg.read_cfg(cfg_path);
	
	for (auto &r : cfg.mstringmap)
	{	
		if (r.first == "TIMESTEP")
			delta_t = String2Float()(r.second);

		else if (r.first == "SENSOR_RANGE")
			sensor_range = String2Int()(r.second);

		else if (r.first == "PARTICLES_NUMBER")
			particles_numb = String2Int()(r.second);

		else if (r.first == "PORT")
			port = String2Int()(r.second);

		else if (r.first == "GPS_STD")
			sigma_pos = String2Array()(r.second);

		else if (r.first == "LANDMARK_STD")
			sigma_landmark= String2Array()(r.second);
	}
}
void Master::run()
{
	if (!read_map_data("../data/map_data.txt", map)) 
		std::cout << "Error: Could not open map file" << std::endl;
	
	read_cfg("../data/cfg.txt");
	
	std::cout<<"TimeStep        = "<<delta_t<<std::endl;
	std::cout<<"Sensor Range    = "<<sensor_range<<std::endl;
	std::cout<<"Particles Number= "<<particles_numb<<std::endl;
	std::cout<<"Port            = "<<port<<std::endl;
	std::cout<<"GPS Unct        = "<<sigma_pos<<std::endl;
	std::cout<<"Landmark Unct   = "<<sigma_landmark<<std::endl;
	
	
	h.onMessage			([this](uWS::WebSocket<uWS::SERVER> ws, char *message, size_t length, uWS::OpCode opCode)
	{
		if (length && length > 2 && message[0] == '4' && message[1] == '2')
		{	
			auto s = hasData(std::string(message));	
			if (s != "")
			{
				auto j = nlohmann::json::parse(s);
				if (j[0].get<std::string>() == "telemetry")
				{
					
					if (!pf.initialized())
					{
						const double sense_x	  = std::stod(j[1]["sense_x"].    get<std::string>());
						const double sense_y	  = std::stod(j[1]["sense_y"].    get<std::string>());
						const double sense_theta  = std::stod(j[1]["sense_theta"].get<std::string>());

						pf.init(particles_numb,sense_x, sense_y, sense_theta, sigma_pos);
					
					}
					else 
					{
						const double prv_velocity = std::stod(j[1]["previous_velocity"].get<std::string>());
						const double prv_yawrate  = std::stod(j[1]["previous_yawrate"]. get<std::string>());

						pf.prediction(delta_t, sigma_pos, prv_velocity, prv_yawrate);
					}

					std::string				 sense_observations_x = j[1]["sense_observations_x"];
					std::string				 sense_observations_y = j[1]["sense_observations_y"];

					std::vector<float>		 x_sense, y_sense;
					std::istringstream		 iss_x(sense_observations_x), iss_y(sense_observations_y);

					std::copy(std::istream_iterator<float>(iss_x), std::istream_iterator<float>(), std::back_inserter(x_sense));
					std::copy(std::istream_iterator<float>(iss_y), std::istream_iterator<float>(), std::back_inserter(y_sense));

					std::vector<LandmarkObs> noisy_observations(x_sense.size());
					
					for (unsigned int i = 0; i < x_sense.size(); ++i)
						noisy_observations[i] = LandmarkObs(x_sense[i], y_sense[i], 0);

				
					pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
					pf.resample();

					Particle best_particle(pf.get_best_particle());
					
					nlohmann::json msgJson;
					msgJson["best_particle_x"]			  = best_particle.x;
					msgJson["best_particle_y"]			  = best_particle.y;
					msgJson["best_particle_theta"]		  = best_particle.theta;
					msgJson["best_particle_associations"] = pf.getAssociations(best_particle);
					msgJson["best_particle_sense_x"]	  = pf.getSenseX(best_particle);
					msgJson["best_particle_sense_y"]	  = pf.getSenseY(best_particle);

					auto msg = "42[\"best_particle\"," + msgJson.dump() + "]";
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
						
				}
			}
			else
			{
				const std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});
	h.onHttpRequest		([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
	{
		const std::string s = "<h1>Hello world!</h1>";

		if (req.getUrl().valueLength == 1)
			res->end(s.data(), s.length());

		else
			res->end(nullptr, 0);
	});
	h.onConnection		([this](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
	{
		std::cout << "Connected!!!" << std::endl;
	});
	h.onDisconnection	([this](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) 
	{
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
	}
	h.run();
}

