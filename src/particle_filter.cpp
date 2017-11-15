/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // magic number alert!
  // the project template code expects num_particles to be set here, instead in a const value,
  // which I think is problematic, but that's the expectation.
  // This number is a balance between performance and accuracy.
  // 1000 particles is too slow with negligible accuracy improvement,
  // and 10 particles tends to have ~1.5 times the inaccuracy of 100
  this->num_particles = 100;

  // init gaussian random number generation
  std::default_random_engine random_engine;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  this->particles = std::vector<Particle>();
  this->weights = std::vector<double>();

  for (int index = 0; index < this->num_particles; index++) {
    double weight = 1.0;
    Particle particle = {
      index,                  // id
      dist_x(random_engine),
      dist_y(random_engine),
      dist_theta(random_engine),
      weight,
      std::vector<int>(),     // associations
      std::vector<double>(),  // sense_x
      std::vector<double>()   // sense_y
    };


    this->particles.push_back(particle);
    this->weights.push_back(weight);
  }

  this->is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // init gaussian random number generation
  std::default_random_engine random_engine;
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);
  
  double velocity_over_yaw_rate = 0;
  bool non_zero_yaw = false;
  if (fabs(yaw_rate) > 1e-7) {
    velocity_over_yaw_rate = velocity / yaw_rate;
    non_zero_yaw = true;
  }
  double delta_yaw = yaw_rate * delta_t;

  for (std::vector<Particle>::iterator i = this->particles.begin(); i != this->particles.end(); ++i) {
    Particle particle = *i;

    // predict assuming no sensor noise
    if (non_zero_yaw) {
      particle.x += velocity_over_yaw_rate * (sin(particle.theta + delta_yaw) - sin(particle.theta));
      particle.y += velocity_over_yaw_rate * (cos(particle.theta) - cos(particle.theta + delta_yaw));
      particle.theta += delta_yaw;
    } else {
      particle.x += velocity * delta_t * cos(particle.theta);
      particle.y += velocity * delta_t * sin(particle.theta);
    }

    // add gaussian noise to prediction to model sensor noise
    particle.x += dist_x(random_engine);
    particle.y += dist_y(random_engine);
    particle.theta += dist_theta(random_engine);

    *i = particle;
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  for (
      std::vector<LandmarkObs>::iterator observations_iterator = observations.begin();
      observations_iterator != observations.end();
      ++observations_iterator
      )
  {
    LandmarkObs observation = *observations_iterator;

    LandmarkObs matching_prediction = predicted[0];
    double minimum_distance = dist(matching_prediction.x, matching_prediction.y, observation.x, observation.y);
    for (
        std::vector<LandmarkObs>::iterator predicted_iterator = predicted.begin();
        predicted_iterator != predicted.end();
        ++predicted_iterator
        )
    {
      LandmarkObs prediction = *predicted_iterator;
      double observed_distance = dist(prediction.x, prediction.y, observation.x, observation.y);
      if (observed_distance <= minimum_distance) {
        minimum_distance = observed_distance;
        observations_iterator->id = prediction.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

  for (
      int particle_index = 0;
      particle_index < this->num_particles;
      particle_index++
      )
  {
    Particle particle = particles[particle_index];

    // convert observations to map coordinates
    std::vector<LandmarkObs> map_observations = std::vector<LandmarkObs>();
    for (
        std::vector<LandmarkObs>::const_iterator observations_iterator = observations.begin();
        observations_iterator != observations.end();
        ++observations_iterator
        ) {
      LandmarkObs observation = *observations_iterator;

      double x_map = particle.x + (cos(particle.theta) * observation.x) - (sin(particle.theta) * observation.y);
      double y_map = particle.y + (sin(particle.theta) * observation.x) + (cos(particle.theta) * observation.y);

      LandmarkObs map_coordinate_observation = {
        observation.id,
        x_map,
        y_map
      };

      map_observations.push_back(map_coordinate_observation);

    }

    // find the predicted observations for the particle in map coordinates
    std::vector<LandmarkObs> predicted_observations = std::vector<LandmarkObs>();
    for (
        std::vector<Map::single_landmark_s>::const_iterator map_iterator = map_landmarks.landmark_list.begin();
        map_iterator != map_landmarks.landmark_list.end();
        ++map_iterator
        ) {
    
      Map::single_landmark_s landmark = *map_iterator;

      // id_i, x_f, y_f
      double distance = dist(landmark.x_f, landmark.y_f, particle.x, particle.y);
      if (distance <= sensor_range) {
        LandmarkObs map_coordinate_observation = {
          landmark.id_i,
          landmark.x_f,
          landmark.y_f
        };

        predicted_observations.push_back(map_coordinate_observation);
      }

    }

    // associate observations with nearest predicted landmark
    this->dataAssociation(predicted_observations, map_observations);

    // find (un-normalized) probability of observation
    double new_weight = 1;
    for (
        std::vector<LandmarkObs>::iterator map_observations_iterator = map_observations.begin();
        map_observations_iterator != map_observations.end();
        ++map_observations_iterator
        )
    {
      LandmarkObs observation = *map_observations_iterator;
      double sigma_x = std_landmark[0];
      double sigma_y = std_landmark[1];
      double x = observation.x;
      double y = observation.y;
      
      LandmarkObs prediction;
      for (
          std::vector<LandmarkObs>::iterator predicted_observations_iterator = predicted_observations.begin();
          predicted_observations_iterator != predicted_observations.end();
          ++predicted_observations_iterator
          )
      {
        LandmarkObs p = *predicted_observations_iterator;
        if (p.id == observation.id) {
          prediction = p;
          break;
        }
      }

      double mu_x = prediction.x;
      double mu_y = prediction.y;

      double landmark_probability = multivariate_gaussian(sigma_x, sigma_y, x, y, mu_x, mu_y);
      new_weight *= landmark_probability;
    }
    weights[particle_index] = new_weight;
    particles[particle_index].weight = new_weight;
  }

  // normalize the weights to add to a valid probability distribution
  // this step is computationally somewhat expensive,
  // error prone (issues w/infinity and zero), and not required,
  // so I'm just not doing it
  /*
  double weight_sum = std::accumulate(weights.begin(), weights.end(), 0);
  //std::cout << weight_sum << std::endl;
  for (int i = 0; i < this->num_particles; ++i) {
    //weights[i] /= weight_sum;
    particles[i].weight = weights[i];
  }
  */
  
}

void ParticleFilter::resample() {
  std::default_random_engine random_engine;

  
  // resampling implementation using the recommended discrete_distribution
  std::discrete_distribution<> distribution(weights.begin(), weights.end());

  std::vector<Particle> resampled_particles = std::vector<Particle>();
  while (resampled_particles.size() < this->num_particles) {
    int index = distribution(random_engine);
    resampled_particles.push_back(particles[index]);
  }

  
  // use a "resampling wheel" to efficiently resample from the particles
  // based on their probability distribution
  // this code works great, but is no faster than the simpler implementation above
  /*
  std::uniform_real_distribution<> distribution(0, 1);
  double max_weight = *std::max_element(weights.begin(), weights.end());
  double doubled_max_weight = max_weight * 2;
  int wheel_index = std::floor(distribution(random_engine) * this->num_particles);

  double beta_value = 0;
  std::vector<Particle> resampled_particles = std::vector<Particle>();
  while (resampled_particles.size() < this->num_particles) {
    beta_value += distribution(random_engine) * doubled_max_weight;
    while (beta_value > weights[wheel_index]) {
      beta_value -= weights[wheel_index];
      wheel_index = (wheel_index + 1) % this->num_particles;
    }
    resampled_particles.push_back(particles[wheel_index]);
  }
  */

  this->particles = resampled_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations = associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
