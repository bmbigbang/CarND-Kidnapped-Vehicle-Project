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

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// assuming a 51 by 51 grid size for the map, running from -5*std to 5*std in each dimension .
	// initializing particles with evenly spread particles within the grid and same orientation
	num_particles = 51 * 51;

	weights.assign(num_particles, 1.0);

	for (int i = 0; i <= 50; i++) {
        for (int j = 0; j <= 50; j++) {
            Particle p;
            p.id = (1000 * (i + 1)) + (j + 1);
            p.x = x * (((i - 25) * 2.0) / 5) * std[0];
            p.y = y * (((j - 25) * 2.0) / 5) * std[1];
            p.theta = theta;
            p.weight = 1.0;
            particles.push_back(p);
        }
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    default_random_engine gen;

    normal_distribution<double> N_x_pred(0, std_pos[0]);
    normal_distribution<double> N_y_pred(0, std_pos[1]);
    normal_distribution<double> N_theta_pred(0, std_pos[2]);

    for (int i = 0; i < num_particles; i++) {

        // define values for prediction calculation
        Particle p = particles[i];
        double x0 = p.x;
        double y0 = p.y;
        double yaw0 = p.theta;
        double yaw1 = p.theta + (yaw_rate * delta_t);

        // add gaussian noise from normal distrubtions
        double sample_x, sample_y, sample_theta;
        sample_x = N_x_pred(gen);
        sample_y = N_y_pred(gen);
        sample_theta = N_theta_pred(gen);

        // update particle values
        p.x = sample_x + x0 + ((velocity / yaw_rate) * (sin(yaw1) - sin(yaw0)));
        p.y = sample_y + y0 + ((velocity / yaw_rate) * (cos(yaw0) - cos(yaw1)));
        p.theta = sample_theta + yaw1;

	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.


    for (int i = 0; i < observations.size(); ++i) {
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html


    // define car's mapped coordinates u, k
    LandmarkObs t_obs;
    for (int i = 0; i < num_particles; i++) {
        // define values for prediction calculation
        Particle p = particles[i];
        double min_x = 500.0;
        double min_y = 500.0;
        double prob = 1.0;

        for (int j = 0; j < observations.size(); j++) {
            LandmarkObs obs = observations[j];
            t_obs.x = p.x - (obs.y * sin(p.theta - (M_PI / 2))) + (obs.x * cos(p.theta - (M_PI / 2)));
            t_obs.y = p.y + (obs.y * cos(p.theta - (M_PI / 2))) + (obs.x * sin(p.theta - (M_PI / 2)));

            // find nearest neighbour to observation
            for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
                Map::single_landmark_s landmark = map_landmarks.landmark_list[k];
                if (fabs(t_obs.x - landmark.x_f) < min_x && (
                         (t_obs.x <= 0 && landmark.x_f <= 0) || (t_obs.x > 0 && landmark.x_f > 0)) &&
                    fabs(t_obs.y - landmark.y_f) < min_y && (
                         (t_obs.y <= 0 && landmark.y_f <= 0) || (t_obs.y > 0 && landmark.y_f > 0))) {
                    min_x = t_obs.x - landmark.x_f;
                    min_y = t_obs.y - landmark.y_f;
                };
            }

            // multiply to prob multivariate gaussian holder

            prob *= exp(-1 * ((pow(min_x, 2) / (2 * pow(std_landmark[0], 2))) +
                               (pow(min_y, 2) / (2 * pow(std_landmark[1], 2))))
                    ) / (M_PI * 2 * std_landmark[0] * std_landmark[1]);

        }
        weights[i] = prob;
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // find the highest weight
    double max_w = 0.0;
    for (int i = 0; i < weights.size(); i++) {
        if (max_w < weights[i]) {
            max_w = weights[i];
        }
    }

    int r = (double)rand() / (double)((unsigned)RAND_MAX) * num_particles;

    std::vector<Particle> resampled_particles;
    std::vector<double> resampled_weights;
    for (int i = 0; i < num_particles; i++) {
        double better = (double)rand() / (double)((unsigned)RAND_MAX) * 2 * max_w;

        while (weights[r] < better) {
            better -= weights[r];
            r = (r + 1) % num_particles;
        }
        resampled_particles.push_back(particles[r]);
        resampled_weights.push_back(weights[r]);
    }
    particles = resampled_particles;
    weights = resampled_weights;

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
