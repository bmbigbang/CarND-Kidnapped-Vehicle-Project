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

	// assuming a 101 by 101 grid size for the map, running from -10*std to 10*std in each dimension .
	// initializing particles with evenly spread particles within the grid and same orientation
	num_particles = 101 * 101;

	weights.assign(num_particles, 1.0);

	for (int i = 0; i <= 100; i++) {
        for (int j = 0; j <= 100; j++) {
            Particle p;
            p.id = (1000 * (i + 1)) + (j + 1);
            p.x = x * (((i - 50) * 2.0) / 10) * std[0];
            p.y = y * (((j - 50) * 2.0) / 10) * std[1];
            p.theta = theta;
            p.weight = 1.0;
            particles.push_back(p);
        }
    }
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
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
