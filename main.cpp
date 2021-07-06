#include <iostream>
#include <algorithm>
#include <vector>

#include "help_functions.h"

using namespace std;

/// initialize priors assuming vehicle at landmark +/- 1.0 meters at position stdev
std::vector<float> initialize_priors(int map_size, std::vector<float> landmark_positions, float position_stdev);

float motion_model(float pseudo_position, float movement, std::vector<float> priors, int map_size, int control_stdev);

std::vector<float> pseudo_range_estimator(std::vector<float> landmark_positions, float pseudo_position);

float observation_model(std::vector<float> landmark_positions, std::vector<float> observations, std::vector<float> pseudo_ranges, float distance_max, float observation_stdev);

int main()
{
    /// set standard derivation of control
    float control_stdev = 1.0f;

    /// meters vehicle moves per time step
    float movement_per_timestep = 1.0f;

    /// set standard derivation of position
    float position_stdev = 1.0f;

    /// set map horizon distance in meters
    int map_size = 25;

    /// initialize landmarks
    std::vector<float> landmark_positions {5, 10, 20};

    /// define observations
    std::vector<float> observations {5.5, 13, 15};

    /// define max distance
    float distance_max = map_size;

    /// set standard derivation of observation
    float observation_stdev = 1.0f;

    /// initialize priors
    std::vector<float> priors = initialize_priors(map_size, landmark_positions, position_stdev);

    /*
    for (unsigned int i = 0; i < map_size; i++) {
        float pseudo_position = float(i);

        /// get the motion model probability for each x position
        float motion_prob = motion_model(pseudo_position, movement_per_timestep, priors, map_size, control_stdev);

        std::cout << pseudo_position << "\t" << motion_prob << endl;
    }
    */

    for (unsigned int i = 0; i < map_size; i++) {
        float pseudo_position = float(i);

        std::vector<float> pseudo_ranges = pseudo_range_estimator(landmark_positions, pseudo_position);

        float observations_prob = observation_model(landmark_positions, observations, pseudo_ranges, distance_max, observation_stdev);

        /*
        if (pseudo_ranges.size() > 0) {
            for (unsigned int s = 0; s < pseudo_ranges.size(); s++) {
                std::cout<<"x: "<<i<<"\t"<<pseudo_ranges[s]<<endl;
            }
        }
        */
        std::cout<<observations_prob<<endl;
    }

    return 0;
}

float observation_model(std::vector<float> landmark_positions, std::vector<float> observations, std::vector<float> pseudo_ranges, float distance_max, float observation_stdev) {

    float distance_prob = 1.0f;

    for (unsigned int z = 0; z < observations.size(); z++) {
        float pseudo_range_min;

        if(pseudo_ranges.size() > 0) {
            pseudo_range_min = pseudo_ranges[0];

            pseudo_ranges.erase(pseudo_ranges.begin());
        }
        else {
            pseudo_range_min = std::numeric_limits<const float>::infinity();
        }

        /// estimate the probability for observation model, this is our likelihood
        distance_prob *= Helpers::normpdf(observations[z], pseudo_range_min, observation_stdev);
    }

    return distance_prob;

}

std::vector<float> pseudo_range_estimator(std::vector<float> landmark_positions, float pseudo_position) {

    std::vector<float> pseudo_ranges;

    for (unsigned int i = 0; i < landmark_positions.size(); i++) {
        float pseudo_range = landmark_positions[i] - pseudo_position;

        if (pseudo_position > 0.0f) {
            pseudo_ranges.push_back(pseudo_range);
        }
    }

    /// sort pseudo range vector
    sort(pseudo_ranges.begin(), pseudo_ranges.end());

    return pseudo_ranges;
}

std::vector<float> initialize_priors(int map_size, std::vector<float> landmark_positions, float position_stdev) {

    /// set all priors to 0.0
    std::vector<float> priors(map_size, 0.0);

    /// set each landmark position +/1 to 1.0/9.0 (9 possible positions)
    float normaliztion_term = landmark_positions.size() * (position_stdev * 2 + 1);

    for (unsigned int i = 0; i < landmark_positions.size(); i++) {
        int landmark_center = landmark_positions[i];
        priors[landmark_center] = 1.0f/normaliztion_term;
        priors[landmark_center - position_stdev] = 1.0f/normaliztion_term;
        priors[landmark_center + position_stdev] = 1.0f/normaliztion_term;
    }

    return priors;
}

float motion_model(float pseudo_position, float movement, std::vector<float> priors, int map_size, int control_stdev) {

    /// initialize probability
    float position_prob = 0.0f;

    for (unsigned int j = 0; j < map_size; j++) {
        float next_pseudo_position = float(j);

        float distance_ij = pseudo_position - next_pseudo_position;

        /// get the motion model probability for each x position
        float transition_prob = Helpers::normpdf(distance_ij, movement, control_stdev);

        /// estimate probability for the motion model, this is out prior
        position_prob += transition_prob * priors[j];
    }

    return position_prob;
}
