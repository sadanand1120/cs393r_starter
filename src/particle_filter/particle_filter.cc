//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using Eigen::Vector2f;
using Eigen::Vector2i;
using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

// DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

Eigen::Vector2f avg_loc(0,0);
Eigen::Vector2f last_seen_obs_loc(0,0);

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter()
    : prev_odom_loc_(0, 0),
      prev_odom_angle_(0),
      odom_initialized_(false),
      k1(0),
      k2(0),
      k3(0),
      k4(0),
      k5(0),
      num_particles(50),
      num_lasers(100),
      i1(0),
      i2(0),
      dshort(0),
      dlong(0),
      sigmas(0),
      obs_update_skip_steps(0),
      obs_update_skip_dist(0),
      step_counter_(0) {}

void ParticleFilter::SetHparams(float _k1, float _k2, float _k3, float _k4, float _k5, int _num_particles,
                                int _num_lasers, float _i1, float _i2, float _dshort, float _dlong, float _sigmas,
                                int _obs_update_skip_steps, float _obs_update_skip_dist) {
  k1 = _k1;
  k2 = _k2;
  k3 = _k3;
  k4 = _k4;
  k5 = _k5;
  num_particles = _num_particles;
  num_lasers = _num_lasers;
  i1 = _i1;
  i2 = _i2;
  dshort = _dshort;
  dlong = _dlong;
  sigmas = _sigmas;
  obs_update_skip_steps = _obs_update_skip_steps;
  obs_update_skip_dist = _obs_update_skip_dist;
}

void ParticleFilter::GetParticles(vector<Particle>* particles) const { *particles = particles_; }

void ParticleFilter::GetPredictedPointCloud(const Eigen::Vector2f& loc, const float angle, int num_ranges,
                                            float range_min, float range_max, float angle_min, float angle_max,
                                            float angle_increment, std::vector<Eigen::Vector2f>* scan_ptr) {
  std::vector<Eigen::Vector2f>& scan = *scan_ptr;
  scan.clear();
  scan.resize(num_ranges);

  Eigen::Vector2f laser_offset(0.21, 0);                                        // Laser offset in the base_link frame
  Eigen::Vector2f laser_loc = loc + Eigen::Rotation2Df(-angle) * laser_offset;  // Transform to laser location

  // Calculate how many lasers to skip based on num_lasers
  int lasers_to_skip = std::max(1, num_ranges / num_lasers);  // Ensure we don't divide by zero or skip none

  float current_angle = angle_max;
  for (int i = 0; i < num_ranges; i += lasers_to_skip) {
    current_angle = angle_max - i * angle_increment;

    // Check if the current angle is beyond the minimum limit
    if (current_angle < angle_min) {
      break;
    }

    Eigen::Vector2f ray_dir(std::cos(angle + current_angle), std::sin(angle + current_angle));
    Eigen::Vector2f max_point = laser_loc + ray_dir * range_max;  // End point of the laser ray at max range

    bool found_intersection = false;
    float min_distance = range_max * 2;  // Initialize with a value beyond the maximum range for no intersection

    // Iterate over all line segments in the map to find the closest intersection
    for (const auto& map_line : map_.lines) {
      Eigen::Vector2f intersection_point;
      if (map_line.Intersection(line2f(laser_loc, max_point), &intersection_point)) {
        float distance = (intersection_point - laser_loc).norm();
        if (distance >= range_min && distance <= range_max) {
          min_distance = std::min(min_distance, distance);
          found_intersection = true;
        }
      }
    }

    if (!found_intersection) {
      // Set to a default value if no intersection is found
      min_distance = 10 * range_max;
    } else if (min_distance > range_max) {
      // Truncate the distance to within the [range_min, range_max] range if it's outside
      min_distance = range_max;
    } else if (min_distance < range_min) {
      // Truncate the distance to within the [range_min, range_max] range if it's outside
      min_distance = range_min;
    }

    if (!found_intersection) {
      Eigen::Vector2f scan_point = loc;
      scan[i / lasers_to_skip] = scan_point;  // Store the scan point at the adjusted index based on skipped lasers
    } else {
      Eigen::Vector2f scan_point = laser_loc + ray_dir * min_distance;
      scan[i / lasers_to_skip] = scan_point;  // Store the scan point at the adjusted index based on skipped lasers
    }
  }
}

double ParticleFilter::ComputeLogLikelihood(double s, double pred_s, double range_min, double range_max, double dshort,
                                            double dlong, double sigmas) {
  // Check for the corner case first
  if ((s < range_min || s > range_max) && (pred_s < range_min || pred_s > range_max)) {
    return 0;
  }

  double delta = 0.0;  // This will hold the value used in the computation

  if (s < range_min || s > range_max) {
    // delta = std::pow(range_max, 2);
    delta = 0.0;
  } else if (s < pred_s - dshort) {
    delta = std::pow(dshort, 2);
  } else if (s > pred_s + dlong) {
    delta = std::pow(dlong, 2);
  } else {
    delta = std::pow(s - pred_s, 2);
  }

  double logLikelihood = -delta / std::pow(sigmas, 2);
  return logLikelihood;
}

void ParticleFilter::Update(const std::vector<float>& ranges, float range_min, float range_max, float angle_min,
                            float angle_max, float angle_increment, Particle* p_ptr) {
  // Predict the expected observations for this particle
  std::vector<Eigen::Vector2f> predictedPointCloud;
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, ranges.size(), range_min, range_max, angle_min, angle_max,
                         angle_increment, &predictedPointCloud);

  // Calculate the number of lasers to skip
  int lasers_to_skip = std::max(1, static_cast<int>(ranges.size()) / num_lasers);

  double logLikelihoodSum = 0.0;
  Eigen::Vector2f laser_offset(0.21, 0);  // Laser offset in the base_link frame

  printf("--------------------------------------------------------------------------------------\n");
  for (unsigned int i = 0; i < ranges.size(); i += lasers_to_skip) {
    float s = ranges[i];  // Observed range
    // Convert predicted point from laser frame back to range
    Eigen::Vector2f predictedPoint = predictedPointCloud[i / lasers_to_skip];
    Eigen::Vector2f laser_loc =
        p_ptr->loc + Eigen::Rotation2Df(p_ptr->angle) * laser_offset;  // Transform to laser location
    float pred_s = (predictedPoint - laser_loc).norm();

    // Compute log likelihood of observed range given the predicted range
    double logLikelihood = ComputeLogLikelihood(s, pred_s, range_min, range_max, dshort, dlong, sigmas);
    // printf("Log Likelihood: %f\n", logLikelihood);
    logLikelihoodSum += logLikelihood;
    // print i and loglikelihood sum
    printf("Laser Index: %d\n", i);
    printf("Log Likelihood Sum: %f\n", logLikelihoodSum);
  }
  printf("--------------------------------------------------------------------------------------\n");

  // Update the particle's log weight
  p_ptr->logweight = 0.2 * p_ptr->logweight + 0.8 * logLikelihoodSum;
}

void ParticleFilter::Resample() {
  std::vector<Particle> new_particles;

  // // Find the max logweight
  // double max_logweight = particles_[0].logweight;
  // for (const Particle& p : particles_) {
  //   if (p.logweight > max_logweight) {
  //     max_logweight = p.logweight;
  //   }
  // }
  // Find min logweight
  double max_logweight = particles_[0].logweight;
  for (const Particle& p : particles_) {
    if (p.logweight < max_logweight) {
      max_logweight = p.logweight;
    }
  }

  // Normalize logweights by subtracting max_logweight and convert to linear scale
  std::vector<double> weights;
  double sum_weights = 0.0;
  for (const Particle& p : particles_) {
    double weight = p.logweight - max_logweight;  // Convert to linear scale
    weights.push_back(weight);
    sum_weights += weight;
  }

  // Renormalize weights to sum to 1
  for (double& weight : weights) {
    weight /= sum_weights;
  }

  // // print weights array
  // printf("--------------------------------------------------------------------------------------\n");
  // for (size_t i = 0; i < weights.size(); i++) {
  //   printf("Weight: %f\n", weights[i]);
  // }
  // printf("--------------------------------------------------------------------------------------\n");

  // Low Variance Resampling
  int N = particles_.size();
  double r = rng_.UniformRandom(0, 1.0 / N);
  double c = weights[0];
  double increment = 1.0 / N;
  int i = 0;
  for (int m = 0; m < N; ++m) {
    double U = r + m * increment;
    while (U > c) {
      i = i + 1;
      c = c + weights[i];
    }
    new_particles.push_back(particles_[i]);
  }

  particles_ = new_particles;  // Replace the old particles with the new resampled particles
}

void ParticleFilter::ObserveLaser(const std::vector<float>& ranges, float range_min, float range_max, float angle_min,
                                  float angle_max, float angle_increment) {
  // Check if we should skip this update
  float dist = (avg_loc - last_seen_obs_loc).norm();
  if (dist < obs_update_skip_dist) {
    // Increment step counter and skip this observation
    return;
  }

  // Reset the step counter since we're processing this observation
  step_counter_ = 0;
  last_seen_obs_loc = avg_loc;

  // Perform the update step for each particle based on the new laser observation
  for (Particle& particle : particles_) {
    Update(ranges, range_min, range_max, angle_min, angle_max, angle_increment, &particle);
    // print particle weight
    // printf("Particle Weight: %f\n", particle.logweight);
  }

  // Resample the particles based on their updated weights
  Resample();
}

void ParticleFilter::Predict(const Vector2f& odom_loc, const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  // We can only start prediction once we have obtained at least 2 odometry
  // readings.

  if (!odom_initialized_) {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
    return;
  }

  // If we get here, then odometry has been initialized
  Eigen::Rotation2Df r(-prev_odom_angle_);
  Vector2f delta_T = r * (odom_loc - prev_odom_loc_);
  float delta_theta = odom_angle - prev_odom_angle_;

  // Motion Model:
  // Here do thresholding instead for a straightline
  Vector2f p(0.0, -1.0);
  Vector2f n(1.0, 0.0);
  float travel = 0.0;
  float c = 0.0;  // curvature
  if (delta_T.y() == 0.00) {
    travel = delta_T.norm();
    c = 0.0;
  } else {
    // Radius of turning; the resulting sign of R matters.
    float R = 0.5 * (math_util::Sq(delta_T.x()) + math_util::Sq(delta_T.y())) / delta_T.y();
    // Radial vector
    p = delta_T + Vector2f(0, -R);
    p.normalize();
    // Tangent vector
    n(-p.y(), p.x());
    // Distance traveled, note that acos returns a value betwee 0 and PI
    // Multiply by the radius to get distance traveled
    travel = std::acos(-1.0 * math_util::Sign(R) * p.y()) * std::fabs(R);
    c = 1 / R;  // curvature
  }

  // For the predict step, for every particle under consideration, we sample
  // that particle's motion with respect to the motion model.
  // Particles are in the map frame
  Vector2f noisy_T(0, 0);
  for (Particle& part : particles_) {
    // Create a noisy version of delta_T
    // Error for travel
    float ep_n = rng_.Gaussian(0.0, k1 * travel + k2 * std::fabs(delta_theta));
    // Error for radius and slip
    float ep_p = rng_.Gaussian(0.0, k3 * c + k4 * std::fabs(delta_theta));
    float ep_theta = rng_.Gaussian(0.0, k5 * std::fabs(delta_theta));
    // printf("Tangential Error: %f\n", ep_n);
    // printf("Radial Error: %f\n", ep_p);
    // printf("Angle Error: %f\n", ep_theta);

    noisy_T = delta_T + ep_n * n + ep_p * p;

    part.loc = part.loc + Eigen::Rotation2Df(part.angle) * noisy_T;
    part.angle = part.angle + delta_theta + ep_theta;
  }

  // Now, update the odometry info for next time.
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file, const Vector2f& loc, const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  // Reset the particles vector (clear pointers)
  particles_.clear();
  particles_.reserve(num_particles);

  // Initialize particles around the provided location and angle
  for (int i = 0; i < num_particles; ++i) {
    Particle p;
    // Assuming loc and angle are means of the distributions
    p.loc = loc + Eigen::Vector2f(rng_.Gaussian(0, std::sqrt(i1)), rng_.Gaussian(0, std::sqrt(i1)));
    p.angle = angle + rng_.Gaussian(0, std::sqrt(i2));
    p.logweight = log(1.0 / num_particles);  // Initially, all particles have the same weight

    particles_.push_back(p);
  }

  // Initialize odometry
  odom_initialized_ = false;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, float* angle_ptr) const {
  Eigen::Vector2f loc(0, 0);
  float angle = 0;
  int total_particles = particles_.size();

  // Sum up all particle locations and angles
  for (const Particle& p : particles_) {
    loc += p.loc;
    angle += p.angle;
  }

  // Compute the average by dividing by the total number of particles
  if (total_particles > 0) {
    loc /= total_particles;
    angle /= total_particles;
  }

  avg_loc = loc;

  // Assign the computed values to output parameters
  *loc_ptr = loc;
  *angle_ptr = angle;
}

// void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, float* angle_ptr) const {
//   Eigen::Vector2f loc(0, 0);
//   float angle = 0;
//   double total_weight = 0;

//   // Find min logweight
//   double max_logweight = particles_[0].logweight;
//   for (const Particle& p : particles_) {
//     if (p.logweight < max_logweight) {
//       max_logweight = p.logweight;
//     }
//   }

//   // Compute weighted sum of particle locations and angles
//   for (const Particle& p : particles_) {
//     float weight = p.logweight - max_logweight;
//     loc += p.loc * weight;
//     // For angle, we need to ensure correct averaging of circular quantities
//     angle += Eigen::Rotation2Df(p.angle).angle() * weight;
//     total_weight += weight;
//   }

//   // Normalize to get the weighted average
//   if (total_weight > 0) {
//     loc /= total_weight;
//     angle /= total_weight;
//   }

//   // Assign the computed values to output parameters
//   *loc_ptr = loc;
//   *angle_ptr = angle;
// }

}  // namespace particle_filter
