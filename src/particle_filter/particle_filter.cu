#include "config_reader/config_reader.h"
#include "particle_filter.cuh"


namespace particle_filter {

CONFIG_FLOAT(num_particles_, "num_particles");
CONFIG_FLOAT(k_x_, "k_x");
CONFIG_FLOAT(k_y_, "k_y");
CONFIG_FLOAT(k_theta_, "k_theta");
CONFIG_FLOAT(k_laser_loc_x_, "k_laser_loc.x");
CONFIG_FLOAT(k_laser_loc_y_, "k_laser_loc.y");
CONFIG_FLOAT(d_short, "d_short");
CONFIG_FLOAT(d_long, "d_long");
CONFIG_FLOAT(likelihood_sigma, "likelihood_sigma");
CONFIG_FLOAT(likelihood_gamma, "likelihood_gamma");
CONFIG_FLOAT(dist_threshold, "dist_threshold");
config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

__constant__ float D_CONFIG_num_particles_ = CONFIG_num_particles_;
__constant__ float D_CONFIG_k_x_ = CONFIG_k_x_;
__constant__ float D_CONFIG_k_y_ = CONFIG_k_y_;
__constant__ float D_CONFIG_k_theta_ = CONFIG_k_theta_;
__constant__ float D_CONFIG_k_laser_loc_x_ = CONFIG_k_laser_loc_x_;
__constant__ float D_CONFIG_k_laser_loc_y_ = CONFIG_k_laser_loc_y_;
__constant__ float D_CONFIG_d_short = CONFIG_d_short;
__constant__ float D_CONFIG_d_long = CONFIG_d_long;
__constant__ float D_CONFIG_likelihood_sigma = CONFIG_likelihood_sigma;
__constant__ float D_CONFIG_likelihood_gamma = CONFIG_likelihood_gamma;
__constant__ float D_CONFIG_dist_threshold = CONFIG_dist_threshold;

// /** BEGIN CUDA IMPLEMENTATIONS **/

__global__ void UpdateKernel(int n,
                             int num_ranges,
                             const float* d_ranges,
                             float range_min,
                             float range_max,
                             float angle_min,
                             float angle_max,
                             particle_filter::Particle* d_particles) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < n; i += stride) {
    Eigen::Vector2f scan[num_ranges];

    // 1 Get predicted scan for each particle
    GetPredictedPointCloudCUDA(d_particles[i].loc,
                               d_particles[i].angle,
                               num_ranges,
                               range_min,
                               range_max,
                               angle_min,
                               angle_max,
                               scan);

    // 2 Convert each particle's scan to the particle laser frame

    // 3 Compute the likelihood of each particle
    //  d_particles[i].log_likelihood = computeNormalLikelihoodCUDA(
    //    scan, d_ranges, num_ranges, range_min, range_max
    //  );
  }
}

__device__ void GetPredictedPointCloudCUDA(const Eigen::Vector2f& loc,
                                           const float angle,
                                           int num_ranges,
                                           float range_min,
                                           float range_max,
                                           float angle_min,
                                           float angle_max,
                                           Eigen::Vector2f* scan) {
  const Eigen::Vector2f kLaserLoc(D_CONFIG_k_laser_loc_x_, D_CONFIG_k_laser_loc_y_);

  for (int i = 0; i < num_ranges; ++i) {
    float theta = angle_min + i * (angle_max - angle_min) / num_ranges;

    // scan[i] = make_float2(range_max * cos(theta), range_max * sin(theta)) +
    //           make_float2(kLaserLocX, kLaserLocY);
  }
}

/** END CUDA IMPLEMENTATIONS **/
}  // namespace particle_filter