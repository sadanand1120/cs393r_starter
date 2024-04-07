#include <cuda.h>
#include <cuda_runtime.h>

#include "particle_filter.h"

__global__ void UpdateKernel(int n,
                             int num_ranges,
                             const float* d_ranges,
                             float range_min,
                             float range_max,
                             float angle_min,
                             float angle_max,
                             particle_filter::Particle* d_particles);