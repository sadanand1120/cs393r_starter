map = "GDC1";
init_x = 14.7;
init_y = 14.24;
init_r = 0; -- in degrees

-- Overall particle filter parameters
num_particles = 50
num_lasers = 50
obs_update_skip_steps = 200
obs_update_skip_dist = 0.1

-- Initial distribution parameters
i1 = 0.2
i2 = 0.2

-- Motion model parameters
k1 = 0.2
k2 = 0.2
k3 = 0.1
k4 = 0.1
k5 = 0.1

-- Observation model parameters
dshort = 0.3
dlong = 0.05
sigmas = 0.05
