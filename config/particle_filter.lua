map = "GDC1";
init_x = 14.7;
init_y = 14.24;
init_r = 0; -- in degrees

-- Overall particle filter parameters
num_particles = 100;
num_lasers = 100;
obs_update_skip_steps = 14;
obs_update_skip_dist = 0.05;

-- Initial distribution parameters
i1 = 0.04;
i2 = 0.04;

-- Motion model parameters
k1 = 0.5;
k2 = 0.04;
k3 = 0.4;
k4 = 0.04;
k5 = 0.01;

-- Observation model parameters
dshort = 0.3;
dlong = 1.0;
sigmas = 0.05;
