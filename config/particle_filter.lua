map = "GDC1";
init_x = 14.7;
init_y = 14.24;
init_r = 0; -- in degrees

-- Overall particle filter parameters
num_particles = 50
num_lasers = 100
obs_update_skip_steps = 0

-- Initial distribution parameters
i1 = 0.1
i2 = 0.1

-- Motion model parameters
k1 = 0.1
k2 = 0.1
k3 = 0.1
k4 = 0.1
k5 = 0.1

-- Observation model parameters
dshort = 0.3
dlong = 1.0
sigmas = 0.05