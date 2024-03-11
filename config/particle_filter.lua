map = "GDC1";
init_x = 14.7;
init_y = 14.24;
init_r = 0; -- in degrees

-- Overall particle filter parameters
num_particles = 50
num_lasers = 10
obs_update_skip_steps = 200

-- Initial distribution parameters
i1 = 0.1
i2 = 0.1

-- Motion model parameters
k1 = 0.0
k2 = 0.0
k3 = 0.0
k4 = 0.0
k5 = 0.0

-- Observation model parameters
dshort = 0.8
dlong = 2.0
sigmas = 0.5