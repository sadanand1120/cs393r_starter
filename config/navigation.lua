NavigationParameters = {
    dt = 0.05;
    -- 0.05 is the default value for system latency on real robot
    system_latency = 0.0; 

    max_linear_accel = 5.0;
    max_linear_deccel = 5.0;
    max_linear_speed = 1.0;

    max_angular_accel = 3.0;
    max_angular_deccel = 3.0;
    max_angular_speed = 1.0;

    max_curvature = 1.0;
    max_path_length = 5.0;
    max_clearance = 1.0;

    clearance_weight = 10.0;
    arc_length_weight = 3.0;
    distance_weight = 5.0;

    goal_tolerance = 0.1;

    robot_length = 0.535;
    robot_width = 0.281;
    robot_wheelbase = 0.324;
    obstacle_margin = 0.05;
    lidar_offset = 0.21;
}
