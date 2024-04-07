#include "navigation.h"

namespace RRT_Tree{
RRT_Tree::RRT_Tree(const Vector2f& root_odom_loc, const float root_odom_angle){
    //RRT_Node root = {.parent = NULL, .inbound_curvature = 0.0, .inboud_vel = 0.0, .odom_loc = root_odom_loc, .odom_angle = root_odom_angle};
    RRT_Node root = RRT_Node(NULL, 0.0, 0.0, root_odom_loc, root_odom_angle);

    std::vector<RRT_Node> tree = std::vector<RRT_Node>();

    tree.push_back(root);
}

RRT_Node RRT_Tree::find_closest(Vector2f sampled_config){
    double min_dist = -1.0;
    RRT_Node best_node = this->root;
    for(RRT_Node n : tree){
        Eigen::Vector2f dist_to_config = n.odom_loc - sampled_config;
        float dist = dist_to_config.norm();
        if(min_dist == -1.0 || dist < min_dist){
            min_dist = dist;
            best_node = n;
        }
    }

    return best_node;
}

std::vector<RRT_Node> RRT_Tree::make_trajectory(struct RRT_Node found_goal_config){
    std::vector<RRT_Node> trajectory = std::vector<RRT_Node>();

    RRT_Node current = found_goal_config;

    while(current != NULL){
        trajectory.push_back(current)

        current = current.parent
    }

    trajectory.reverse();

    return trajectory;
}

Vector2f RRT_Tree::sample_configs(double min_x, double min_y, double max_x, double max_y){
    double x = rng_.UniformRandom(min_x, max_x);
    double y = rng_.UniformRandom(min_y, max_y);

    return Vector2f(x, y);
}

bool RRT_Tree::collision_free(Vector2f n, Vector2f o, const vector_map::VectorMap map){
    // Just do straight line collision here with some margin

    // Steps
    // 1. Create set of points around new pose and closest pose based on margin
    // 2. Create line segments between corresponding points
    // 3. Check for intersection with the map

    double margin = 0.05;

    Vector2f n1 = Vector2f(n.x(), n.y() + margin);
    Vector2f n2 = Vector2f(n.x(), n.y() - margin);
    Vector2f n3 = Vector2f(n.x() + margin, n.y() + margin);
    Vector2f n4 = Vector2f(n.x() + margin, n.y() - margin);
    Vector2f n5 = Vector2f(n.x() - margin, n.y() + margin);
    Vector2f n6 = Vector2f(n.x() - margin, n.y() - margin);
    Vector2f n7 = Vector2f(n.x() + margin, n.y());
    Vector2f n8 = Vector2f(n.x() - margin, n.y());

    Vector2f o1 = Vector2f(o.x(), o.y() + margin);
    Vector2f o2 = Vector2f(o.x(), o.y() - margin);
    Vector2f o3 = Vector2f(o.x() + margin, o.y() + margin);
    Vector2f o4 = Vector2f(o.x() + margin, o.y() - margin);
    Vector2f o5 = Vector2f(o.x() - margin, o.y() + margin);
    Vector2f o6 = Vector2f(o.x() - margin, o.y() - margin);
    Vector2f o7 = Vector2f(o.x() + margin, o.y());
    Vector2f o8 = Vector2f(o.x() - margin, o.y());

    Line2f l0 = Line2f(o, n);
    Line2f l1 = Line2f(o1, n1);
    Line2f l2 = Line2f(o2, n2);
    Line2f l3 = Line2f(o3, n3);
    Line2f l4 = Line2f(o4, n4);
    Line2f l5 = Line2f(o5, n5);
    Line2f l6 = Line2f(o6, n6);
    Line2f l7 = Line2f(o7, n7);
    Line2f l8 = Line2f(o8, n8);

    for (auto line : map){
        if(line.Intersects(l0)) { return false; }
        if(line.Intersects(l1)) { return false; }
        if(line.Intersects(l2)) { return false; }
        if(line.Intersects(l3)) { return false; }
        if(line.Intersects(l4)) { return false; }
        if(line.Intersects(l5)) { return false; }
        if(line.Intersects(l6)) { return false; }
        if(line.Intersects(l7)) { return false; }
        if(line.Intersects(l8)) { return false; }
    }

    return true;
}

RRT_Node RRT_Tree::apply_rand_action(RRT_Node closest){
    // Steps here:
    // 1. Sample random action from [min_curv, max_curv],  [min_vel, max_vel], [0, max_time_step]
    // 2. Apply action over time step to current node
    // 3. Check if obstacle free

    // Sample new action
    double cur_min_curve = max(Action_Space.min_curv, closest.inbound_curvature - Action_Space.delta_curve);
    double cur_max_curve = min(Action_Space.max_curv, closest.inbound_curvature + Action_Space.delta_curve);
    double cur_min_vel = max(Action_Space.min_vel, closest.inboud_vel - Action_Space.delta_vel);
    double cur_max_vel = min(Action_Space.max_vel, closest.inbound_vel + Action_Space.delta_vel);

    double new_vel = rng._UniformRandom(cur_min_vel, cur_max_vel);
    double new_curve = rng._UniformRandom(cur_min_curve, cur_max_curve);
    double new_time = rng._UniformRandom(0, Action_Space.max_time_step);

    // Apply Action to closest
    Vector2f new_pose = Vector2f(0.0, 0.0);
    if (new_curve == 0) {
      // Only need to update x pose in base link frame
      new_pose.x() += new_vel * new_time;  // TODO: What is the actual duration here?
    } else {
      double cur_angular_change = new_curve * new_vel * new_time;  // TODO: What is the actual duration here?
      angular_change += cur_angular_change;                                  // TODO: What is the actual duration here?

      // Center of turning
      Eigen::Vector2f center_of_turning = Vector2f(0, 1 / new_curve);

      // New pose in base link frame
      Eigen::Rotation2Df r(cur_angular_change);
      Vector2f arc_trans_pose = r * (-1 * center_of_turning) + center_of_turning;

      // New pose in map frame
      Eigen::Rotation2Df r_adj(angular_change);
      Vector2f rotated_arc_trans_pose = r * arc_trans_pose;
      new_pose += rotated_arc_trans_pose;
    }

    Vector2f world_frame_new_pose = new_pose + closest.odom_loc;
    double world_frame_new_angle = angular_change + closest.odom_angle

    // Check collision
    if (collision_free(world_frame_new_pose, world_frame_new_angle, closest)){
        return {.parent = closest, .inbound_curvature = new_curve, .inboud_vel = new_vel, .odom_loc = world_frame_new_pose, . odom_angle = world_frame_new_angle};
    } else {
        return NULL;
    }
}

bool RRT_Tree::in_goal_config(Vector2f new_config, std::vector<Vector2f> goal_configs){
    if (new_config == NULL) { return false; }
    result = true;

    if(new_config[0] < goal_configs[0][0] || new_config[0] > goal_configs[1][0]) { result = false; }
    if(new_config[1] < goal_configs[0][1] || new_config[1] > goal_configs[1][1]) { result = false; }

    return result;
}

std::vector<Vector2f> RRT_Tree::plan_trajectory(const Vector2f& odom_loc, const float odom_angle, std::vector<Vector2f> goal_configs, const vector_map::VectorMap map) {
    // Calculate a trajectory for the robot using RRT

    // Steps:
    // 1. Create tree with current location as the root
    // 2. Repeat below until goal config reached
    //      1. Sample locations in the map
    //      2. Find the nearest node in the tree
    //      3. Select action to drive towards node (can be randomly sampled here it turns out)
    //      4. If config produced by new action is valid (collision free) add it to the tree

    // Get max/min x/y from map
    double min_x = -1.0;
    double min_y = -1.0;
    double max_x = -1.0;
    double max_y = -1.0;
    for (const auto& map_line : map_.lines) {
        double line_x1 = map_line.p0[0]
        double line_x2 = map_line.p1[0]
        
        double line_y1 = map_line.p0[1]
        double line_y2 = map_line.p1[1]

        if(min_x == -1.0 || line_x1 < min_x) {min_x = line_x1;}
        if(min_x == -1.0 || line_x2 < min_x) {min_x = line_x2;}
        if(max_x == -1.0 || line_x1 > max_x) {max_x = line_x1;}
        if(max_x == -1.0 || line_x2 > max_x) {max_x = line_x2;}

        if(min_y == -1.0 || line_y1 < min_y) {min_y = line_y1;}
        if(min_y == -1.0 || line_y2 < min_y) {min_y = line_y2;}
        if(max_y == -1.0 || line_y1 > max_y) {max_y = line_y1;}
        if(max_y == -1.0 || line_y2 > max_y) {max_y = line_y2;}
    }

    // Initialize tree
    RRT_Tree rrt_tree = RRT_Tree(odom_loc, odom_angle);

    // Sample new configs
    Vector2f sampled_config = sample_configs(min_x, min_y, max_x, max_y);

    // Get closest rrt node
    RRT_Node closest = rrt_tree.find_closest(sample_config);

    // Apply random action from closest
    // If obstacle return NULL
    RRT_Node new_config = apply_rand_action(closest);

    // If not null add to tree 
    if (new_config != NULL){
        rrt_tree.tree.push_back(new_config)
    }

    // Repeat until goald found
    while (!in_goal_config(new_config, goal_configs)){
        // Sample new configs
        Vector2f sampled_config = sample_configs(min_x, min_y, max_x, max_y);

        // Get closest rrt node
        RRT_Node closest = rrt_tree.find_closest(sample_config);

        // Apply random action from closest
        // If obstacle return NULL
        RRT_Node new_config = apply_rand_action(closest);

        // If not null add to tree 
        if (new_config != NULL){
            rrt_tree.tree.push_back(new_config)
        }
    }

    return rrt_tree.make_trajectory(new_config)
}
}