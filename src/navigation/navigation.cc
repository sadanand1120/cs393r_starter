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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "navigation.h"

#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "motion_primitives.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "visualization/visualization.h"

DEFINE_bool(simulation, true, "Run in simulation mode");
DEFINE_bool(Test1DTOC, false, "Run 1D line time-optimal controller test");
DEFINE_bool(TestSamplePaths, true, "Run sample paths test");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
}  // namespace

namespace rrt_tree {
RRT_Tree::RRT_Tree(const Vector2f& root_odom_loc, const float root_odom_angle) {
  root.parent = NULL;
  root.inbound_curvature = 0.0;
  root.inbound_vel = 0.0;
  root.odom_loc = root_odom_loc;
  root.odom_angle = root_odom_angle;

  tree = std::vector<RRT_Node*>();

  tree.push_back(&root);

  printf("Address of Root is: %p\n", &root);
}

RRT_Node* RRT_Tree::find_closest(Vector2f sampled_config) {
  //printf("Executing Find Closest:\n");
  double min_dist = -1.0;
  RRT_Node* best_node = &root;
  //printf("Best Node initially set to %p\n", best_node);
  for (RRT_Node* n : tree) {
    //printf("loop Node in tree address is %p\n", &n);
    Eigen::Vector2f dist_to_config = n->odom_loc - sampled_config;
    float dist = dist_to_config.norm();
    if (min_dist == -1.0 || dist < min_dist) {
      min_dist = dist;
      best_node = n;
      //printf("Best Node Set to address: %p\n", best_node);
    }
  }

  return best_node;
}

std::list<RRT_Node*> RRT_Tree::make_trajectory(RRT_Node* found_goal_config) {
  std::list<RRT_Node*> trajectory = std::list<RRT_Node*>();

  //printf("Inside Make Trajectory\n");
  printf("Total Nodes in Tree: %ld\n", tree.size());
  //for (RRT_Node* r : tree) {
    //printf("Node Address: %p\n", r);
    //printf("Node Parent: %p\n", r->parent);
    //printf("Node Curvature: %f\n", r->inbound_curvature);
    //printf("Node Velocity: %f\n", r->inbound_vel);
    //printf("Node Odom Location: %f %f\n", r->odom_loc.x(), r->odom_loc.y());
    //printf("Node Odom Angle: %f\n", r->odom_angle);
  //}

  RRT_Node* current = found_goal_config;
  int total_pushed = 0;
  while (current->parent != NULL) {
    trajectory.push_front(current);
    total_pushed += 1;
    //printf("Total Pushed: %d\n", total_pushed);

    current = current->parent;
  }

  trajectory.push_front(current);

  //std::reverse(trajectory.begin(), trajectory.end());

  return trajectory;
}

Vector2f RRT_Tree::sample_configs(double min_x, double min_y, double max_x, double max_y) {
  double x = rng_.UniformRandom(min_x, max_x);
  double y = rng_.UniformRandom(min_y, max_y);

  return Vector2f(x, y);
}

bool RRT_Tree::collision_free(Vector2f n, Vector2f o, const vector_map::VectorMap map) {
  // Just do straight line collision here with some margin

  // Steps
  // 1. Create set of points around new pose and closest pose based on margin
  // 2. Create line segments between corresponding points
  // 3. Check for intersection with the map

  /*double margin = 0.05;

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
  Vector2f o8 = Vector2f(o.x() - margin, o.y());*/

  line2f l0 = line2f(o, n);/*
  line2f l1 = line2f(o1, n1);
  line2f l2 = line2f(o2, n2);
  line2f l3 = line2f(o3, n3);
  line2f l4 = line2f(o4, n4);
  line2f l5 = line2f(o5, n5);
  line2f l6 = line2f(o6, n6);
  line2f l7 = line2f(o7, n7);
  line2f l8 = line2f(o8, n8);*/

  for (size_t j = 0; j < map.lines.size(); ++j) {
    const line2f line = map.lines[j];
    if (line.Intersects(l0)) {
      return false;
    }/*
    if (line.Intersects(l1)) {
      return false;
    }
    if (line.Intersects(l2)) {
      return false;
    }
    if (line.Intersects(l3)) {
      return false;
    }
    if (line.Intersects(l4)) {
      return false;
    }
    if (line.Intersects(l5)) {
      return false;
    }
    if (line.Intersects(l6)) {
      return false;
    }
    if (line.Intersects(l7)) {
      return false;
    }
    if (line.Intersects(l8)) {
      return false;
    }*/
  }

  return true;
}

RRT_Node* RRT_Tree::apply_rand_action(RRT_Node* closest, const vector_map::VectorMap map) {
  // Steps here:
  // 1. Sample random action from [min_curv, max_curv],  [min_vel, max_vel], [0, max_time_step]
  // 2. Apply action over time step to current node
  // 3. Check if obstacle free

  // Sample new action
  navigation::Action_Space aspace;
  //double cur_min_curve = max(aspace.min_curve, closest->inbound_curvature - aspace.delta_curve);
  //double cur_max_curve = min(aspace.max_curve, closest->inbound_curvature + aspace.delta_curve);
  //double cur_min_vel = max(aspace.min_vel, closest->inbound_vel - aspace.delta_vel);
  //double cur_max_vel = min(aspace.max_vel, closest->inbound_vel + aspace.delta_vel);

  double new_vel = rng_.UniformRandom(aspace.min_vel, aspace.max_vel);
  double new_curve = rng_.UniformRandom(aspace.min_curve, aspace.max_curve);
  double new_time = aspace.max_time_step;

  // Apply Action to closest
  Vector2f new_pose = Vector2f(0.0, 0.0);
  double cur_angular_change = 0.0;
  if (new_curve == 0) {
    // Only need to update x pose in base link frame
    new_pose.x() += new_vel * new_time;  // TODO: What is the actual duration here?
  } else {
    cur_angular_change = new_curve * new_vel * new_time;  // TODO: What is the actual duration here?

    // Center of turning
    Eigen::Vector2f center_of_turning = Vector2f(0, 1 / new_curve);

    // New pose in base link frame
    Eigen::Rotation2Df r(cur_angular_change);
    new_pose = r * (-1 * center_of_turning) + center_of_turning;
  }

  Eigen::Rotation2Df r(closest->odom_angle);

  new_pose = r * new_pose;
  Vector2f world_frame_new_pose = new_pose + closest->odom_loc;
  double world_frame_new_angle =
      cur_angular_change + closest->odom_angle;

  // Check collision
  if (collision_free(world_frame_new_pose, closest->odom_loc, map)) {
        RRT_Node* new_node = new RRT_Node;
        new_node->parent = closest;
        new_node->inbound_curvature = new_curve;
        new_node->inbound_vel = new_vel;
        new_node->odom_loc = world_frame_new_pose;
        new_node->odom_angle = world_frame_new_angle;

        return new_node;
  }
  else {
    RRT_Node* new_node = new RRT_Node;
    new_node->parent = closest;
    new_node->inbound_curvature = new_curve;
    new_node->inbound_vel = new_vel;
    new_node->odom_loc = world_frame_new_pose;
    new_node->odom_angle = world_frame_new_angle;
    new_node->broken = true;
    return new_node;
  }
}

bool RRT_Tree::in_goal_config(Vector2f new_config, Vector2f goal, double goal_radius) {
  if ((new_config - goal).norm() < goal_radius) {
    return true;
  }

  return false;
}

std::list<RRT_Node*> RRT_Tree::plan_trajectory(const Vector2f& odom_loc, const float odom_angle, Vector2f goal, double goal_radius, const vector_map::VectorMap map) {
  // Calculate a trajectory for the robot using RRT

  // Steps:
  // 1. Create tree with current location as the root
  // 2. Repeat below until goal config reached
  //      1. Sample locations in the map
  //      2. Find the nearest node in the tree
  //      3. Select action to drive towards node (can be randomly sampled here it turns out)
  //      4. If config produced by new action is valid (collision free) add it to the tree

  printf("Planning a trajectory...\n");

  // Get max/min x/y from map
  double min_x = -1.0;
  double min_y = -1.0;
  double max_x = -1.0;
  double max_y = -1.0;
  for (const auto& map_line : map.lines) {
    double line_x1 = map_line.p0[0];
    double line_x2 = map_line.p1[0];

    double line_y1 = map_line.p0[1];
    double line_y2 = map_line.p1[1];

    if (min_x == -1.0 || line_x1 < min_x) {
      min_x = line_x1;
    }
    if (min_x == -1.0 || line_x2 < min_x) {
      min_x = line_x2;
    }
    if (max_x == -1.0 || line_x1 > max_x) {
      max_x = line_x1;
    }
    if (max_x == -1.0 || line_x2 > max_x) {
      max_x = line_x2;
    }

    if (min_y == -1.0 || line_y1 < min_y) {
      min_y = line_y1;
    }
    if (min_y == -1.0 || line_y2 < min_y) {
      min_y = line_y2;
    }
    if (max_y == -1.0 || line_y1 > max_y) {
      max_y = line_y1;
    }
    if (max_y == -1.0 || line_y2 > max_y) {
      max_y = line_y2;
    }
  }

  // Initialize tree
  printf("Initializing a tree\n");
  RRT_Tree rrt_tree = RRT_Tree(odom_loc, odom_angle);
  printf("rrt_tree.root address is %p\n", &(rrt_tree.root));
  printf("rrt_tree.tree[0] address is %p\n", rrt_tree.tree[0]);

  // Sample new configs
  printf("Sampling new configs\n");
  Vector2f sampled_config = sample_configs(min_x, min_y, max_x, max_y);
  visualization::DrawCross(sampled_config, 0.2, 0x020202, global_viz_msg_);

  // Get closest rrt node
  printf("Finding closest rrt node\n");
  RRT_Node* closest = rrt_tree.find_closest(sampled_config);
  visualization::DrawCross(closest->odom_loc, 0.2, 0x0000FF, global_viz_msg_);

  // Apply random action from closest
  // If obstacle return NULL
  printf("Sampling a random action from closest\n");
  RRT_Node* new_config = apply_rand_action(closest, map);
    visualization::DrawCross(new_config->odom_loc, 0.2, 0x00FF00, global_viz_msg_);

  // If not null add to tree
  if (!new_config->broken) {
    rrt_tree.tree.push_back(new_config);
  }

  // Repeat until goald found
  printf("Trying to reach the goal\n");

  double min_dist_to_goal = 100.0;
  double max_dist_from_root = 0.0;
  int max_samples = 1000;
  int total_samples = 0;

  // Repeat until goal found
  while (!in_goal_config(new_config->odom_loc, goal, goal_radius) && total_samples < max_samples) {
    // Sample new configs 
    //With some probability epsilon, sample the goal
    Vector2f sampled_config;
    if (rng_.UniformRandom(0, 1) < 0.25){
        sampled_config = sample_configs(goal.x()-goal_radius, goal.y() - goal_radius, goal.x() + goal_radius, goal.y() + goal_radius);
    } else {
        sampled_config = sample_configs(min_x, min_y, max_x, max_y);
    }
    visualization::DrawCross(sampled_config, 0.2, 0x020202, global_viz_msg_);

    // Get closest rrt node
    RRT_Node* closest = rrt_tree.find_closest(sampled_config);
    //cout << "Closest Loc: " << closest.odom_loc << endl;
    visualization::DrawCross(closest->odom_loc, 0.2, 0x0000FF, global_viz_msg_);

    // Apply random action from closest
    // If obstacle return NULL
    new_config = apply_rand_action(closest, map);
    visualization::DrawCross(new_config->odom_loc, 0.2, 0x00FF00, global_viz_msg_);

    // If not null add to tree
    if (!new_config->broken) {
      rrt_tree.tree.push_back(new_config);

      if ((new_config->odom_loc - goal).norm() < min_dist_to_goal){
        min_dist_to_goal = (new_config->odom_loc - goal).norm();
      }

      if ((new_config->odom_loc - root.odom_loc).norm() > max_dist_from_root){
        max_dist_from_root = (new_config->odom_loc - root.odom_loc).norm();
      }

      cout << "Added Config: " << endl << new_config->odom_loc << endl << "========" << endl;
      cout << "Min distance to goal: " << min_dist_to_goal << endl;
      cout << "Max distance from root: " << max_dist_from_root << endl; 
    }
    total_samples += 1;
  }

/*
  while (!in_goal_config(new_config->odom_loc, goal_configs) && total_samples < max_samples) {
    // Sample new configs
    Vector2f sampled_config = sample_configs(min_x, min_y, max_x, max_y);

    // Get closest rrt node
    RRT_Node* closest = rrt_tree.find_closest(sampled_config);
    visualization::DrawCross(closest->odom_loc, 1, 32762, local_viz_msg_);

    // Apply random action from closest
    // If obstacle return NULL
    RRT_Node* new_config = apply_rand_action(closest, map);
    visualization::DrawCross(new_config->odom_loc, 1, 3272, local_viz_msg_);

    // If not null add to tree
    if (!new_config->broken) {
      rrt_tree.tree.push_back(new_config);
    }
    total_samples += 1;
  }*/
  printf("Finished Looping\n");

  return rrt_tree.make_trajectory(new_config);
}
}  // namespace rrt_tree














namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, NavigationParams& params, ros::NodeHandle* n)
    : params_(params),
      autonomy_enabled_(false),
      odom_initialized_(false),
      localization_initialized_(false),
      has_plan_(false),
      robot_vel_(0, 0),
      robot_omega_(0),
      robot_loc_(0, 0),
      robot_angle_(0),
      nav_complete_(true),
      nav_goal_loc_(0, 0),
      nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 10);
  local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  assert(params_.obstacle_margin < (1.0f / params_.max_curvature - params_.robot_width) / 2);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
}

void Navigation::SetAutonomy(bool autonomy_enabled) {
  autonomy_enabled_ = autonomy_enabled;

  if (!autonomy_enabled_ and command_history_.size() > 0) {
    printf("Clearing command history\n");
    command_history_.clear();  // Clear command history when autonomy is disabled
  }
}

void Navigation::UpdateLocation(const Vector2f& loc, float angle) {
  if (!localization_initialized_) {
    robot_start_loc_ = loc;
    robot_start_angle_ = angle;
    localization_initialized_ = true;
    robot_loc_ = loc;
    robot_angle_ = angle;
    return;
  }
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::PruneCommandQueue() {
  if (command_history_.empty()) return;
  const double update_time = min(t_odom_, t_point_cloud_);  // conservative time range for command history

  for (auto cmd_it = command_history_.begin(); cmd_it != command_history_.end();) {
    if (cmd_it->time < update_time - params_.dt) {  // we received another command that
                                                    // is closer to our update time
                                                    // anything farther in time away than 1 dt from
                                                    // update time is irrelevant
      cmd_it = command_history_.erase(cmd_it);
    } else {
      ++cmd_it;
    }
  }
}

void Navigation::UpdateOdometry(const Vector2f& loc, float angle, const Vector2f& vel, float ang_vel, double time) {
  if (FLAGS_v > 0) {
    cout << "================ [Navigation] UPDATE ODOMETRY ================" << endl;
    cout << "loc: " << loc << " angle: " << angle << " vel: " << vel.transpose() << " ang_vel: " << ang_vel
         << " time: " << time << endl;
    cout << "==============================================================\n" << endl;
  }

  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  t_odom_ = time;
  PruneCommandQueue();

  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud, double time) {
  if (FLAGS_v > 0) {
    cout << "=============== [Navigation] UPDATE POINTCLOUD ===============" << endl;
    cout << "cloud size: " << cloud.size() << " time: " << time << endl;
    cout << "==============================================================\n" << endl;
  }
  point_cloud_ = cloud;
  t_point_cloud_ = time;
  PruneCommandQueue();
}

void Navigation::UpdateCommandHistory(const AckermannCurvatureDriveMsg& drive_msg) {
  Command new_cmd;
  new_cmd.time = ros::Time::now().toSec() + params_.system_latency;
  new_cmd.drive_msg = drive_msg;

  command_history_.push_back(new_cmd);
}

void Navigation::ForwardPredict(double time) {
  if (FLAGS_v > 1) {
    cout << "================ [Navigation] FORWARD PREDICT ================" << endl;
    cout << "forward time: " << time << endl;
  }

  Eigen::Affine2f fp_odom_tf = Eigen::Translation2f(odom_loc_) * Eigen::Rotation2Df(odom_angle_);
  Eigen::Affine2f fp_local_tf = Eigen::Affine2f::Identity();
  for (const Command& cmd : command_history_) {
    const float cmd_v = cmd.drive_msg.velocity;
    const float cmd_omega = cmd.drive_msg.velocity * cmd.drive_msg.curvature;
    // Assume constant velocity and omega over the time interval
    // want to compute distance traveled (vel * dt) and arc length (ang_vel * dt)
    // and the value of the linear and angular velocity to use gives us the area
    // of the trapezoid in the v-t space when we multiply by dt
    const float v_mid = (robot_vel_.norm() + cmd_v) / 2.0f;
    const float omega_mid = (robot_omega_ + cmd_omega) / 2.0f;

    // Forward Predict Odometry
    if (cmd.time >= t_odom_ - params_.dt) {  // start forward integrating from command before our
                                             // most recent odometry (we are still executing the
                                             // command in the timestep before the odom message)
      // we know that this gives us the last command because the controller runs at a
      // fixed frequency and you generate one command per controller iteration every dt

      // in every iteration besides the first command in the control history, t_odom_ <
      // cmd.time
      const double dt = (t_odom_ > cmd.time)  // find time that we are executing the previous command for
                            ? min<double>(params_.dt - (t_odom_ - cmd.time), params_.dt)
                            : min<double>(time - cmd.time,
                                          params_.dt);  // upper bound on how long we execute a command

      // see trapezoid comment above for v_mid and omega_mid
      const float dtheta = omega_mid * dt;
      const float ds = v_mid * dt;
      // Translation coeff for exponential map of se2 -- lie algebra dead reckoning /
      // autonomous error equation: does not estimate future state based on past state
      // Special matrix that tells you how much the angular rate of rotation affects the
      // translation because we integrate the linear velocity and have angular rotation
      // unless we are moving straight, we accummulate error by just using the linear
      // velocity (the tangent -- it is a linear approximation) especially if our
      // curvature is high a smaller dt may not necessarily decrease error with the
      // linear velocity approximation; we could increase and decrease our error between
      // steps
      Eigen::Matrix2f V = Eigen::Matrix2f::Identity();
      if (fabs(dtheta) > kEpsilon) {
        V << sin(dtheta) / dtheta, (cos(dtheta) - 1) / dtheta, (1 - cos(dtheta)) / dtheta, sin(dtheta) / dtheta;
      }
      // Exponential map of translation part of se2 (in local frame)
      Eigen::Vector2f dloc = V * Eigen::Vector2f(ds, 0);
      // Update odom_tf in odom frame
      fp_odom_tf = fp_odom_tf * Eigen::Translation2f(dloc) * Eigen::Rotation2Df(dtheta);
      // Update odom_loc_ and odom_angle_ in odom frame
      fp_odom_tf = fp_odom_tf * Eigen::Translation2f(dloc) * Eigen::Rotation2Df(dtheta);
      odom_loc_ = fp_odom_tf.translation();
      odom_angle_ = atan2(fp_odom_tf(1, 0), fp_odom_tf(0, 0));

      // Update robot_vel_ and robot_omega_
      robot_vel_ = Eigen::Rotation2Df(odom_angle_) * Vector2f(cmd_v, 0);
      robot_omega_ = cmd_omega;
      t_odom_ = cmd.time;  // keep track of time of command we last executed

      if (FLAGS_v > 1) {
        cout << "dtheta " << dtheta << ", ds " << ds << endl;
        cout << "V \n" << V << endl;
        cout << "odom_loc_: " << odom_loc_.transpose() << ", odom_angle_: " << odom_angle_ << endl;
      }
    }

    // Forward Predict Point Cloud
    if (cmd.time >= t_point_cloud_ - params_.dt) {
      const double dt = (t_point_cloud_ > cmd.time) ? min<double>(params_.dt - (t_point_cloud_ - cmd.time), params_.dt)
                                                    : min<double>(time - cmd.time, params_.dt);
      // cout << "dt " << dt << endl;
      // cout << "params_.dt " << params_.dt << endl;
      // cout << "t_point_cloud_ " << t_point_cloud_ << " cmd.time " << cmd.time <<
      // endl;
      const float dtheta = omega_mid * dt;
      const float ds = v_mid * dt;
      // Translation coeff for exponential map of se2
      Eigen::Matrix2f V = Eigen::Matrix2f::Identity();
      if (fabs(dtheta) > kEpsilon) {
        V << sin(dtheta) / dtheta, (cos(dtheta) - 1) / dtheta, (1 - cos(dtheta)) / dtheta, sin(dtheta) / dtheta;
      }
      // cout << "V" << V << endl;
      // Exponential map of translation part of se2 (in local frame)
      Eigen::Vector2f dloc = V * Eigen::Vector2f(ds, 0);
      // Update local_tf in local frame
      fp_local_tf = fp_local_tf * Eigen::Translation2f(dloc) * Eigen::Rotation2Df(dtheta);
    }
  }

  // Forward Predict Point Cloud
  Eigen::Affine2f inv_fp_local_tf = fp_local_tf.inverse();
  fp_point_cloud_.resize(point_cloud_.size());
  for (size_t i = 0; i < point_cloud_.size(); i++) {
    fp_point_cloud_[i] = inv_fp_local_tf * point_cloud_[i];
  }
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) {
    return;
  }

  if (FLAGS_simulation || autonomy_enabled_) {
    // Predict laserscan and robot's odom state
    ForwardPredict(ros::Time::now().toSec() + params_.system_latency);

    cout << "RUN" << endl;

    if (FLAGS_Test1DTOC) {
      test1DTOC();
    }
    //if (FLAGS_TestSamplePaths) {
      //testSamplePaths(drive_msg_);
    //}
    planner(drive_msg_);

    drive_msg_.header.stamp = ros::Time::now();
    drive_pub_.publish(drive_msg_);
    // Queue the current command for future comparison.
    UpdateCommandHistory(drive_msg_);
  }

  // Visualize pointcloud
  for (auto point : fp_point_cloud_) {
    visualization::DrawPoint(point, 32762, local_viz_msg_);  // 32762
  }

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();

  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
}

void Navigation::test1DTOC() {
  float c = 0;
  // float r = 1 / c;
  float r = 100;
  drive_msg_.curvature = c;
  float theta = M_PI_4;
  Vector2f goal_loc =
      robot_start_loc_ + Rotation2Df(robot_start_angle_) * Vector2f(fabs(r) * sin(theta), r * (1 - cos(theta)));

  // Vector2f goal_loc_in_base_link_ = goal_loc - odom_loc_;
  visualization::DrawCross(Vector2f(0, 0), 0.5, 32762, global_viz_msg_);
  visualization::DrawCross(goal_loc, 0.1, 32762, global_viz_msg_);
  printf("Running 1D TOC test\n");
  printf("odom start loc x: %0.3f y: %0.3f theta:%0.3f\n", odom_start_loc_[0], odom_start_loc_[1], odom_start_angle_);
  printf("robot start loc x: %0.3f y: %0.3f theta %0.3f", robot_start_loc_[0], robot_start_loc_[1], robot_start_angle_);
  printf("robot loc x: %0.3f y: %0.3f theta %0.3f", robot_loc_[0], robot_loc_[1], robot_angle_);
  printf("odom loc x: %0.3f y: %0.3f theta:%0.3f\n", odom_loc_[0], odom_loc_[1], odom_angle_);
  printf("goal loc x: %0.3f y: %0.3f\n", goal_loc[0], goal_loc[1]);
  float d = (goal_loc - robot_loc_).norm();
  if (d > params_.goal_tolerance) {
    float left_theta = acos((Sq(r) + Sq(r) - Sq(d)) / (2 * Sq(r)));  // Law of Cosines
    float left_arc_length = fabs(r * left_theta);
    motion_primitives::ConstantCurvatureArc curve(c, left_arc_length);
    curve.getControlOnCurve(params_.linear_limits, robot_vel_.norm(), params_.dt, drive_msg_.velocity);
  } else {
    drive_msg_.velocity = 0;
    printf("Test complete\n");
    return;  // Don't update anything once test is finished
  }
}

void Navigation::planner(AckermannCurvatureDriveMsg& drive_msg) {
  // Current Pose:
  // robot_loc_;
  // robot_angle_;
  
  // Sample paths
  printf("Inside sample paths\n");
  printf("odom loc x: %0.3f y: %0.3f theta:%0.3f\n", odom_loc_[0], odom_loc_[1], odom_angle_);

  // Create Goal Configs
  //Vector2f goal(6.85,12.07);
  //Vector2f goal(8.0, 12.0);
  //Vector2f goal(9.0, 9.0);
  //Vector2f goal(-1.5, 5.0);
  //Vector2f goal(2.0, 7.0);
  Vector2f goal(-4.0, 6.0);
  double goal_radius = 0.25;
  visualization::DrawPoint(goal, 0xFF0000, global_viz_msg_);

  // 1. Create a plan if there is no plan
  std::list<rrt_tree::RRT_Node*> trajectory;
  if (!has_plan_) {
    // Make a plan if there is not existing plan
    printf("Makaing a tree\n");
    rrt_tree::RRT_Tree tree = rrt_tree::RRT_Tree(robot_loc_, robot_angle_);
    trajectory = tree.plan_trajectory(robot_loc_, robot_angle_, goal, goal_radius, map_);
    //printf("Planned a trajectory\n");
    //for (rrt_tree::RRT_Node* node_ptr : trajectory) {
    //  printf("Address: %p\n", node_ptr);
    //  printf("  Parent: %p\n", node_ptr->parent);
    printf("Trajectory length: %ld\n", trajectory.size());
    //}
    has_plan_ = true;
  }

  // 2. Given the current pose and current plan, select a global waypoint
  rrt_tree::RRT_Node* global_target_node = selectWaypoint(robot_loc_, robot_angle_, trajectory);
  if (global_target_node == NULL ){
    printf("No waypoint found");
    has_plan_ = false;
    return;
  }
  // Convert to base_link frame
  Vector2f local_target = Eigen::Rotation2Df(-robot_angle_)*(global_target_node->odom_loc - robot_loc_);

  // Visualize
  visualization::DrawCross(local_target, 0.5, 0xFF00FF, local_viz_msg_);
  visualization::DrawCross(goal, 0.6, 0x999900, global_viz_msg_);
  visualization::DrawCross(global_target_node->odom_loc, 0.6, 0x999900, global_viz_msg_);

  // 3. Check if a local plan exists to the selected waypoint.
  auto ackermann_sampler_ = motion_primitives::AckermannSampler(params_);
  ackermann_sampler_.update(robot_vel_, robot_omega_, local_target, point_cloud_);
  auto paths = ackermann_sampler_.getSamples(50);

  auto ackermann_evaluator_ = motion_primitives::AckermannEvaluator(params_);
  ackermann_evaluator_.update(local_target);
  auto best_path = ackermann_evaluator_.findBestPath(paths);
  // The best path is a curve. We can check if the endpoint is close to the local_target. If the endpoint
  // is close to the local target, then we assume that the local target is reachable from the robot's
  // current position.

  const float dist_to_target = (local_target - best_path->getEndPoint()).norm(); // Feasibility check
  // 3. This is the reachability check
  if (dist_to_target < 0.10) {
    // Assume reachable
    best_path->getControlOnCurve(params_.linear_limits, robot_vel_.norm(), params_.dt, drive_msg.velocity);
    drive_msg.curvature = best_path->curvature();

  } else {
    // Stop, and replan
    float zero_v = 0.0;
    best_path->getControlOnCurve(params_.linear_limits, robot_vel_.norm(), params_.dt, zero_v);
    drive_msg.curvature = best_path->curvature();
    has_plan_ = false; // Replan at next run
  }

  // Visualize
  for (auto path : paths) {
    visualization::DrawPathOption(path->curvature(), path->arc_length(), path->clearance(), 32762, false,
                                    local_viz_msg_);
      // cout << "idx: " << idx++ <<  ", Curvature: " << path->curvature() << " Arc
      // Length: " << path->arc_length()
      //      << " Clearance: " << path->clearance() << endl;
  }

  visualization::DrawPathOption(best_path->curvature(), best_path->arc_length(), best_path->clearance(), 10000, false,
                                  local_viz_msg_);

  return;
}

rrt_tree::RRT_Node* Navigation::selectWaypoint(
    Eigen::Vector2f& robot_loc_, 
    float robot_angle_, 
    std::list<rrt_tree::RRT_Node*>& trajectory) {
  // For selecting a waypoint, we find the one closest waypoint that is not at the car.

  bool found_waypoint = false;
  rrt_tree::RRT_Node* wayptr = NULL;
  while (!found_waypoint && !trajectory.empty()) {
    if (!trajectory.empty()) {
      auto node_ptr = trajectory.front();
      if ((robot_loc_ - node_ptr->odom_loc).norm() < 0.01) { 
          trajectory.pop_front();
          found_waypoint = false;
          wayptr = node_ptr; // If we exhaust all the waypoints in the list,
          // then at least wayptr will be the last one in the list.
      } else {
          found_waypoint = true;
          wayptr = node_ptr;
      }
    }
  }
  if (!found_waypoint) {
    has_plan_ = false;
  }

  return wayptr;
}



void Navigation::testSamplePaths(AckermannCurvatureDriveMsg& drive_msg) {
  // Sample paths
  printf("Inside sample paths\n");
  printf("odom loc x: %0.3f y: %0.3f theta:%0.3f\n", odom_loc_[0], odom_loc_[1], odom_angle_);

  // Create Goal Configs
  //Vector2f goal(6.85,12.07);
  //Vector2f goal(8.0, 12.0);
  //Vector2f goal(9.0, 9.0);
  //Vector2f goal(-1.5, 5.0);
  //Vector2f goal(2.0, 7.0);
  Vector2f goal(-4.0, 6.0);
  double goal_radius = 0.25;
  visualization::DrawPoint(goal, 0xFF0000, global_viz_msg_);



  //Navigation::SetNavGoal(Vector2f(-4.450, 6.680), 0.0);
  //std::vector<Vector2f> goal_configs;
  //Vector2f goal_delta(0.5, 0.5);
  //goal_configs.push_back(nav_goal_loc_ - goal_delta);
  //goal_configs.push_back(nav_goal_loc_ + goal_delta);
  // Get path
  printf("Makaing a tree\n");
  rrt_tree::RRT_Tree tree = rrt_tree::RRT_Tree(robot_loc_, robot_angle_);
  std::list<rrt_tree::RRT_Node*> trajectory = tree.plan_trajectory(robot_loc_, robot_angle_, goal, goal_radius, map_);
  //std::list<rrt_tree::RRT_Node*> trajectory = tree.plan_trajectory(robot_loc_, robot_angle_, goal_configs, map_);
  printf("Planned a trajectory\n");
  for (rrt_tree::RRT_Node* node_ptr : trajectory) {
    printf("Address: %p\n", node_ptr);
    printf("  Parent: %p\n", node_ptr->parent);
  }


  for (rrt_tree::RRT_Node* global_target_node : trajectory) {
    // Need to transform from global target to local target
    Vector2f local_target = Eigen::Rotation2Df(-robot_angle_)*(global_target_node->odom_loc - robot_loc_);
    //Vector2f local_target = Eigen::Rotation2Df(-robot_angle_)*(goal - robot_loc_);
    visualization::DrawCross(local_target, 0.5, 0xFF00FF, local_viz_msg_);
    visualization::DrawCross(goal, 0.6, 0x999900, global_viz_msg_);
    visualization::DrawCross(global_target_node->odom_loc, 0.6, 0x999900, global_viz_msg_);


    //Vector2f local_target = global_target_node->odom_loc;
    auto ackermann_sampler_ = motion_primitives::AckermannSampler(params_);
    ackermann_sampler_.update(robot_vel_, robot_omega_, local_target, point_cloud_);
    auto paths = ackermann_sampler_.getSamples(50);

    auto ackermann_evaluator_ = motion_primitives::AckermannEvaluator(params_);
    ackermann_evaluator_.update(local_target);
    auto best_path = ackermann_evaluator_.findBestPath(paths);
    best_path->getControlOnCurve(params_.linear_limits, robot_vel_.norm(), params_.dt, drive_msg.velocity);
    drive_msg.curvature = best_path->curvature();

    // Visualize paths
    // int idx = 0;
    for (auto path : paths) {
      visualization::DrawPathOption(path->curvature(), path->arc_length(), path->clearance(), 32762, false,
                                    local_viz_msg_);
      // cout << "idx: " << idx++ <<  ", Curvature: " << path->curvature() << " Arc
      // Length: " << path->arc_length()
      //      << " Clearance: " << path->clearance() << endl;
    }

    visualization::DrawPathOption(best_path->curvature(), best_path->arc_length(), best_path->clearance(), 10000, false,
                                  local_viz_msg_);
  //break;
  }

  return;
}

}  // namespace navigation
