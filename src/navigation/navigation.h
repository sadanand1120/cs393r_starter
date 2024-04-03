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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#include "vector_map/vector_map.h"
#include <deque>

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  // float clearance;
  float free_path_length;
  // Eigen::Vector2f obstruction;
  // Eigen::Vector2f closest_point;
  double score;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Controls {
  double time;
  double curvature;
  double velocity;
};

struct RRT_Node {
  struct RRT_Node* parent;
  double inbound_curvature;
  double inboud_vel;
  const Vector2f& odom_loc;
  const float odom_angle;
};

struct Action_Space{
  double min_curve = -1.0;
  double max_curve = 1.0;

  double delta_curve = 0.2;

  double min_vel = -1.0;
  double max_vel = 1.0;
  double delta_vel = 0.2;

  double max_time_step = 0.1;
}

class RRT_Tree {
  public:
    // Constructor
    explicit RRT_Tree(const Vector2f& root_odom_loc, const float root_odom_angle);

    struct RRT_Node find_closest(Vector2f sampled_config);

    std::vector<struct RRT_Node> make_trajectory(struct RRT_Node found_goal_config);
}

class Navigation {
 public:
  // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc, float angle, const Eigen::Vector2f& vel, float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud, double time);

  // Forward predicts the lidar from previous actions
  std::vector<Eigen::Vector2f> forward_predict_cloud(const std::vector<Eigen::Vector2f> cloud,
                                                     std::deque<Controls> controls);

  // This functions calculates the arc length given a curvature vaule
  // and maximum angle along that curvature
  double calc_arc_length(double curvature, double angle);

  // This function returns the absolute value of the velocity
  double get_velocity(double arc_length, double pred_vel);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  void FilterSectorRight(const Eigen::Vector2f& reference, const std::vector<Eigen::Vector2f>& cloud, float y_val,
                         float max_dist, std::vector<Eigen::Vector2f>& bucket);
  void FilterSectorLeft(const Eigen::Vector2f& reference, const std::vector<Eigen::Vector2f>& cloud, float y_val,
                        float max_dist, std::vector<Eigen::Vector2f>& bucket);
  void FilterDistances(const Eigen::Vector2f& reference, const std::vector<Eigen::Vector2f>& cloud, float min_dist,
                       float max_dist, std::vector<Eigen::Vector2f>& bucket);
  double MinimumDistanceToObstacle(const std::vector<Eigen::Vector2f>& cloud, double curvature);

  // Path scoring utilites
  std::vector<PathOption> GeneratePathOptions(const std::vector<Eigen::Vector2f>& new_cloud, float cmax, float cstep,
                                              float w1, float w2);
  PathOption ChooseBestPathOption(const std::vector<PathOption>& path_options);
  Eigen::Vector2f CalculateClosestPointOnArc(const Eigen::Vector2f& robot_loc, const Eigen::Vector2f& goal_loc,
                                             float curvature, float robot_angle);
  Eigen::Vector2f CalculateArcEndpoint(const Eigen::Vector2f& robot_loc, float curvature, float arc_length,
                                       float robot_angle);
  bool IsEndpointAfterCPA(const Eigen::Vector2f& robot_loc, const Eigen::Vector2f& closest_point_loc,
                          const Eigen::Vector2f& arc_endpoint_loc, float curvature, float arc_length);

 private:
  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;
  double obs_time;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
  // Map of the environment.
  vector_map::VectorMap map_;

  // Setup control latency queue
  std::deque<Controls> controls;
};

}  // namespace navigation

#endif  // NAVIGATION_H
