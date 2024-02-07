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

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/math/geometry.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <deque>

using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using Eigen::Vector2f;
using std::sqrt;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

/* ********** *
 * PARAMETERS *
 * ********** *
 */

const float max_accel = 4;
const float max_vel = 1.0;
double time_interval = 0.1;      // TODO: Get an actual value for this?
double actuation_latency = 0.2;  // TODO: get an actual value for this

}  // namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n)
    : odom_initialized_(false),
      localization_initialized_(false),
      robot_loc_(0, 0),
      robot_angle_(0),
      robot_vel_(0, 0),
      robot_omega_(0),
      nav_complete_(true),
      nav_goal_loc_(0, 0),
      nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  controls = std::deque<Controls>();
}

double Navigation::MinimumDistanceToObstacle(const vector<Vector2f>& cloud, double curvature) {
  // Given an arc (expressed by the curvature parameter, and the car physical dimensions, compute the distance
  // that the car can travel from its current location to the obstacle.
  float actual_car_length = 0.535;  // [meters]
  float length_margin = 0.2;
  float car_length = actual_car_length + length_margin;
  float actual_car_width = 0.281;  // [meters]
  float width_margin = 0.2;
  float car_width = actual_car_width + width_margin;
  // float wheel_base = 0.324; // [meters]
  // float max_curvature = 1.0;
  float actual_offset = 0.09;  // meters <-- need to measure this
  float offset = actual_offset + (length_margin / 2);

  // We work in the baselink frame of reference. In the base link frame of reference, the center of turning is
  // located at (0, 1/c)
  // TODO: NOTE: Need to take care of case where curvature is 0.0

  if (curvature == 0.0) {
    // Filter for points that are immediately in front of the car:
    float closest_x = 1000.0;
    cout << "here" << cloud.size() << endl;
    for (Vector2f point : cloud) {
      float y0 = point.y();
      float x0 = point.x();
      if (y0 <= car_width / 2.0 && y0 >= -car_width / 2.0 && x0 >= actual_car_length - offset) {
        closest_x = std::min(closest_x, x0);
      }
    }
    return closest_x - car_length + offset;
  } else {  // curvature != 0.0

    Vector2f center_of_turning = Vector2f(0.0, 1.0 / curvature);

    float origin_to_left = 0.0;
    float origin_to_right = 0.0;
    float origin_to_front = car_length - offset;

    if (curvature > 0) {
      origin_to_left = 1.0 / curvature - car_width / 2.0;
      origin_to_right = 1.0 / curvature + car_width / 2.0;
      origin_to_front = car_length - offset;
    } else {  // Curvature < 0
      origin_to_left = -1.0 / curvature + car_width / 2.0;
      origin_to_right = -1.0 / curvature - car_width / 2.0;
      origin_to_front = car_length - offset;
    }

    // Vector2f rear_left = Vector2f(-offset, car_width / 2.0);
    // Vector2f front_left = Vector2f(car_length-offset, car_width / 2.0);
    // Vector2f rear_right = Vector2f(-offset, -car_width / 2.0);
    // Vector2f front_right = Vector2f(car_length-offset, -car_width / 2.0);

    float rear_left_radius = sqrt(offset * offset + origin_to_left * origin_to_left);
    float rear_right_radius = sqrt(offset * offset + origin_to_right * origin_to_right);
    float front_left_radius = sqrt(origin_to_front * origin_to_front + origin_to_left * origin_to_left);
    float front_right_radius = sqrt(origin_to_front * origin_to_front + origin_to_right * origin_to_right);

    // The shortest radius is rear_left; the longest is rear right
    // We can filter into regions: obstacles that would hit the left side of the car; obstacles that would hit
    // the right side of the car
    cout << "Curvature is : " << curvature << endl;
    cout << "Rear Right Radius:" << rear_right_radius << endl;
    cout << "Rear Left Radius:" << rear_left_radius << endl;
    cout << "Front Right Radius:" << front_right_radius << endl;
    cout << "Front Left Radius:" << front_left_radius << endl;

    vector<Vector2f> left_sweep = vector<Vector2f>();
    vector<Vector2f> front_sweep = vector<Vector2f>();
    vector<Vector2f> right_sweep = vector<Vector2f>();

    // Put the point cloud points into buckets based on where in the swept region they lie.
    if (curvature > 0) {
      // Left side of car
      Navigation::FilterDistances(center_of_turning, cloud, rear_left_radius, front_left_radius, left_sweep);
      // Front side of car
      Navigation::FilterDistances(center_of_turning, cloud, front_left_radius, front_right_radius, front_sweep);
      // Right side of car
      Navigation::FilterSectorRight(center_of_turning, cloud, -car_width / 2.0, rear_right_radius, right_sweep);
    } else {  // curvature is negative
      // Right side of car
      Navigation::FilterDistances(center_of_turning, cloud, rear_right_radius, front_right_radius, right_sweep);
      // Front side of car
      Navigation::FilterDistances(center_of_turning, cloud, front_right_radius, front_left_radius, front_sweep);
      // Left side of car
      Navigation::FilterSectorLeft(center_of_turning, cloud, car_width / 2.0, rear_left_radius, left_sweep);
    }

    cout << "Left Sweep Size: " << left_sweep.size() << endl;
    cout << "Right Sweep Size: " << right_sweep.size() << endl;
    cout << "Front Sweep Size: " << front_sweep.size() << endl;

    // For each point in the bucket, determine the minimum arc distance from the point to the car
    float cinv = 1.0 / curvature;
    float left_smallest = 2 * M_PI;
    float front_smallest = 2 * M_PI;
    float right_smallest = 2 * M_PI;
    if (curvature > 0) {
      // Distance angle of points to LEFT side of the car
      for (Vector2f point : left_sweep) {
        Vector2f delta = point - center_of_turning;
        // Point cloud point
        float x0 = point.x();
        float y0 = point.y();
        // Circle intersect with line segment solution
        float y_val = car_width / 2.0;
        float x_val = sqrt(Sq(x0) + Sq(y0 - cinv) - Sq(car_width / 2.0 - cinv));
        Vector2f p_val = Vector2f(x_val, y_val);
        Vector2f p_diff = p_val - center_of_turning;

        // Compute a dot b = |a||b|cos\theta --> \theta = Arccos(a dot b / (|a||b|))
        float cos_theta =
            delta.dot(p_val - center_of_turning) / (geometry::GetNormOrZero(delta) * geometry::GetNormOrZero((p_diff)));
        float angle = std::acos(cos_theta);
        // Check which side of the line the obstacle lies on
        if (x_val < 0.0) {
          if (y0 > (y_val - cinv) * x0 / x_val + cinv) {
            angle = 2 * M_PI - angle;
          }
        } else if (x_val > 0.0) {
          if (y0 < (y_val - cinv) * x0 / x_val + cinv) {
            angle = 2 * M_PI - angle;
          }
        } else {
          angle = M_PI;
        }
        // keep shortest distance to the car
        left_smallest = std::min(angle, left_smallest);
      }

      // Distance angle of points to FRONT side of the car
      for (Vector2f point : front_sweep) {
        Vector2f delta = point - center_of_turning;
        // Point cloud point
        float x0 = point.x();
        float y0 = point.y();
        // Circle intersect with line segment solution
        float x_val = car_length - offset;
        // NOTE that there are two solutions. The smaller one should have -w/2 < y < w/2, but we can test that
        // assumption.
        float y_val = cinv - sqrt(Sq(x0) + Sq(y0 - cinv) - Sq(x_val));
        // float y_pos = cinv + sqrt(Sq(x0) + Sq(y0-cinv) - Sq(x_val));
        // float y_val = neg;
        // if(y_val <= car_width/2.0 && y_val >= -car_width/2.0) {
        //   cout << "Valid value: "
        // }
        Vector2f p_val = Vector2f(x_val, y_val);
        Vector2f p_diff = p_val - center_of_turning;

        // Compute a dot b = |a||b|cos\theta --> \theta = Arccos(a dot b / (|a||b|))
        float cos_theta =
            delta.dot(p_val - center_of_turning) / (geometry::GetNormOrZero(delta) * geometry::GetNormOrZero(p_diff));
        float angle = std::acos(cos_theta);
        // Check which side of the line the obstacle lies on
        if (y0 < (y_val - cinv) / x_val + cinv) {
          angle = 2 * M_PI - angle;
        }

        // keep shortest distance to the car
        front_smallest = std::min(angle, front_smallest);
      }

      // Distance angle of points to RIGHT side of the car
      for (Vector2f point : right_sweep) {
        Vector2f delta = point - center_of_turning;
        // Point cloud point
        float x0 = point.x();
        float y0 = point.y();
        // Circle intersect with line segment solution
        float y_val = -car_width / 2.0;
        float x_val = -sqrt(Sq(x0) + Sq(y0 - cinv) - Sq(y_val - cinv));
        Vector2f p_val = Vector2f(x_val, y_val);
        Vector2f p_diff = p_val - center_of_turning;

        // Compute a dot b = |a||b|cos\theta --> \theta = Arccos(a dot b / (|a||b|))
        float cos_theta =
            delta.dot(p_val - center_of_turning) / (geometry::GetNormOrZero(delta) * geometry::GetNormOrZero(p_diff));
        float angle = std::acos(cos_theta);
        // keep shortest distance to the car
        right_smallest = std::min(angle, right_smallest);
      }
    } else {  // Curvature < 0
      // Distance angle of points to FRONT side of the car
      for (Vector2f point : front_sweep) {
        Vector2f delta = point - center_of_turning;
        // Point cloud point
        float x0 = point.x();
        float y0 = point.y();
        // Circle intersect with line segment solution
        float x_val = car_length - offset;
        // The different is the + sign
        float y_val = cinv + sqrt(Sq(x0) + Sq(y0 - cinv) - Sq(x_val));
        Vector2f p_val = Vector2f(x_val, y_val);
        Vector2f p_diff = p_val - center_of_turning;

        // Compute a dot b = |a||b|cos\theta --> \theta = Arccos(a dot b / (|a||b|))
        float cos_theta =
            delta.dot(p_val - center_of_turning) / (geometry::GetNormOrZero(delta) * geometry::GetNormOrZero(p_diff));
        float angle = std::acos(cos_theta);
        if (y0 > (y_val - cinv) / x_val + cinv) {
          angle = 2 * M_PI - angle;
        }
        // keep shortest distance to the car
        front_smallest = std::min(angle, front_smallest);
      }

      // Distance angle of points to RIGHT side of the car
      for (Vector2f point : right_sweep) {
        Vector2f delta = point - center_of_turning;
        // Point cloud point
        float x0 = point.x();
        float y0 = point.y();
        // Circle intersect with line segment solution
        float y_val = -car_width / 2.0;
        float x_val = sqrt(Sq(x0) + Sq(y0 - cinv) - Sq(y_val - cinv));
        Vector2f p_val = Vector2f(x_val, y_val);
        Vector2f p_diff = p_val - center_of_turning;

        // Compute a dot b = |a||b|cos\theta --> \theta = Arccos(a dot b / (|a||b|))
        float cos_theta =
            delta.dot(p_val - center_of_turning) / (geometry::GetNormOrZero(delta) * geometry::GetNormOrZero(p_diff));
        float angle = std::acos(cos_theta);
        if (x_val < 0.0) {
          if (y0 < (y_val - cinv) * x0 / x_val + cinv) {
            angle = 2 * M_PI - angle;
          }
        } else if (x_val > 0.0) {
          if (y0 > (y_val - cinv) * x0 / x_val + cinv) {
            angle = 2 * M_PI - angle;
          }
        } else {
          angle = M_PI;
        }
        // keep shortest distance to the car
        right_smallest = std::min(angle, right_smallest);
      }
      // Distance angle of points to LEFT side of the car
      for (Vector2f point : left_sweep) {
        Vector2f delta = point - center_of_turning;
        // Point cloud point
        float x0 = point.x();
        float y0 = point.y();
        // Circle intersect with line segment solution
        float y_val = car_width / 2.0;
        float x_val = -sqrt(Sq(x0) + Sq(y0 - cinv) - Sq(y_val - cinv));
        Vector2f p_val = Vector2f(x_val, y_val);
        Vector2f p_diff = p_val - center_of_turning;

        // Compute a dot b = |a||b|cos\theta --> \theta = Arccos(a dot b / (|a||b|))
        float cos_theta =
            delta.dot(p_val - center_of_turning) / (geometry::GetNormOrZero(delta) * geometry::GetNormOrZero(p_diff));
        float angle = std::acos(cos_theta);
        // keep shortest distance to the car
        left_smallest = std::min(angle, left_smallest);
      }
    }

    float smallest_angle = std::min(std::min(left_smallest, right_smallest), front_smallest);
    cout << "All Angles: L: " << left_smallest << " R: " << right_smallest << "F: " << front_smallest << endl;
    return Navigation::calc_arc_length(curvature, smallest_angle);
  }
}

void Navigation::FilterSectorRight(const Vector2f& reference, const vector<Vector2f>& cloud, float y_val,
                                   float max_dist, vector<Vector2f>& bucket) {
  // If the point is between the circle and the chord of the circle:
  float max_dist_sq = max_dist * max_dist;
  for (Vector2f point : cloud) {
    Vector2f delta = point - reference;
    float delta_sq = delta.squaredNorm();

    if (delta_sq <= max_dist_sq && point.y() <= y_val) {
      // Add point to the return bucket
      bucket.push_back(point);
    }
  }
}

void Navigation::FilterSectorLeft(const Vector2f& reference, const vector<Vector2f>& cloud, float y_val, float max_dist,
                                  vector<Vector2f>& bucket) {
  // If the point is between the circle and the chord of the circle:
  float max_dist_sq = max_dist * max_dist;
  for (Vector2f point : cloud) {
    Vector2f delta = point - reference;
    float delta_sq = delta.squaredNorm();

    if (delta_sq <= max_dist_sq && point.y() >= y_val) {
      // Add point to the return bucket
      bucket.push_back(point);
    }
  }
}

void Navigation::FilterDistances(const Vector2f& reference, const vector<Vector2f>& cloud, float min_dist,
                                 float max_dist, vector<Vector2f>& bucket) {
  // Given a point cloud, filter them into whether they are located between min_dist and max_dist from the reference
  // point
  float min_dist_sq = min_dist * min_dist;
  float max_dist_sq = max_dist * max_dist;

  for (Vector2f point : cloud) {
    Vector2f delta = point - reference;
    float delta_sq = delta.squaredNorm();

    if (delta_sq >= min_dist_sq && delta_sq <= max_dist_sq) {
      // Add point to the return bucket
      bucket.push_back(point);
    }
  }
  // The bucket should now be populated with the correct points.
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc, float angle, const Vector2f& vel, float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;

  // update robot_loc_ based on diff of odom
  robot_loc_ = robot_loc_ + (odom_loc_ - odom_start_loc_);
  // update robot angle
  robot_angle_ = robot_angle_ + (odom_angle_ - odom_start_angle_);
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud, double time) {
  // A scan from the cloud, associated with the baselink frame at that time.
  point_cloud_ = cloud;
  obs_time = time;
}

vector<Vector2f> Navigation::forward_predict_cloud(const vector<Vector2f> cloud, std::deque<Controls> controls) {
  Vector2f new_pose = Vector2f(0.0, 0.0);
  double angular_change = 0;

  // Update with odom
  // new_pose = new_pose + (robot_vel_ * time_interval);

  double last_time = 0.0;
  for (Controls i : controls) {
    if (last_time != 0.0) {
      time_interval = i.time - last_time;
    }

    last_time = i.time;

    // Apply control on robot loc and get location for next time step

    if (i.curvature == 0) {
      // Only need to update x pose in base link frame
      new_pose.x() += i.velocity * time_interval;  // TODO: What is the actual duration here?
    } else {
      double cur_angular_change = i.curvature * i.velocity * time_interval;  // TODO: What is the actual duration here?
      angular_change += cur_angular_change;                                  // TODO: What is the actual duration here?

      // Center of turning
      Eigen::Vector2f center_of_turning = Vector2f(0, 1 / i.curvature);

      // New pose in base link frame
      Eigen::Rotation2Df r(cur_angular_change);
      Vector2f arc_trans_pose = r * (-1 * center_of_turning) + center_of_turning;

      // New pose in map frame
      Eigen::Rotation2Df r_adj(angular_change);
      Vector2f rotated_arc_trans_pose = r * arc_trans_pose;
      new_pose += rotated_arc_trans_pose;
    }
  }

  // Calculate lidar estimate relative to new pose
  vector<Vector2f> new_cloud = vector<Vector2f>();
  for (Vector2f point : cloud) {
    // Apply rotation
    Eigen::Rotation2Df r(-angular_change);
    Vector2f new_point = r * point;

    // Apply translation
    new_point = new_point - new_pose;

    new_cloud.push_back(new_point);
  }

  return new_cloud;
}

double Navigation::get_velocity(double arc_length, double pred_vel) {
  cout << "Vel: " << pred_vel << endl;

  if (arc_length <= (abs(pred_vel) * abs(pred_vel)) / (2 * max_accel)) {
    // Decelerate
    if (pred_vel < 0) {
      cout << "Backwards Decelerating" << endl;
      return pred_vel + (max_accel * time_interval);
    } else {
      cout << "Forward Decelerating" << endl;
      return pred_vel - (max_accel * time_interval);
    }
  } else {
    // TODO Sadanand: not reqd, only equal to max_vel
    if (abs(pred_vel) >= max_vel) {
      // Maintain
      cout << "Maintaining (Default)" << endl;
      if (pred_vel < 0) {
        return -1 * max_vel;
      } else {
        return max_vel;
      }
    } else {
      // Find if we can accelerate safely here
      // by forward evaluation of constraints

      // TODO Sadanand: this fwd check not needed, just do whats in else here

      double temp_next_vel = abs(pred_vel) + (max_accel * time_interval);
      double temp_next_arc_length = arc_length - (temp_next_vel * time_interval);

      if (temp_next_arc_length <= (temp_next_vel * temp_next_vel) / (2 * max_accel)) {
        // Unsafe to accelerate
        cout << "Maintaining (Safety)" << endl;
        return pred_vel;
      } else {
        // Can accelerate safely here
        if (pred_vel < 0) {
          cout << "Backwards Accelerating" << endl;
          return pred_vel - (max_accel * time_interval);
        } else {
          cout << "Forward Accelerating" << endl;
          return pred_vel + (max_accel * time_interval);
        }
      }
    }
  }

  // TODO Sadanand: impossible, remove this
  // Default case, shouldn't reach
  cout << "BAD RETURN" << endl;
  return 0.0;
}

double Navigation::calc_arc_length(double curvature, double angle) { return abs(angle / curvature); }

vector<PathOption> Navigation::GeneratePathOptions(const vector<Vector2f>& new_cloud, float cmax, float cstep, float w1,
                                                   float w2) {
  vector<PathOption> path_options;
  for (float curvature = -cmax; curvature <= cmax; curvature += cstep) {
    PathOption option;
    option.curvature = curvature;
    option.free_path_length = Navigation::MinimumDistanceToObstacle(new_cloud, curvature);

    // get cpa on arc
    float fwd_x_goal = 50.0;
    const Eigen::Vector2f nav_goal_loc_temp = Vector2f(fwd_x_goal, 0);
    Eigen::Vector2f closest_point =
        Navigation::CalculateClosestPointOnArc(robot_loc_, nav_goal_loc_temp, curvature, robot_angle_);

    // get endpoint of arc
    Eigen::Vector2f endpoint =
        Navigation::CalculateArcEndpoint(robot_loc_, curvature, option.free_path_length, robot_angle_);

    float cpa_dist = 0.0;

    if (Navigation::IsEndpointAfterCPA(robot_loc_, closest_point, endpoint, curvature, option.free_path_length)) {
      cpa_dist = (closest_point - nav_goal_loc_temp).norm();
    } else {
      cpa_dist = (endpoint - nav_goal_loc_temp).norm();
    }

    option.score = option.free_path_length + w1 * (1 - abs(curvature)) + w2 * (fwd_x_goal - cpa_dist);
    std::cout << "Curvature: " << curvature << " free_path_length: " << option.free_path_length
              << " cpa_dist: " << cpa_dist << " score: " << option.score << std::endl;
    path_options.push_back(option);
  }
  return path_options;
}

PathOption Navigation::ChooseBestPathOption(const vector<PathOption>& path_options) {
  PathOption best_option;
  float highest_score = -std::numeric_limits<float>::max();
  for (const auto& option : path_options) {
    // Check if the current option has a higher score
    if (option.score > highest_score) {
      highest_score = option.score;
      best_option = option;
    }
    // In case of a tie, prefer an option with a curvature closer to 0.0
    else if (option.score == highest_score && std::abs(option.curvature) < std::abs(best_option.curvature)) {
      best_option = option;
    }
  }
  return best_option;
}

bool Navigation::IsEndpointAfterCPA(const Eigen::Vector2f& robot_loc, const Eigen::Vector2f& closest_point_loc,
                                    const Eigen::Vector2f& arc_endpoint_loc, float curvature, float arc_length) {
  if (std::abs(curvature) < 1e-6) {  // Check for straight line case
    // Movement in straight line
    return (arc_endpoint_loc - robot_loc).norm() > (closest_point_loc - robot_loc).norm();

  } else {
    // Calculate the angles from the center of the circle to the robot_loc, CPA, and endpoint
    // Calculate radius of the circle
    float radius = 1 / std::abs(curvature);
    // Calculate the center of the rotation circle
    Eigen::Vector2f center =
        robot_loc + Eigen::Rotation2Df(curvature > 0 ? -M_PI_2 : M_PI_2) * Eigen::Vector2f(0, radius);
    Eigen::Vector2f center_to_robot = robot_loc - center;
    Eigen::Vector2f center_to_cpa = closest_point_loc - center;
    Eigen::Vector2f center_to_endpoint = arc_endpoint_loc - center;

    float angle_to_robot = std::atan2(center_to_robot.y(), center_to_robot.x());
    float angle_to_cpa = std::atan2(center_to_cpa.y(), center_to_cpa.x());
    float angle_to_endpoint = std::atan2(center_to_endpoint.y(), center_to_endpoint.x());

    // Normalize angles to the range [0, 2π)
    auto normalize_angle = [](float angle) {
      while (angle < 0) angle += 2 * M_PI;
      while (angle >= 2 * M_PI) angle -= 2 * M_PI;
      return angle;
    };

    angle_to_robot = normalize_angle(angle_to_robot);
    angle_to_cpa = normalize_angle(angle_to_cpa);
    angle_to_endpoint = normalize_angle(angle_to_endpoint);

    // Determine if the endpoint is after the CPA based on curvature
    if (curvature > 0) {  // Left turn
      return normalize_angle(angle_to_endpoint - angle_to_robot) > normalize_angle(angle_to_cpa - angle_to_robot);
    } else {  // Right turn
      return normalize_angle(angle_to_robot - angle_to_endpoint) > normalize_angle(angle_to_robot - angle_to_cpa);
    }
  }
}

Eigen::Vector2f Navigation::CalculateArcEndpoint(const Eigen::Vector2f& robot_loc, float curvature, float arc_length,
                                                 float robot_angle) {
  Eigen::Vector2f endpoint;

  if (std::abs(curvature) < 1e-6) {  // Check for straight line case
    // Movement in straight line
    endpoint = robot_loc + Eigen::Vector2f(std::cos(robot_angle) * arc_length, std::sin(robot_angle) * arc_length);
  } else {
    // Calculate radius of the circle
    float radius = 1 / std::abs(curvature);
    // Calculate angle subtended by the arc (in radians)
    float angle = arc_length / radius;
    // Calculate the center of the rotation circle
    Eigen::Vector2f center =
        robot_loc + Eigen::Rotation2Df(curvature > 0 ? -M_PI_2 : M_PI_2) * Eigen::Vector2f(0, radius);

    // Calculate endpoint of the arc
    Eigen::Affine2f transform(Eigen::Translation2f(center) * Eigen::Rotation2Df(angle));
    endpoint = transform * (robot_loc - center) + center;
  }

  return endpoint;
}

Eigen::Vector2f Navigation::CalculateClosestPointOnArc(const Eigen::Vector2f& robot_loc,
                                                       const Eigen::Vector2f& goal_loc, float curvature,
                                                       float robot_angle) {
  Eigen::Vector2f closest_point_loc_;

  if (std::abs(curvature) < 1e-6) {
    // The path is a straight line
    Eigen::Vector2f robot_to_goal = goal_loc - robot_loc;
    Eigen::Vector2f robot_orientation(std::cos(robot_angle), std::sin(robot_angle));
    float distance_along_path = robot_to_goal.dot(robot_orientation);
    closest_point_loc_ = robot_loc + robot_orientation * distance_along_path;
  } else {
    // The path is an arc of a circle
    float radius = 1 / std::abs(curvature);
    Eigen::Vector2f circle_center =
        robot_loc + Eigen::Rotation2Df(curvature > 0 ? -M_PI_2 : M_PI_2) * Eigen::Vector2f(0, radius);

    Eigen::Vector2f center_to_goal = goal_loc - circle_center;
    float goal_dist = center_to_goal.norm();

    // Check if the goal is inside the circle
    if (goal_dist < radius) {
      // The closest point is not defined in this case because the goal is inside the turn circle
      // return Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
      // return 0 TODO: hack
      return Eigen::Vector2f(0, 0);
    }

    // Calculate the angle between the robot's starting location and the goal location
    float angle_to_goal = std::atan2(center_to_goal.y(), center_to_goal.x());
    // Calculate the angle from the robot's starting location to the closest point on the arc
    float angle_to_closest_point = angle_to_goal - std::asin(radius / goal_dist);

    // Calculate closest point coordinates on the arc
    Eigen::Rotation2Df rot(angle_to_closest_point);
    closest_point_loc_ = circle_center + rot * Eigen::Vector2f(radius, 0);
  }

  return closest_point_loc_;
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // Forward predict point_cloud_ and pop top of queue that is too old
  while (!controls.empty()) {
    if (!controls.empty() && ((obs_time - controls.front().time) > actuation_latency)) {
      controls.pop_front();
    } else {
      break;
    }
  }

  vector<Vector2f> new_cloud = Navigation::forward_predict_cloud(point_cloud_, controls);

  // Scoring
  float cmax = 1.0;
  float cstep = 0.05;
  float w1 = 0.0;
  float w2 = 2.0;
  vector<PathOption> path_options = Navigation::GeneratePathOptions(new_cloud, cmax, cstep, w1, w2);
  PathOption best_option = Navigation::ChooseBestPathOption(path_options);

  // vector<Vector2f> new_cloud = point_cloud_;

  // The control iteration goes here.
  // Feel free to make helper functions to structure the control appropriately.

  // The latest observed point cloud is accessible via "point_cloud_"

  // Based on selected curvature (and thus arc lenght), get potential velocity value

  double curvature = best_option.curvature;

  // TODO: Transform point cloud to baselink frame.
  // Navigation::TransformPointCloudToBaseLink(point_cloud_, offset);

  double arc_length = Navigation::MinimumDistanceToObstacle(new_cloud, curvature);

  cout << "Arc Length: " << arc_length << endl;

  double velocity = 0.0;
  if (!controls.empty()) {
    velocity = Navigation::get_velocity(arc_length, controls.back().velocity);
  } else {
    velocity = Navigation::get_velocity(arc_length, 0.0);
  }

  drive_msg_.curvature = curvature;
  cout << "Sending Velocity: " << velocity << endl;
  cout << "Queue Size: " << controls.size() << endl;
  drive_msg_.velocity = velocity;

  Controls cur_control = {.time = ros::Time::now().toSec(), .curvature = curvature, .velocity = velocity};

  controls.push_back(cur_control);

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
