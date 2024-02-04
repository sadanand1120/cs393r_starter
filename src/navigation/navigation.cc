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

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using std::sqrt;

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
const float max_vel = 1;
const float time_interval = 0.10; // TODO: Get an actual value for this?
ros::Duration actuation_latency(0.15); // TODO: get an actual value for this

} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  controls = std::deque<Controls>();
}

double Navigation::MinimumDistanceToObstacle(const vector<Vector2f>& cloud, double curvature) {
  // Given an arc (expressed by the curvature parameter, and the car physical dimensions, compute the distance
  // that the car can travel from its current location to the obstacle.
  float car_length = 0.535; // [meters]
  float car_width = 0.281;  // [meters]
  //float wheel_base = 0.324; // [meters]
  //float max_curvature = 1.0;
  float offset = 0.001; // meters <-- need to measure this

  // We work in the baselink frame of reference. In the base link frame of reference, the center of turning is
  // located at (0, 1/c)
  // TODO: NOTE: Need to take care of case where curvature is 0.0

  if(curvature == 0.0) {
    // Filter for points that are immediately in front of the car:
    float closest_x = 1000.0;
    cout << "here" << cloud.size() << endl;
    for(Vector2f point : cloud) {
      float y0 = point.y();
      float x0 = point.x();
      if(y0 <= car_width /2.0 && y0 >= -car_width/2.0 && x0 >= car_length - offset) {
        closest_x = std::min(closest_x, x0);
      }
    } 
    return closest_x - car_length + offset;
  } else { // curvature != 0.0

  Vector2f center_of_turning = Vector2f(0.0, 1.0/curvature);
  float origin_to_left = 1.0/curvature - car_width / 2.0;
  float origin_to_right = 1.0/curvature + car_width / 2.0;
  float origin_to_front = car_length - offset;

  //Vector2f rear_left = Vector2f(-offset, car_width / 2.0);
  //Vector2f front_left = Vector2f(car_length-offset, car_width / 2.0);
  //Vector2f rear_right = Vector2f(-offset, -car_width / 2.0);
  //Vector2f front_right = Vector2f(car_length-offset, -car_width / 2.0);

  float rear_left_radius = sqrt(offset * offset + origin_to_left * origin_to_left);
  float rear_right_radius = sqrt(offset * offset + origin_to_right * origin_to_right);
  float front_left_radius = sqrt(origin_to_front * origin_to_front + origin_to_left * origin_to_left);
  float front_right_radius = sqrt(origin_to_front * origin_to_front + origin_to_right * origin_to_right);

  // The shortest radius is rear_left; the longest is rear right
  // We can filter into regions: obstacles that would hit the left side of the car; obstacles that would hit
  // the right side of the car

  vector<Vector2f> left_sweep = vector<Vector2f>();
  vector<Vector2f> front_sweep = vector<Vector2f>();
  vector<Vector2f> right_sweep = vector<Vector2f>();

  // Put the point cloud points into buckets based on where in the swept region they lie.
  if(curvature > 0) {
    // Left side of car
    Navigation::FilterDistances(center_of_turning, cloud, rear_left_radius, front_left_radius, left_sweep);
    // Front side of car
    Navigation::FilterDistances(center_of_turning, cloud, front_left_radius, front_right_radius, front_sweep);
    // Right side of car
    Navigation::FilterSectorRight(center_of_turning, cloud, rear_right_radius, front_right_radius, right_sweep);
  } else { // curvature is negative
    // Right side of car
    Navigation::FilterDistances(center_of_turning, cloud, rear_right_radius, front_right_radius, right_sweep);
    // Front side of car
    Navigation::FilterDistances(center_of_turning, cloud, front_right_radius, front_left_radius, front_sweep);
    // Left side of car
    Navigation::FilterSectorLeft(center_of_turning, cloud, rear_left_radius, front_left_radius, left_sweep);
  }

  // For each point in the bucket, determine the minimum arc distance from the point to the car
  float cinv = 1.0/curvature;
  float left_smallest = 2*M_PI;
  float front_smallest = 2*M_PI;
  float right_smallest = 2*M_PI;
  if(curvature > 0) {
    // Distance angle of points to LEFT side of the car
    for(Vector2f point : left_sweep) {
      Vector2f delta = point - center_of_turning;
      // Point cloud point
      float x0 = point.x();
      float y0 = point.y();
      // Circle intersect with line segment solution
      float y_val = car_width/2.0;
      float x_val = sqrt(Sq(x0) + Sq(y0-cinv) - Sq(car_width/2.0 - cinv));
      Vector2f p_val = Vector2f(x_val, y_val);
      Vector2f p_diff = p_val - center_of_turning;

      // Compute a dot b = |a||b|cos\theta --> \theta = Arccos(a dot b / (|a||b|))
      float cos_theta = delta.dot(p_val - center_of_turning) / 
        (geometry::GetNormOrZero(delta)*geometry::GetNormOrZero((p_diff)));
      float angle = std::acos(cos_theta);
      // keep shortest distance to the car
      left_smallest = std::min(angle, left_smallest);
    }

    // Distance angle of points to FRONT side of the car
    for(Vector2f point : front_sweep) {
      Vector2f delta = point - center_of_turning;
      // Point cloud point
      float x0 = point.x();
      float y0 = point.y();
      // Circle intersect with line segment solution
      float x_val = car_length-offset;
      float y_val = cinv - sqrt(Sq(x0) + Sq(y0-cinv) - Sq(x_val));
      Vector2f p_val = Vector2f(x_val, y_val);
      Vector2f p_diff = p_val - center_of_turning;

      // Compute a dot b = |a||b|cos\theta --> \theta = Arccos(a dot b / (|a||b|))
      float cos_theta = delta.dot(p_val - center_of_turning) / 
        (geometry::GetNormOrZero(delta)*geometry::GetNormOrZero(p_diff));
      float angle = std::acos(cos_theta);
      // keep shortest distance to the car
      front_smallest = std::min(angle, front_smallest);
    }
    
    // Distance angle of points to RIGHT side of the car
    for(Vector2f point : right_sweep) {
      Vector2f delta = point - center_of_turning;
      // Point cloud point
      float x0 = point.x();
      float y0 = point.y();
      // Circle intersect with line segment solution
      float y_val = -car_width/2.0;
      float x_val = -sqrt(Sq(x0) + Sq(y0-cinv) - Sq(y_val-cinv));
      Vector2f p_val = Vector2f(x_val, y_val);
      Vector2f p_diff = p_val - center_of_turning;

      // Compute a dot b = |a||b|cos\theta --> \theta = Arccos(a dot b / (|a||b|))
      float cos_theta = delta.dot(p_val - center_of_turning) / 
        (geometry::GetNormOrZero(delta)*geometry::GetNormOrZero(p_diff));
      float angle = std::acos(cos_theta);
      // keep shortest distance to the car
      right_smallest = std::min(angle, right_smallest);
    }
  } else { // Curvature < 0
    // Distance angle of points to FRONT side of the car
    for(Vector2f point : front_sweep) {
      Vector2f delta = point - center_of_turning;
      // Point cloud point
      float x0 = point.x();
      float y0 = point.y();
      // Circle intersect with line segment solution
      float x_val = car_length-offset;
      // The different is the + sign
      float y_val = cinv + sqrt(Sq(x0) + Sq(y0-cinv) - Sq(x_val));
      Vector2f p_val = Vector2f(x_val, y_val);
      Vector2f p_diff = p_val - center_of_turning;

      // Compute a dot b = |a||b|cos\theta --> \theta = Arccos(a dot b / (|a||b|))
      float cos_theta = delta.dot(p_val - center_of_turning) / 
        (geometry::GetNormOrZero(delta)*geometry::GetNormOrZero(p_diff));
      float angle = std::acos(cos_theta);
      // keep shortest distance to the car
      front_smallest = std::min(angle, front_smallest);
    }
    
    // Distance angle of points to RIGHT side of the car
    for(Vector2f point : right_sweep) {
      Vector2f delta = point - center_of_turning;
      // Point cloud point
      float x0 = point.x();
      float y0 = point.y();
      // Circle intersect with line segment solution
      float y_val = -car_width/2.0;
      float x_val = sqrt(Sq(x0) + Sq(y0-cinv) - Sq(y_val-cinv));
      Vector2f p_val = Vector2f(x_val, y_val);
      Vector2f p_diff = p_val - center_of_turning;

      // Compute a dot b = |a||b|cos\theta --> \theta = Arccos(a dot b / (|a||b|))
      float cos_theta = delta.dot(p_val - center_of_turning) / 
        (geometry::GetNormOrZero(delta)*geometry::GetNormOrZero(p_diff));
      float angle = std::acos(cos_theta);
      // keep shortest distance to the car
      right_smallest = std::min(angle, right_smallest);
    }
    // Distance angle of points to LEFT side of the car
    for(Vector2f point : left_sweep) {
      Vector2f delta = point - center_of_turning;
      // Point cloud point
      float x0 = point.x();
      float y0 = point.y();
      // Circle intersect with line segment solution
      float y_val = car_width/2.0;
      float x_val = -sqrt(Sq(x0) + Sq(y0-cinv) - Sq(y_val - cinv));
      Vector2f p_val = Vector2f(x_val, y_val);
      Vector2f p_diff = p_val - center_of_turning;

      // Compute a dot b = |a||b|cos\theta --> \theta = Arccos(a dot b / (|a||b|))
      float cos_theta = delta.dot(p_val - center_of_turning) / 
        (geometry::GetNormOrZero(delta)*geometry::GetNormOrZero(p_diff));
      float angle = std::acos(cos_theta);
      // keep shortest distance to the car
      left_smallest = std::min(angle, left_smallest);
    }
  }

  float smallest_angle = std::min(std::min(left_smallest, right_smallest), front_smallest);
  return Navigation::calc_arc_length(curvature, smallest_angle);
  }
}

void Navigation::FilterSectorRight(const Vector2f& reference, const vector<Vector2f>& cloud, float y_val, float max_dist, vector<Vector2f>& bucket) {
  // If the point is between the circle and the chord of the circle:
  float max_dist_sq = max_dist * max_dist;
  for(Vector2f point : cloud) {
    Vector2f delta = point - reference;
    float delta_sq = delta.squaredNorm();
    
    if (delta_sq <= max_dist_sq && point.y() <= y_val) {
      // Add point to the return bucket
      bucket.push_back(point);
    }
  }
}

void Navigation::FilterSectorLeft(const Vector2f& reference, const vector<Vector2f>& cloud, float y_val, float max_dist, vector<Vector2f>& bucket) {
  // If the point is between the circle and the chord of the circle:
  float max_dist_sq = max_dist * max_dist;
  for(Vector2f point : cloud) {
    Vector2f delta = point - reference;
    float delta_sq = delta.squaredNorm();
    
    if (delta_sq <= max_dist_sq && point.y() >= y_val) {
      // Add point to the return bucket
      bucket.push_back(point);
    }
  }
}

void Navigation::FilterDistances(const Vector2f& reference, const vector<Vector2f>& cloud, float min_dist, float max_dist, vector<Vector2f>& bucket) {
  // Given a point cloud, filter them into whether they are located between min_dist and max_dist from the reference point
  float min_dist_sq = min_dist * min_dist;
  float max_dist_sq = max_dist * max_dist;

  for(Vector2f point : cloud) {
    Vector2f delta = point - reference;
    float delta_sq = delta.squaredNorm();

    if (delta_sq >= min_dist_sq && delta_sq <= max_dist_sq) {
      // Add point to the return bucket
      bucket.push_back(point);
    }
  }
  // The bucket should now be populated with the correct points.
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
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
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

vector<Vector2f> Navigation::forward_predict_cloud(const vector<Vector2f> cloud, std::deque<Controls> controls){
	Vector2f new_pose = Vector2f(0.0, 0.0);
	double angular_change = 0;
	for (Controls i: controls){
		//Apply control on robot loc and get location for next time step

		if (i.curvature == 0){
			// Only need to update x pose in base link frame
			new_pose.x() += i.velocity * time_interval; //TODO: What is the actual duration here?
		} else {
			angular_change += i.curvature * i.velocity * time_interval; //TODO: What is the actual duration here?

			// Center of turning
			Eigen::Vector2f center_of_turning = Vector2f(0, 1/i.curvature);

			// New pose in base link frame
			Eigen::Rotation2Df r(angular_change);
			Vector2f new_pose = r * (-1 * center_of_turning);

			// New pose in map frame
			new_pose += new_pose;
		}
	}

	// Calculate lidar estimate relative to new pose
	vector<Vector2f> new_cloud = vector<Vector2f>();
	for (Vector2f point : cloud){
		// Apply rotation
		Eigen::Rotation2Df r(angular_change);
		Vector2f new_point = r * point;

		// Apply translation
		new_point = new_point + new_pose;

		new_cloud.push_back(new_point);
	}

	return new_cloud;
}

bool Navigation::check_is_backward(){
	// Now check whether base link velocity is positive in x (i.e. forward)
	cout << "Robot X Vel: " << robot_vel_.x() << endl;
	cout << "Robot Y Vel: " << robot_vel_.y() << endl;
	return robot_vel_.x() < 0;
}

double Navigation::get_abs_val_velocity(double arc_length){
	double cur_vel_abs_val = sqrt(robot_vel_.x()*robot_vel_.x() + robot_vel_.y()*robot_vel_.y());

      	bool is_backward = Navigation::check_is_backward();

	cout << "Vel Abs Val: " << cur_vel_abs_val << endl;
	cout << is_backward << endl;

	if (arc_length <= cur_vel_abs_val / (2 * max_accel)){
		// Decelerate
		if (is_backward){
			cout << "Backwards Decelerating" << endl;
			return (-1 * cur_vel_abs_val) + (max_accel * time_interval);
		} else {
			cout << "Forward Decelerating" << endl;
			return cur_vel_abs_val - (max_accel * time_interval);
		}
	} else {
		if (cur_vel_abs_val >= max_vel){
			// Maintain
			cout << "Maintaining (Default)" << endl;
			return max_vel;
		} else {
			// Find if we can accelerate safely here
			// by forward evaluation of constraints
			
			double temp_next_vel = cur_vel_abs_val + (max_accel * time_interval);
			double temp_next_arc_length = arc_length - (temp_next_vel * time_interval);

			if (temp_next_arc_length <= temp_next_vel / (2 * max_accel)){
				// Unsafe to accelerate
				cout << "Maintaining (Safety)" << endl;
				return cur_vel_abs_val;
			} else {
				// Can accelerate safely here
				if (is_backward){
					cout << "Backwards Accelerating" << endl;
					return (-1 * cur_vel_abs_val) - (max_accel * time_interval);
				} else {
					cout << "Forward Accelerating" << endl;
					return cur_vel_abs_val + (max_accel * time_interval);
				}
			}
		}
	}

	// Default case, shouldn't reach
	cout << "BAD RETURN" << endl;
	return 0.0;
}

double Navigation::calc_arc_length(double curvature, double angle){
	return angle / curvature;
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // Forward predict point_cloud_ and pop top of queue that is too old
  while (!controls.empty()){
	if (!controls.empty() && ((ros::Time::now() - controls.front().time) > actuation_latency)){
  		controls.pop_front();
	} else {
		break;
	}
  }
  vector<Vector2f> new_cloud = Navigation::forward_predict_cloud(point_cloud_, controls);

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Based on selected curvature (and thus arc lenght), get potential velocity value

  // TODO: REMOVE THESE TEMP VALUES
  double curvature = 0;

  // TODO: Transform point cloud to baselink frame.
  //Navigation::TransformPointCloudToBaseLink(point_cloud_, offset);

  double arc_length = Navigation::MinimumDistanceToObstacle(new_cloud, curvature);

  cout << "Arc Length: " << arc_length << endl;

  double velocity = Navigation::get_abs_val_velocity(arc_length);
  drive_msg_.curvature = curvature;
  cout << "Sending Velocity: " << velocity << endl;
  cout << "Queue Size: " << controls.size() << endl;
  drive_msg_.velocity = velocity;

  Controls cur_control = {.time = ros::Time::now(), .curvature = curvature, .velocity = velocity };

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
