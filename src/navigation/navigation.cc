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
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
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
const float max_vel = 1;
const float time_interval = 0.001; // TODO: Get an actual value for this?

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

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Based on selected curvature (and thus arc lenght), get potential velocity value

  // TODO: REMOVE THESE TEMP VALUES
  double arc_length = 5;
  double curvature = -1 / 1.05;

  double velocity = Navigation::get_abs_val_velocity(arc_length);
  drive_msg_.curvature = curvature;
  drive_msg_.velocity = velocity;

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
