#include <stdio.h>
#include <stdint.h>

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



int main() {
   // This test checks the arc length computations.
  float actual_car_length = 0.535; // [meters]
  float length_margin = 0.05;
  float car_length = actual_car_length + length_margin;
  float actual_car_width = 0.281;  // [meters]
  float width_margin = 0.05;
  float car_width = actual_car_width + width_margin;
  //float wheel_base = 0.324; // [meters]
  //float max_curvature = 1.0;
  float actual_offset = 0.1; // meters <-- need to measure this
  float offset = actual_offset + (length_margin / 2);

  // Above is the car model
  float origin_to_left = 0.0;
  float origin_to_right = 0.0;
  float origin_to_front = car_length - offset;

  float curvature; // Read in from the command line

  // Compute the car circles.
  if(curvature > 0) {
    origin_to_left = 1.0/curvature - car_width / 2.0;
    origin_to_right = 1.0/curvature + car_width / 2.0;
    origin_to_front = car_length - offset;
  } else { // Curvature < 0
    origin_to_left = 1.0/curvature + car_width / 2.0;
    origin_to_right = 1.0/curvature - car_width / 2.0;
    origin_to_front = car_length - offset;
  }

  float rear_left_radius = sqrt(offset * offset + origin_to_left * origin_to_left);
  float rear_right_radius = sqrt(offset * offset + origin_to_right * origin_to_right);
  float front_left_radius = sqrt(origin_to_front * origin_to_front + origin_to_left * origin_to_left);
  float front_right_radius = sqrt(origin_to_front * origin_to_front + origin_to_right * origin_to_right);

  test_point = point_on_circle(curvature, radius, theta);
  vector<Vector2f> v();
  v.push_back(test_point);

  double arc_length = Navigation::MinimumDistanceToObstacle(v, curvature);

}

Vector2f point_on_circle(float curvature, float radius, float theta) {
  // Given a radius, returns a circle centered at the center of turning, which
  // is (0, 1/c)

  Vector2f point = Vector2f(radius*std::cos(theta), radius*std::sin(theta) + 1/curvature);
  return point;
}
