#include "navigation.h"

#include <signal.h>


#include "amrl_msgs/Localization2DMsg.h"
#include "config_reader/config_reader.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "navigation.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/LaserScan.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

DEFINE_string(loc_topic, "localization", "Name of ROS topic for localization");
DEFINE_string(map, "GDC1", "Name of vector map file");

bool run_ = true;
void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

Vector2f robot_loc_(0.0, 0.0);
double robot_angle_ = 0.0;

void LocalizationCallback2(const amrl_msgs::Localization2DMsg msg) {
  cout << "IN LC2" << endl;
  robot_loc_ = Vector2f(msg.pose.x, msg.pose.y);
  robot_angle_ = msg.pose.theta;
}

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

int main(int argc, char** argv){
    google::ParseCommandLineFlags(&argc, &argv, false);
    signal(SIGINT, SignalHandler);

    ros::init(argc, argv, "high_level_nav_plan", ros::init_options::NoSigintHandler);
    ros::NodeHandle hn;  

    ros::Subscriber localization_sub =
      hn.subscribe(FLAGS_loc_topic, 1, &LocalizationCallback2);

    ros::Publisher set_loc_pub;
    set_loc_pub = hn.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    ros::spinOnce();

    // Init Map
    vector_map::VectorMap map_;
    map_.Load(GetMapFileFromName(FLAGS_map));

    // Create Goal Configs
    //Vector2f goal(6.85,12.07);
    //Vector2f goal(8.0, 12.0);
    Vector2f goal(9.0, 9.0);
    //Vector2f goal(-1.5, 5.0);
    double goal_radius = 0.25;
    cout << "ROBOT LOCATION: " << robot_loc_ << endl;
    rrt_tree::RRT_Tree tree = rrt_tree::RRT_Tree(robot_loc_, robot_angle_);
    std::list<rrt_tree::RRT_Node*> trajectory = tree.plan_trajectory(robot_loc_, robot_angle_, goal, goal_radius, map_);

    RateLoop loop(20.0);
    for (rrt_tree::RRT_Node* local_target_node : trajectory) {
        geometry_msgs::PoseStamped new_msg;
        new_msg.pose.position.x = local_target_node->odom_loc.x();
        new_msg.pose.position.y = local_target_node->odom_loc.y();
        
        set_loc_pub.publish(new_msg);

        while((robot_loc_ - local_target_node->odom_loc).norm() > goal_radius){
            ros::spinOnce();
            loop.Sleep();
        }
    }

    return 0;
}