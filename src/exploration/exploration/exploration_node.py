import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

import ros2_numpy

import WFD

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class ExplorerNode(Node):

    def __init__(self):
        super().__init__('exploration_node')

        # Subscribe to the nav_msgs/OccupancyGrid topic...
        self.occupancy_map_subscriber = self.create_subscription(
            OccupancyGrid, ### TODO What's the write name for this?
            'nav_msgs/OccupancyGrid',
            self.process_occupancy_grid_callback,
            1) ## TODO: Queue size?

        self.pose_publisher = self.create_publisher(
            PoseStamped,
           '/goal_pose')

    def process_occupancy_grid_callback(self, occupancy_grid_msg):
        ## Turn the OccupancyGrid msg into a masked numpy array
        masked_np_array = ros_numpy.numpify(occupancy_grid_msg)
        ## Need the robot's current pose.

        ## Pass the array to WFD
        target_pose = WFD.get_next_obs_point(masked_np_array, current_pose)
        ## Publish the target_pose to the correct topic..
        ## TODO: Convert the target_pose to the proper message type
        pose_msg = navigator.getPoseStamped(target_pose, TurtleBot4Directions.NORTH)
        
        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    # TODO: Get the actual pose of the robot
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # TODO: Start the SLAM node
    # startSLAM()...

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    have_frontiers = True

    while have_frontiers:
       ## 1. Query SLAM node for an image ---
       ## NOTE: Turtlebot 4 Navigation Package uses
       ## "slam_toolbox".. Will need to run it in synchronous mode?
       ## It will publish a nav_msgs/OccupancyGrid to map
       
       ##  every map_update_interval frequency.
       ## There is a ros_numpy library that is able to convert ROS messages
       ## into numpy arrays
       ## 2. Get occupancy grid from the image, save at a numpyarray
       target_pose = WFD.get_next_obs_point(occupancy_grid, initial_pose)
       
       



    # Set goal poses
    goal_pose = navigator.getPoseStamped([13.0, 5.0], TurtleBot4Directions.EAST)

    # Undock
    navigator.undock()

    # Go to each goal pose
    navigator.startToPose(goal_pose)

    rclpy.shutdown()
"""
