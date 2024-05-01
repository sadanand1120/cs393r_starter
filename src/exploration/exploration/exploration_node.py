import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

#from irobot_create_msgs.action import NavigateToPosition
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped

import numpy as np
import queue

import torch

import random

from matplotlib import pyplot as plt

#import WFD

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

def is_frontier(node, occupancy_grid):
    # Evaluates whether a point is on the frontier of exploration
    if occupancy_grid[node[0], node[1]] != 0.5:
        return False # Assumes frontier must be unexplored
    
    for adj_node in get_adjacent(node, occupancy_grid):
        if occupancy_grid[adj_node[0], adj_node[1]] == 0:
            return True # Assumes frontier must be adjacent to free space
        
    return False # if none of the above must not be frontier


def get_adjacent(node, occupancy_grid):
    # Get the adjacent nodes
    adj_list = []
    adj_dim_diffs = [-1, 0, 1]

    for x_diff in adj_dim_diffs:
        for y_diff in adj_dim_diffs:
            # 0,0 skip since that is current node
            if x_diff == 0 and y_diff == 0:
                continue

            n_x = node[0] + x_diff
            n_y = node[1] + y_diff

            if n_x > 0 and n_x < np.shape(occupancy_grid)[0]:
                #assert False # This assumes 0 to max indexing for occupancy grid
                if n_y > 0 and n_y < np.shape(occupancy_grid)[1]:
                    #assert False # This assumes that the occupancy grid is x by y 
                    if occupancy_grid[n_x, n_y] != 1: 
                        # Assumes 1 means obstacle
                        adj_list.append((n_x, n_y))

    return adj_list

def get_dist(pt1, pt2):
    x_diff = pt1[0] - pt2[0]
    y_diff = pt1[1] - pt2[1]

    return np.sqrt((x_diff**2) + (y_diff**2))

def round_occ_grid(grid):
    return np.round(grid*2)/2

def has_open_neighbour(occupancy_grid, node):
    for adj_node in get_adjacent(node, occupancy_grid):
        if occupancy_grid[adj_node[0], adj_node[1]] == 0.0:
            return True

    return False

def get_next_obs_point(occupancy_grid, pose, min_dist_to_next_pose=4):
    print("In WFD get_next_obs_point beginning")
    print(f"Occupancy Grid:{np.shape(occupancy_grid)}\n", occupancy_grid)
    print("Current Pose:\n", pose)

    if occupancy_grid[pose[0], pose[1]] != 0.0:
        print(pose)
        print(occupancy_grid[pose[0], pose[1]])
        occupancy_grid[pose[0], pose[1]] = 0.75
        occupancy_grid[14, 70] = 0.25 # (6.8323068230171575, -7.227799975885903), res: [0.19851485 0.19767442]
                                        # should be (4, -4), origin (-9,-7)

                                        # AT  (3.71, -3.29) -> (19, 69)
        plt.imshow(occupancy_grid)
        plt.show()
        assert False
        #occupancy_grid[pose[0], pose[1]] = 0.0

    # Round occupancy grid (if it's true probabilities)
    # such that 0 == assumed free space
    #           1 == assumed obstacle
    #           0.5 == assumed unexplored

    occupancy_grid = round_occ_grid(occupancy_grid)

    # Init map queue
    map_queue = queue.Queue()
    map_queue.put(pose)

    # Init lists to mark vals
    map_open_list = []
    map_open_list.append(pose)
    map_close_list = []
    frontier_open_list = []
    frontier_close_list = []

    # Init list of frontiers
    frontiers = []

    print("Starting WFD BFS")
    total_checked = 0

    while not map_queue.empty():
        print("Map Queue Size: ", map_queue.qsize(), \
            f"  Total Checked: {total_checked} of {np.shape(occupancy_grid)[0] * np.shape(occupancy_grid)[1]}")
        cur_node = map_queue.get()
        total_checked += 1

        if cur_node in map_close_list:
            continue

        if is_frontier(cur_node, occupancy_grid):
            frontier_queue = queue.Queue()
            new_frontier = []
            frontier_queue.put(cur_node)
            frontier_open_list.append(cur_node)

            while not frontier_queue.empty():
                cur_f_node = frontier_queue.get()

                if cur_f_node in map_close_list or cur_f_node in frontier_close_list:
                    continue

                if is_frontier(cur_f_node, occupancy_grid):
                    new_frontier.append(cur_f_node)

                    for adj_node in get_adjacent(cur_f_node, occupancy_grid):
                        if adj_node not in frontier_open_list and adj_node not in frontier_close_list and adj_node not in map_close_list:
                            frontier_queue.put(adj_node)
                            frontier_open_list.append(adj_node)

                frontier_close_list.append(cur_f_node)
                
            if new_frontier != []:
                frontiers.append(new_frontier)

                for node in new_frontier:
                    map_close_list.append(node)

        for adj_node in get_adjacent(cur_node, occupancy_grid):
            if adj_node not in map_open_list and adj_node not in map_close_list and has_open_neighbour(occupancy_grid, adj_node):
                map_queue.put(adj_node)

                map_open_list.append(adj_node)

        map_close_list.append(cur_node)

    print("Done WFD BFS. Starting search for best frontier")
    # Calculate the closest frontier median and return it
    next_loc = (-1,-1)
    min_dist = -1
    frontier_idx = 0
    cur_idx = 0
    print("Number of Frontiers", len(frontiers))
    print("Occupancy Grid Min/Max:\n", np.min(occupancy_grid), np.max(occupancy_grid))
    for frontier in frontiers:
        # Get the average x and y point
        x_avg = 0
        y_avg = 0

        for node in frontier:
            x_avg += node[0]/len(frontier)
            y_avg += node[1]/len(frontier)

        dist = get_dist(pose, (x_avg, y_avg))
        if (dist < min_dist and dist > min_dist_to_next_pose) or min_dist == -1:
            min_dist = dist
            frontier_idx = cur_idx

        cur_idx += 1

    rand_pose = random.choice(frontiers[frontier_idx])        
    next_loc = (int(rand_pose[0]), int(rand_pose[1]))

    print("Returning best frontier point: ", next_loc)

    return next_loc




def occupancygrid_to_numpy(msg, max_pool_data=True, max_data_dim=100):
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    data = np.array(data, dtype=np.float32)
    data = np.where(data < 0, 0.5, data/100.0)

    if max_pool_data:
        max_dim = np.max(np.shape(data))

        max_pool_dim = max_dim / (max_data_dim + 1)
        max_pool_dim = int(np.ceil(max_pool_dim))

        old_data_shape = np.shape(data)

        torch_data = torch.from_numpy(data)
        torch_data = torch_data.expand(1, torch_data.size()[0], torch_data.size()[1])
        m = torch.nn.MaxPool2d(max_pool_dim, stride=max_pool_dim)
        torch_data = m(torch_data)
        data = torch_data.numpy()
        data = np.squeeze(data)

        new_data_shape = np.shape(data)

        temp_resolution = msg.info.resolution * np.array(old_data_shape) / (np.array(new_data_shape) + 1)
        resolution = np.array([temp_resolution[1], temp_resolution[0]])

    else:
        resolution = msg.info.resolution

    origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    return data, origin, resolution

class ExplorerNode(Node):

    def __init__(self):
        print("In exploration Node init")
        super().__init__('exploration_node')
        
        #self.navigator = TurtleBot4Navigator()

        print("Waiting for Nav2")
        # Wait for Nav2
        #self.navigator.waitUntilNav2Active()
        print("Nav2 Active")

        #self.action_client= ActionClient(
        #    self, NavigateToPosition, '/ut/navigate_to_position')

        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Subscribe to the nav_msgs/OccupancyGrid topic...
        self.occupancy_map_subscriber = self.create_subscription(
            OccupancyGrid, ### TODO What's the write name for this?
            '/map',
            self.process_occupancy_grid_callback,
            1) ## TODO: Queue size?

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            1)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, #TODO IS THIS CORRECT?
            '/cmd_vel',
            self.process_cmd_vel,
            1
        )

        self.ut_cmd_vel_pub = self.create_publisher(
            Twist, # CORRECT?? TODO
            '/ut/cmd_vel',
            1
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.process_odom,
            1)

        self.cur_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.process_cur_pose,
            1
        )

        self.xy = (0,0)
        self.executing = False
        self.occupancy_grid_msg = None
        self.new_pose = False

        print("Done Exploration Node INIT")

    def process_odom(self, odom_msg):
        self.xy_odom = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)

    def process_cur_pose(self, pose_msg):
        self.xy = (pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y)
        self.new_pose = True

    def send_goal(self, pose_msg):
        #goal_msg = NavigateToPosition.Goal()
        goal_msg = NavigateToPose.Goal()

        self.executing = True

        #pose_msg = navigator.getPoseStamped(target_pose, angle)

        goal_msg.pose = pose_msg

        self.action_client.wait_for_server()

        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal Rejected")
            self.get_logger().info('Goal Rejected')
            self.exec_occupancy_grid(self.occupancy_grid_msg)
            return

        print("Successful Goal")

        self.get_logger().info('Goal Accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result {0}'.format(result))

        self.executing = False

        self.exec_occupancy_grid(self.occupancy_grid_msg)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback {0}'.format(feedback))

    def process_cmd_vel(self, cmd_vel_msg_):
        # Forward to ut/cmd_vel
        self.ut_cmd_vel_pub.publish(cmd_vel_msg_)

    def process_occupancy_grid_callback(self, occupancy_grid_msg):
        if not self.new_pose:
            return
        if self.executing:
            return

        self.new_pose = False

        if self.occupancy_grid_msg == None:
            self.exec_occupancy_grid(occupancy_grid_msg)

        self.occupancy_grid_msg = occupancy_grid_msg

        print("Got Occupancy Grid")

    def exec_occupancy_grid(self, occupancy_grid_msg):
        ## Turn the OccupancyGrid msg into a masked numpy array
        masked_np_array, offset, resolution = occupancygrid_to_numpy(occupancy_grid_msg)
        ## Need the robot's current pose.

        ## Pass the array to WFD
        print("Saved current xy: ", self.xy)
        current_pose = (int((self.xy[1] - offset[1])/resolution[1]), int((self.xy[0] - offset[0])/resolution[0]))
        target_pose = get_next_obs_point(masked_np_array, current_pose)
        ## Publish the target_pose to the correct topic..
        ## TODO: Convert the target_pose to the proper message type
        #pose_msg = self.navigator.getPoseStamped(target_pose, TurtleBot4Directions.NORTH)
        pose_msg = PoseStamped()

        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.position.x = float(target_pose[1] * resolution[1] + offset[0])
        pose_msg.pose.position.y = float(target_pose[0] * resolution[0] + offset[1])

        print(f"Sending Goal: {(pose_msg.pose.position.x, pose_msg.pose.position.y)} from position: {(self.xy[0], self.xy[1])}")

        rotation = TurtleBot4Directions.NORTH # TODO Need to actually pick a direciton intelligently
        pose_msg.pose.orientation.z = np.sin(np.deg2rad(rotation) / 2)
        pose_msg.pose.orientation.w = np.cos(np.deg2rad(rotation) / 2)

        self.executing = True

        self.send_goal(pose_msg)
        #self.pose_publisher.publish(pose_msg)

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
