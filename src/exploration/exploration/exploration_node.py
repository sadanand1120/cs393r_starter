import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

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

    return math.sqrt((x_diff**2) + (y_diff**2))

def round_occ_grid(grid):
    return np.round(grid*2)/2

def get_next_obs_point(occupancy_grid, pose):
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
    map_close_list = []
    frontier_open_list = []
    frontier_close_list = []

    # Init list of frontiers
    frontiers = []

    while not map_queue.empty():
        cur_node = map_queue.get()

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
                
            frontiers.append(new_frontier)

            for node in new_frontier:
                map_close_list.append(node)

        for adj_node in get_adjacent(cur_node, occupancy_grid):
            if adj_node not in map_open_list and adj_node not in map_close_list:
                map_queue.put(adj_node)

                map_open_list.append(adj_node)

        map_close_list.append(cur_node)

    
    # Calculate the closest frontier median and return it
    next_loc = (-1,-1)
    min_dist = -1
    print(len(frontiers))
    for frontier in frontiers:
        # Get the average x and y point
        x_avg = 0
        y_avg = 0

        for node in frontier:
            x_avg += node[0]/len(frontier)
            y_avg += node[1]/len(frontier)

        dist = get_dist(pose, (x_avg, y_avg))
        if dist < min_dist or min_dist == -1:
            min_dist = dist
            
            next_loc = (int(round(x_avg)), int(round(y_avg)))

    return next_loc




def occupancygrid_to_numpy(msg):
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    return np.ma.array(data, mask=data==-1, fill_value=-1)

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
            '/goal_pose',
            1)

    def process_occupancy_grid_callback(self, occupancy_grid_msg):
        ## Turn the OccupancyGrid msg into a masked numpy array
        masked_np_array = occupancygrid_to_numpy(occupancy_grid_msg)
        ## Need the robot's current pose.

        ## Pass the array to WFD
        target_pose = get_next_obs_point(masked_np_array, current_pose)
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
