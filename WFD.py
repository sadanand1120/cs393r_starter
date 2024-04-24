import queue

def is_frontier(node, occupancy_grid):
    # Evaluates whether a point is on the frontier of exploration
    if occupancy_grid[node[0], node[1]] != 0.5:
        return False # Assumes frontier must be unexplored
    
    for adj_node in get_adjacent(node):
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

            if n_x > 0 and n_x < len(occupancy_grid):
                assert False # This assumes 0 to max indexing for occupancy grid
                if n_y > 0 and n_y < len(occupancy_grid[0]):
                    assert False # This assumes that the occupancy grid is x by y 
                    if occupancy_grid[nx, ny] == 1: 
                        # Assumes 1 means obstacle
                        adj_list.append((n_x, n_y))

    return adj_dim_diffs

def get_next_obs_point(occupancy_grid, pose):
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