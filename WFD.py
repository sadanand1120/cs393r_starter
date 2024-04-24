import queue

def is_frontier(node, occupancy_grid):
    # Evaluates whether a point is on the frontier of exploration

def get_adjacent(node, occupancy_grid):
    # Get the adjacent nodes

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