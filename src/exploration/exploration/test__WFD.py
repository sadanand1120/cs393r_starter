import sys
import os
sys.path.append(os.getcwd())

import math
import numpy as np
import WFD

# Unit Tests
dist = WFD.get_dist((0,0), (0,0))
assert dist == 0

print('Test 1 Passed')

dist = WFD.get_dist((10.30894890,2.13), (12.1293,9.10923290))

assert dist == math.sqrt((10.30894890-12.1293)**2 + (2.13-9.10923290)**2)
print('Test 2 Passed')


# Occupancy grid (will modify later)
occupancy_grid = np.ones((20,20))
occupancy_grid *= 0.5

adjacent = WFD.get_adjacent((10,10), occupancy_grid)
assert adjacent == [(9,9),(9,10),(9,11),(10,9), (10,11),(11,9),(11,10),(11,11)]
print('Test 3 Passed')



for x in range(np.shape(occupancy_grid)[0]):
    for y in range(np.shape(occupancy_grid)[1]):
        assert not WFD.is_frontier((x,y), occupancy_grid)

print('Test 4 Passed')



occupancy_grid[10,10] = 0
assert not WFD.is_frontier((10,10), occupancy_grid)
for adj_node in adjacent:
    assert WFD.is_frontier(adj_node, occupancy_grid)
print('Test 5 Passed')


# Make adjacent open space and test frontier finding
for adj_node in adjacent:
    occupancy_grid[adj_node[0], adj_node[1]] = 0

occupancy_grid[9,8] = 0
occupancy_grid[8,8] = 0

assert WFD.get_next_obs_point(occupancy_grid, (9,9)) == (10,9)
print('Test 6 Passed')


occupancy_grid[10, 12] = 0
occupancy_grid[10, 13] = 0
occupancy_grid[10, 14] = 0

occupancy_grid[12, 10] = 0
occupancy_grid[13, 10] = 0
occupancy_grid[14, 10] = 0

assert WFD.get_next_obs_point(occupancy_grid, (10,10)) == (11, 10)
print('Test 7 Passed')


# Make some have obstacles
for adj_node in adjacent:
    occupancy_grid[adj_node[0], adj_node[1]] = 1

occupancy_grid[10,11] = 0
occupancy_grid[11,10] = 0

assert WFD.get_next_obs_point(occupancy_grid, (10,10)) == (12, 12)
print('Test 8 Passed')

occupancy_grid = np.array([[0.1, 0.3, 0.7, 0.9], [0.1, 0.3, 0.7, 0.9]])
rounded = WFD.round_occ_grid(occupancy_grid)

correct = np.array([[0. , 0.5, 0.5, 1. ], [0. , 0.5, 0.5, 1. ]])
for x in range(np.shape(rounded)[0]):
    for y in range(np.shape(rounded)[1]):
        assert correct[x,y] == rounded[x,y]

print("Test 9 Passed")