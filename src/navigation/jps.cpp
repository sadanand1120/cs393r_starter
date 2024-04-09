#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_set>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "jps.hpp"
#include "tools.hpp"

using namespace std;

typedef pair<double, Location> PQElement;
typedef priority_queue<PQElement, vector<PQElement>, greater<PQElement>> PQLoc;

typedef double(heuristic_fn)(const Location&, const Location&);

Location jump(const Grid& grid, const Location initial, const Location dir, const Location goal) {
  auto new_loc{initial + dir};
  if (!grid.valid_move(initial, dir)) {
    return NoneLoc;
  }
  if (new_loc == goal) {
    return new_loc;
  }
  for (const auto next : grid.pruned_neighbours(new_loc, initial)) {
    if (grid.forced(next, new_loc, dir)) {
      return new_loc;
    }
  }
  if (dir.x != 0 && dir.y != 0) {
    for (const auto& new_dir : {Location{dir.x, 0}, Location{0, dir.y}}) {
      auto jump_point{jump(grid, new_loc, new_dir, goal)};
      if (jump_point != NoneLoc) {
        return new_loc;
      }
    }
  }
  return jump(grid, new_loc, dir, goal);
}

vector<Location> successors(const Grid& grid, const Location& current, const Location& parent, const Location& goal) {
  vector<Location> successors;
  auto neighbours{grid.pruned_neighbours(current, parent)};
  for (const auto& n : neighbours) {
    auto jump_point{jump(grid, current, (n - current).direction(), goal)};
    if (jump_point != NoneLoc) {
      successors.push_back(jump_point);
    }
  }
  return successors;
}

unordered_map<Location, Location> jps(const Grid& grid, const Location& start, const Location& goal,
                                      heuristic_fn heuristic) {
  PQLoc open_set;
  unordered_map<Location, Location> came_from{};
  unordered_map<Location, double> cost_so_far{};

  open_set.emplace(0, start);
  came_from[start] = start;
  cost_so_far[start] = 0;
  Location parent{NoneLoc};

  while (!open_set.empty()) {
    const auto current{open_set.top().second};
    if (current == goal) {
      break;
    }

    open_set.pop();
    if (current != start) {
      parent = came_from[current];
    }

    for (const auto& next : successors(grid, current, parent, goal)) {
      const auto new_cost = cost_so_far[current] + heuristic(current, next);
      auto existing_cost = std::numeric_limits<double>::max();
      if (cost_so_far.count(next)) {
        existing_cost = cost_so_far.at(next);
      }
      if (cost_so_far.find(next) == cost_so_far.end() || new_cost < existing_cost) {
        cost_so_far[next] = new_cost;
        came_from[next] = current;
        open_set.emplace(new_cost + heuristic(next, goal), next);
      }
    }
  }
  return came_from;
}

// Function to read CSV into an unordered_set<Location>
std::unordered_set<Location> read_walls_from_csv(const std::string& filepath) {
  std::unordered_set<Location> walls;
  std::ifstream file(filepath);
  std::string line;

  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string token;
    std::getline(iss, token, ',');
    int x = std::stoi(token);
    std::getline(iss, token, ',');
    int y = std::stoi(token);

    walls.insert({x, y});
  }

  return walls;
}

void write_path_to_csv(const std::string& filepath, const std::vector<Location>& path, const Location& start) {
  std::ofstream file(filepath);
  file << start.x << "," << start.y << "\n";
  for (const auto& loc : path) {
    file << loc.x << "," << loc.y << "\n";
  }
}

Location transformCell(float x, float y) {
  int new_x = (x + 50.0) / 0.5;
  int new_y = (50.0 - y) / 0.5;
  return Location{new_x, new_y};
}

FloatLocation untransformCell(int x, int y) {
  float real_x = x * 0.5 - 50.0;
  float real_y = 50.0 - y * 0.5;
  return FloatLocation{real_x, real_y, Eigen::Vector2f(real_x, real_y)};
}

// int main() {
//   // unordered_set<Location> walls{{0, 0}, {9, 9}};
//   auto walls =
//       read_walls_from_csv("/home/dynamo/AMRL_Research/repos/cs393r_starter/src/navigation/scripts/GDC1_walls.csv");
//   Grid map{200, 200, walls};

//   Location start{124, 70};
//   Location goal{141, 82};

//   auto came_from = jps(map, start, goal, Tool::euclidean);
//   auto path = Tool::reconstruct_path(start, goal, came_from);
//   write_path_to_csv("/home/dynamo/AMRL_Research/repos/cs393r_starter/src/navigation/scripts/GDC1_path.csv", path,
//                     start);
//   printf("Path: \n");
//   std::cout << start << " ";
//   for (const Location& loc : path) {
//     std::cout << loc << " ";
//   }
//   printf("\n --------------------------------------- \n");
//   // Tool::draw_grid(map, {}, {}, path, came_from, start, goal);
// }
