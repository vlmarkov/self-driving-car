#include "a_star.hpp"

#include <algorithm>
#include <queue>
#include <map>
#include <set>

namespace {

constexpr auto PATH_COST = 1;

// Define possible movements (4 directions: up, down, left, right)
constexpr int NEIGHBORS = 4;
constexpr int DIRECTION_X[NEIGHBORS] = {0, 0, -1, 1};
constexpr int DIRECTION_Y[NEIGHBORS] = {-1, 1, 0, 0};

struct Point
{
    int x{0}; // X coordinate
    int y{0}; // Y coordinate
    int f{0}; // Total Cost: The sum of g and h. This is the value used to prioritize nodes in the priority queue.
    int g{0}; // Cost from Start: The cost to reach a node from the start node.
    int h{0}; // Heuristic Cost to Goal: An estimate of the cost from the current node to the goal

    bool operator<(const Point& rhs) const;
    bool operator==(const Point& rhs) const;
};

bool Point::operator<(const Point& rhs) const 
{
    return f < rhs.f;
}

bool Point::operator==(const Point& rhs) const 
{
    return x == rhs.x && y == rhs.y;
}

struct GScoreValue {
    int value{__INT32_MAX__};
};

using Y_COORD = int;
using X_COORD = int;

using GScore = std::map<std::pair<Y_COORD, X_COORD>, GScoreValue>;
using VisitedPoints = std::set<std::pair<Y_COORD, X_COORD>>;
using NotVisitedPoints = std::priority_queue<Point, std::vector<Point>, std::less<Point>>;

} // namespace

std::shared_ptr<Path> Astar::find_path(const std::shared_ptr<Map> map,
                                       const Waypoint& start,
                                       const Waypoint& goal)
{
    std::shared_ptr<Path> path = std::make_shared<Path>();

    const Point start_point{.x = start.x, .y = start.y};
    const Point end_point{.x = goal.x, .y = goal.y};

    GScore g_score;
    VisitedPoints visited;
    NotVisitedPoints not_visited;

    not_visited.push(start_point);
    g_score[{start_point.x, start_point.y}].value = 0;
    path->waypoints.push_back(Path::WaypointInfo{.parent = start, .current = start});

    while (!not_visited.empty()) {
        const auto current = not_visited.top();
        not_visited.pop();

        if (current == end_point) {
            return path;
        }

        visited.insert(std::make_pair(current.y, current.x));

        for (int i = 0; i < NEIGHBORS; ++i) {
            const auto new_x = current.x + DIRECTION_X[i];
            const auto new_y = current.y + DIRECTION_Y[i];

            if (!map->is_reachable(Waypoint{.x = new_x, .y = new_y})) {
                continue;
            }

            if (visited.contains(std::make_pair(new_y, new_x))) {
                continue;
            }

            Point neighbor{.x = new_x, .y = new_y};
            const auto new_g = current.g + PATH_COST;

            if (new_g >= g_score[{neighbor.x, neighbor.y}].value) {
                continue;
            }

            neighbor.g = new_g;
            neighbor.h = std::abs(new_x - goal.x) + std::abs(new_y - goal.y);
            neighbor.f = neighbor.g + neighbor.h;

            not_visited.push(neighbor);
            g_score[{neighbor.x, neighbor.y}].value = neighbor.g;
            path->waypoints.push_back(Path::WaypointInfo{
                .parent = Waypoint{.x = current.x, .y = current.y},
                .current = Waypoint{.x = neighbor.x, .y = neighbor.y}
            });
        }
    }

    return std::make_shared<Path>();
}
