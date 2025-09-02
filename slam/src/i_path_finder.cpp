#include "i_path_finder.hpp"
#include "a_star.hpp"

#include <algorithm>
#include <stdexcept>
#include <iostream>

std::vector<Waypoint> Path::reconstruct(const Waypoint& goal) const {
    if (waypoints.empty())
        return {};

    std::vector<Waypoint> result;
    Waypoint current;
    Waypoint parent;

    auto rbegin = waypoints.rbegin();
    const auto rend = waypoints.rend();

    while (rbegin != rend) {
        const auto it = rbegin;
        rbegin++;

        if (it->current == goal) {
            current = it->current;
            parent = it->parent;
            result.push_back(current);
            break;
        }
    }

    while (rbegin != rend) {
        if (rbegin->current == parent) {
            parent = rbegin->parent;
            result.push_back(rbegin->current);
        }

        rbegin++;
    }

    std::reverse(result.begin(), result.end());
    return result;
}

void Path::visualize() const {
    if (waypoints.empty())
        return;

    int max_x = 0;
    int max_y = 0;

    for (const auto& wp : waypoints) {
        max_x = std::max(max_x, wp.current.x);
        max_y = std::max(max_y, wp.current.y);
    }

    std::vector<std::vector<std::string>> graph(max_y + 1, std::vector<std::string>(max_x + 1, "[ ]"));

    auto print_fn = [](const auto& graph) {
        for (const auto& g : graph) {
            for (const auto& point : g) {
                std::cout << point << "";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    };

    for (const auto& wp : waypoints) {
        graph[wp.current.y][wp.current.x] = "[x]";
        print_fn(graph);
    }
}

std::shared_ptr<IPathFinder> IPathFinder::create(Type type) {
    if (type == IPathFinder::Type::ASTAR) {
        return std::make_shared<Astar>();
    }

    throw std::runtime_error("unsuported path find algorithm");
}
