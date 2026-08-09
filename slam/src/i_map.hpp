#pragma once

#include <vector>

struct Waypoint
{
    int x{0}; // X coordinate
    int y{0}; // Y coordinate

    bool operator==(const Waypoint& rhs) const {
        return x == rhs.x && y == rhs.y;
    }
};

class Map
{
public:
    bool is_reachable(const Waypoint& wp) const;

    std::vector<std::vector<int>> graph;
};
