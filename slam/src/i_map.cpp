#include "i_map.hpp"

namespace {

constexpr auto BLOCKED_CELL = 0;

} // namespace

bool Map::is_reachable(const Waypoint& wp) const {
    if (graph.empty())
        return false;

    if (wp.y < 0) {
        return false;
    }
    if (wp.x < 0) {
        return false;
    }

    if (wp.y >= static_cast<int>(graph.size())) {
        return false;
    }
    if (wp.x >= static_cast<int>(graph[wp.y].size())) {
        return false;
    }

    if (graph[wp.y][wp.x] == BLOCKED_CELL) {
        return false;
    }
    
    return true;
}
