#pragma once

#include "i_map.hpp"

#include <deque>
#include <memory>

struct Path {
    struct WaypointInfo {
        Waypoint parent;
        Waypoint current;
    };

    std::deque<WaypointInfo> waypoints;

    std::vector<Waypoint> reconstruct(const Waypoint& goal) const;
    void visualize() const;
};

class IPathFinder {
public:
    enum class Type{ASTAR};

    virtual ~IPathFinder() = default;

    virtual std::shared_ptr<Path> find_path(const std::shared_ptr<Map> map,
                                            const Waypoint& start,
                                            const Waypoint& goal) = 0;

    static std::shared_ptr<IPathFinder> create(Type type);

protected:
    IPathFinder() = default;
};
