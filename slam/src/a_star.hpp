#pragma once

#include "i_path_finder.hpp"

class Astar : public IPathFinder
{
public:
    Astar() = default;
    ~Astar() = default;

    std::shared_ptr<Path> find_path(const std::shared_ptr<Map> map,
                                    const Waypoint& start,
                                    const Waypoint& goal) final;
};
