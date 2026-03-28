#include "../src/a_star.hpp"

#include <gtest/gtest.h>

namespace {

std::vector<std::vector<int>> SMALL_GRAPH {
    {1, 0, 0, 0 ,0 ,0},
    {1, 1, 1, 0 ,1 ,1},
    {1, 0, 1, 0 ,1 ,1},
    {1, 0, 1, 1 ,1 ,0},
};

std::vector<std::vector<int>> BIG_GRAPH {
    {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
    {1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1},
    {1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0},
    {1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1},
    {1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1},
    {1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1},
};


class AstarTest : public ::testing::Test {
public:
    std::shared_ptr<IPathFinder> path_finder = IPathFinder::create(IPathFinder::Type::ASTAR);
    std::shared_ptr<Map> map = std::make_shared<Map>(Map{.graph = SMALL_GRAPH});
};

class AstarBigGraphTest : public ::testing::Test {
public:
    std::shared_ptr<IPathFinder> path_finder = IPathFinder::create(IPathFinder::Type::ASTAR);
    std::shared_ptr<Map> map = std::make_shared<Map>(Map{.graph = BIG_GRAPH});
};

} // namespace

TEST_F(AstarTest, From00to00ExpectPath0) {
    const Waypoint start{.x = 0, .y = 0};
    const Waypoint goal{.x = 0, .y = 0};
    const std::vector<Waypoint> expected_path{
        {.x = 0, .y = 0}
    };

    const auto path = path_finder->find_path(map, start, goal)->reconstruct(goal);
    EXPECT_EQ(path, expected_path);
}

TEST_F(AstarTest, From00to01ExpectPath1) {
    const Waypoint start{.x = 0, .y = 0};
    const Waypoint goal{.x = 0, .y = 1};
    const std::vector<Waypoint> expected_path{
        {.x = 0, .y = 0},
        {.x = 0, .y = 1}
    };

    const auto path = path_finder->find_path(map, start, goal)->reconstruct(goal);
    EXPECT_EQ(path, expected_path);
}

TEST_F(AstarTest, From00to03ExpectPath3) {
    const Waypoint start{.x = 0, .y = 0};
    const Waypoint goal{.x = 0, .y = 3};
    const std::vector<Waypoint> expected_path{
        {.x = 0, .y = 0},
        {.x = 0, .y = 1},
        {.x = 0, .y = 2},
        {.x = 0, .y = 3}
    };

    const auto path = path_finder->find_path(map, start, goal)->reconstruct(goal);
    EXPECT_EQ(path, expected_path);
}

TEST_F(AstarTest, From00to51ExpectPath11) {
    const Waypoint start{.x = 0, .y = 0};
    const Waypoint goal{.x = 5, .y = 1};
    const std::vector<Waypoint> expected_path{
        {.x = 0, .y = 0},
        {.x = 0, .y = 1},
        {.x = 1, .y = 1},
        {.x = 2, .y = 1},
        {.x = 2, .y = 2},
        {.x = 2, .y = 3},
        {.x = 3, .y = 3},
        {.x = 4, .y = 3},
        {.x = 4, .y = 2},
        {.x = 4, .y = 1},
        {.x = 5, .y = 1}
    };

    const auto path = path_finder->find_path(map, start, goal)->reconstruct(goal);
    EXPECT_EQ(path, expected_path);
}

TEST_F(AstarTest, From00to53ExpectPathEmpty) {
    const Waypoint start{.x = 0, .y = 0};
    const Waypoint goal{.x = 5, .y = 3};
    const std::vector<Waypoint> expected_path{};

    const auto path = path_finder->find_path(map, start, goal)->reconstruct(goal);
    EXPECT_EQ(path, expected_path);
}

TEST_F(AstarTest, From33to00ExpectPath7) {
    const Waypoint start{.x = 3, .y = 3};
    const Waypoint goal{.x = 0, .y = 0};
    const std::vector<Waypoint> expected_path{
       {.x = 3, .y = 3},
       {.x = 2, .y = 3},
       {.x = 2, .y = 2},
       {.x = 2, .y = 1},
       {.x = 1, .y = 1},
       {.x = 0, .y = 1},
       {.x = 0, .y = 0}
    };

    const auto path = path_finder->find_path(map, start, goal)->reconstruct(goal);
    EXPECT_EQ(path, expected_path);
}

TEST_F(AstarTest, From33to51ExpectPath5) {
    const Waypoint start{.x = 3, .y = 3};
    const Waypoint goal{.x = 5, .y = 1};
    const std::vector<Waypoint> expected_path{
        {.x = 3, .y = 3},
        {.x = 4, .y = 3},
        {.x = 4, .y = 2},
        {.x = 4, .y = 1},
        {.x = 5, .y = 1}
    };

    const auto path = path_finder->find_path(map, start, goal)->reconstruct(goal);
    EXPECT_EQ(path, expected_path);
}

TEST_F(AstarBigGraphTest, From53to117ExpectVisualizeNoThrow) {
    const Waypoint start{.x = 5, .y = 3};
    const Waypoint goal{.x = 11, .y = 7};

    EXPECT_NO_THROW(path_finder->find_path(map, start, goal)->visualize());
}
