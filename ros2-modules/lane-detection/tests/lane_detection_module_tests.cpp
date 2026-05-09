#include <lane-detection/lane_detection_module.h>

#include <gtest/gtest.h>

TEST(FunctionalTest, TestImageStraight) {
    LaneDetectionModule ldm({});
    auto frame = cv::imread("../../ros2-modules/lane-detection/tests/images/straight.jpg", cv::IMREAD_UNCHANGED);
    auto status = ldm.detect_lane(std::move(frame));

    ASSERT_TRUE(status.has_value());
    ASSERT_EQ(status->direction, "Head straight");
}

TEST(FunctionalTest, TestImageLeft) {
    LaneDetectionModule ldm({});
    auto frame = cv::imread("../../ros2-modules/lane-detection/tests/images/left.jpg", cv::IMREAD_UNCHANGED);
    auto status = ldm.detect_lane(std::move(frame));

    ASSERT_TRUE(status.has_value());
    ASSERT_EQ(status->direction, "Turn left");
}

TEST(FunctionalTest, TestImageRight) {
    LaneDetectionModule ldm({});
    auto frame = cv::imread("../../ros2-modules/lane-detection/tests/images/right.jpg", cv::IMREAD_UNCHANGED);
    auto status = ldm.detect_lane(std::move(frame));

    ASSERT_TRUE(status.has_value());
    ASSERT_EQ(status->direction, "Turn right");
}

TEST(FunctionalTest, TestImageEmpty) {
    LaneDetectionModule ldm({});
    cv::Mat frame;
    auto status = ldm.detect_lane(std::move(frame));

    ASSERT_FALSE(status.has_value());
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
