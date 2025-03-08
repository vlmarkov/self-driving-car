#include <lane-detection/lane_detection_module.h>

#include <gtest/gtest.h>

TEST(GetSetTest, GetGrayScaleMin) {
    LaneDetectionModule ldm({});
    int lowTheshold = ldm.getGrayScaleMin();

    ASSERT_EQ(lowTheshold, 200);
}

TEST(GetSetTest, GetGrayScaleMax) {
    LaneDetectionModule ldm({});
    int highTheshold = ldm.getGrayScaleMax();

    ASSERT_EQ(highTheshold, 255);
}

TEST(GetSetTest, GetYellowMin) {
    LaneDetectionModule ldm({});
    cv::Scalar lowTheshold = ldm.getYellowMin();

    ASSERT_EQ(lowTheshold, cv::Scalar(20, 100, 100));
}

TEST(GetSetTest, GetYellowMax) {
    LaneDetectionModule ldm({});
    cv::Scalar highTheshold = ldm.getYellowMax();

    ASSERT_EQ(highTheshold, cv::Scalar(30, 255, 255));
}

TEST(GetSetTest, SetGrayScaleMin) {
    LaneDetectionModule ldm({});
    ldm.setGrayScaleMin(150);
    int lowTheshold = ldm.getGrayScaleMin();

    ASSERT_EQ(lowTheshold, 150);
}

TEST(GetSetTest, SetGrayScaleMax) {
    LaneDetectionModule ldm({});
    ldm.setGrayScaleMax(210);
    int highTheshold = ldm.getGrayScaleMax();

    ASSERT_EQ(highTheshold, 210);
}

TEST(GetSetTest, SetYellowMin) {
    LaneDetectionModule ldm({});
    ldm.setYellowMin(cv::Scalar(50, 50, 50));
    cv::Scalar lowTheshold = ldm.getYellowMin();

    ASSERT_EQ(lowTheshold, cv::Scalar(50, 50, 50));
}

TEST(GetSetTest, SetYellowMax) {
    LaneDetectionModule ldm({});
    ldm.setYellowMax(cv::Scalar(150, 150, 150));
    cv::Scalar highTheshold = ldm.getYellowMax();

   ASSERT_EQ(highTheshold, cv::Scalar(150, 150, 150));
}

TEST(FunctionalTest, TestImageStraight) {
    LaneDetectionModule ldm({});
    auto frame = cv::imread("../../ros2-modules/lane-detection/tests/images/straight.jpg", cv::IMREAD_UNCHANGED);
    auto status = ldm.detect_lane(std::move(frame));

    ASSERT_EQ(status.direction, "Head straight");
}

TEST(FunctionalTest, TestImageLeft) {
    LaneDetectionModule ldm({});
    auto frame = cv::imread("../../ros2-modules/lane-detection/tests/images/left.jpg", cv::IMREAD_UNCHANGED);
    auto status = ldm.detect_lane(std::move(frame));

    ASSERT_EQ(status.direction, "Turn left");
}

TEST(FunctionalTest, TestImageRight) {
    LaneDetectionModule ldm({});
    auto frame = cv::imread("../../ros2-modules/lane-detection/tests/images/right.jpg", cv::IMREAD_UNCHANGED);
    auto status = ldm.detect_lane(std::move(frame));

    ASSERT_EQ(status.direction, "Turn right");
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
