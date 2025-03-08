#include <line-detection/lane_detection_module.h>

#include <gtest/gtest.h>

TEST(GetSetTest, GetGrayScaleMin) {
    LaneDetectionModule laneModule;
    int lowTheshold = laneModule.getGrayScaleMin();

    ASSERT_EQ(lowTheshold, 200);
}

TEST(GetSetTest, GetGrayScaleMax) {
    LaneDetectionModule laneModule;
    int highTheshold = laneModule.getGrayScaleMax();

    ASSERT_EQ(highTheshold, 255);
}

TEST(GetSetTest, GetYellowMin) {
    LaneDetectionModule laneModule;
    cv::Scalar lowTheshold = laneModule.getYellowMin();

    ASSERT_EQ(lowTheshold, cv::Scalar(20, 100, 100));
}

TEST(GetSetTest, GetYellowMax) {
    LaneDetectionModule laneModule;
    cv::Scalar highTheshold = laneModule.getYellowMax();

    ASSERT_EQ(highTheshold, cv::Scalar(30, 255, 255));
}

TEST(GetSetTest, SetGrayScaleMin) {
    LaneDetectionModule laneModule;
    laneModule.setGrayScaleMin(150);
    int lowTheshold = laneModule.getGrayScaleMin();

    ASSERT_EQ(lowTheshold, 150);
}

TEST(GetSetTest, SetGrayScaleMax) {
    LaneDetectionModule laneModule;
    laneModule.setGrayScaleMax(210);
    int highTheshold = laneModule.getGrayScaleMax();

    ASSERT_EQ(highTheshold, 210);
}

TEST(GetSetTest, SetYellowMin) {
    LaneDetectionModule laneModule;
    laneModule.setYellowMin(cv::Scalar(50, 50, 50));
    cv::Scalar lowTheshold = laneModule.getYellowMin();

    ASSERT_EQ(lowTheshold, cv::Scalar(50, 50, 50));
}

TEST(GetSetTest, SetYellowMax) {
    LaneDetectionModule laneModule;
    laneModule.setYellowMax(cv::Scalar(150, 150, 150));
    cv::Scalar highTheshold = laneModule.getYellowMax();

   ASSERT_EQ(highTheshold, cv::Scalar(150, 150, 150));
}

TEST(FunctionalTest, TestImageStraight) {
    LaneDetectionModule laneModule;
    auto frame = cv::imread("../../ros2-modules/line-detection/tests/images/straight.jpg", cv::IMREAD_UNCHANGED);
    auto status = laneModule.detectLane(std::move(frame));

    ASSERT_EQ(status.direction, "Head straight");
}

TEST(FunctionalTest, TestImageLeft) {
    LaneDetectionModule laneModule;
    auto frame = cv::imread("../../ros2-modules/line-detection/tests/images/left.jpg", cv::IMREAD_UNCHANGED);
    auto status = laneModule.detectLane(std::move(frame));

    ASSERT_EQ(status.direction, "Turn left");
}

TEST(FunctionalTest, TestImageRight) {
    LaneDetectionModule laneModule;
    auto frame = cv::imread("../../ros2-modules/line-detection/tests/images/right.jpg", cv::IMREAD_UNCHANGED);
    auto status = laneModule.detectLane(std::move(frame));

    ASSERT_EQ(status.direction, "Turn right");
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
