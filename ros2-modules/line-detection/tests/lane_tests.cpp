#include <line-detection/lane.h>

#include <gtest/gtest.h>

TEST(LaneConstructorTest, EmptyConstructor) {
    Lane laneTest;
    int polyOrder = laneTest.getPolyOrder();

    ASSERT_EQ(polyOrder, 1);
}

TEST(LaneConstructorTest, CustomConstructor) {
    Lane laneCustom(2, "blue", 10);
    int polyOrder = laneCustom.getPolyOrder();

    ASSERT_EQ(polyOrder, 2);
}

TEST(GetSetTest, polyOrderGetTest) {
    Lane laneTest2;
    laneTest2.setPolyOrder(3);
    int polyOrder = laneTest2.getPolyOrder();

    ASSERT_EQ(polyOrder, 3);
}

TEST(GetSetTest, startCoodGetTest) {
    Lane laneTest2;
    laneTest2.setStartCoordinate(cv::Point(30, 50));
    cv::Point point = laneTest2.getStartCoordinate();
    
    ASSERT_EQ(point.x, 30);
    ASSERT_EQ(point.y, 50);
}

TEST(GetSetTest, statusGetTest) {
    Lane laneTest2;
    laneTest2.setStatus(true);
    bool status = laneTest2.getStatus();

    ASSERT_EQ(status, true);
}

TEST(GetSetTest, polyCoeffGetTest) {
    Lane laneTest2;
    cv::Mat laneCoeff = (cv::Mat_<float>(3, 1) << 1.2, 0.0, 5.6);
    laneTest2.setPolyCoeff(laneCoeff);
    std::vector<float> coeffOut = laneTest2.getPolyCoeff();
    
    ASSERT_NEAR(1.2, coeffOut[0], 0.1);
    ASSERT_NEAR(0.0, coeffOut[1], 0.1);
    ASSERT_NEAR(5.6, coeffOut[2], 0.1);
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
