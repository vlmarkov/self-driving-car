#include <motion-calibration/digital_values.h>

#include <gtest/gtest.h>

TEST(DigitalValuesTests, ExpectStop) {
    auto digital_values = convert_to_digital_values(0.0, 0.0);
    
    EXPECT_EQ(digital_values.size(), 1);
    EXPECT_EQ(digital_values[0].forward, false);
    EXPECT_EQ(digital_values[0].backward, false);
    EXPECT_EQ(digital_values[0].left_turn, false);
    EXPECT_EQ(digital_values[0].right_turn, false);
    EXPECT_EQ(digital_values[0].timeout_ms, 1000);
    EXPECT_EQ(digital_values[0].command, "stop");
}

TEST(DigitalValuesTests, ExpectForward) {
    auto digital_values = convert_to_digital_values(10.0, 0.0);
    
    EXPECT_EQ(digital_values.size(), 1);
    EXPECT_EQ(digital_values[0].forward, true);
    EXPECT_EQ(digital_values[0].backward, false);
    EXPECT_EQ(digital_values[0].left_turn, false);
    EXPECT_EQ(digital_values[0].right_turn, false);
    EXPECT_EQ(digital_values[0].timeout_ms, 1000);
    EXPECT_EQ(digital_values[0].command, "forward");
}

TEST(DigitalValuesTests, ExpectBackward) {
    auto digital_values = convert_to_digital_values(-10.0, 0.0);
    
    EXPECT_EQ(digital_values.size(), 1);
    EXPECT_EQ(digital_values[0].forward, false);
    EXPECT_EQ(digital_values[0].backward, true);
    EXPECT_EQ(digital_values[0].left_turn, false);
    EXPECT_EQ(digital_values[0].right_turn, false);
    EXPECT_EQ(digital_values[0].timeout_ms, 1000);
    EXPECT_EQ(digital_values[0].command, "backward");
}

TEST(DigitalValuesTests, ExpectLeft) {
    auto digital_values = convert_to_digital_values(0.0, -10.0);
    
    EXPECT_EQ(digital_values.size(), 2);
    EXPECT_EQ(digital_values[0].forward, false);
    EXPECT_EQ(digital_values[0].backward, false);
    EXPECT_EQ(digital_values[0].left_turn, true);
    EXPECT_EQ(digital_values[0].right_turn, false);
    EXPECT_EQ(digital_values[0].timeout_ms, 500);
    EXPECT_EQ(digital_values[0].command, "left");

    EXPECT_EQ(digital_values[1].forward, false);
    EXPECT_EQ(digital_values[1].backward, false);
    EXPECT_EQ(digital_values[1].left_turn, false);
    EXPECT_EQ(digital_values[1].right_turn, false);
    EXPECT_EQ(digital_values[1].timeout_ms, 500);
    EXPECT_EQ(digital_values[1].command, "stop");
}

TEST(DigitalValuesTests, ExpectForwardLeft) {
    auto digital_values = convert_to_digital_values(10.0, -10.0);

    EXPECT_EQ(digital_values.size(), 2);
    EXPECT_EQ(digital_values[0].forward, false);
    EXPECT_EQ(digital_values[0].backward, false);
    EXPECT_EQ(digital_values[0].left_turn, true);
    EXPECT_EQ(digital_values[0].right_turn, false);
    EXPECT_EQ(digital_values[0].timeout_ms, 500);
    EXPECT_EQ(digital_values[0].command, "left");

    EXPECT_EQ(digital_values[1].forward, true);
    EXPECT_EQ(digital_values[1].backward, false);
    EXPECT_EQ(digital_values[1].left_turn, false);
    EXPECT_EQ(digital_values[1].right_turn, false);
    EXPECT_EQ(digital_values[1].timeout_ms, 500);
    EXPECT_EQ(digital_values[1].command, "forward");
}

TEST(DigitalValuesTests, ExpectBackwardLeft) {
    auto digital_values = convert_to_digital_values(-10.0, -10.0);

    EXPECT_EQ(digital_values.size(), 2);
    EXPECT_EQ(digital_values[0].forward, false);
    EXPECT_EQ(digital_values[0].backward, false);
    EXPECT_EQ(digital_values[0].left_turn, true);
    EXPECT_EQ(digital_values[0].right_turn, false);
    EXPECT_EQ(digital_values[0].timeout_ms, 500);
    EXPECT_EQ(digital_values[0].command, "left");

    EXPECT_EQ(digital_values[1].forward, false);
    EXPECT_EQ(digital_values[1].backward, true);
    EXPECT_EQ(digital_values[1].left_turn, false);
    EXPECT_EQ(digital_values[1].right_turn, false);
    EXPECT_EQ(digital_values[1].timeout_ms, 500);
    EXPECT_EQ(digital_values[1].command, "backward");
}

TEST(DigitalValuesTests, ExpectRight) {
    auto digital_values = convert_to_digital_values(0.0, 10.0);
    
    EXPECT_EQ(digital_values.size(), 2);
    EXPECT_EQ(digital_values[0].forward, false);
    EXPECT_EQ(digital_values[0].backward, false);
    EXPECT_EQ(digital_values[0].left_turn, false);
    EXPECT_EQ(digital_values[0].right_turn, true);
    EXPECT_EQ(digital_values[0].timeout_ms, 500);
    EXPECT_EQ(digital_values[0].command, "right");

    EXPECT_EQ(digital_values[1].forward, false);
    EXPECT_EQ(digital_values[1].backward, false);
    EXPECT_EQ(digital_values[1].left_turn, false);
    EXPECT_EQ(digital_values[1].right_turn, false);
    EXPECT_EQ(digital_values[1].timeout_ms, 500);
    EXPECT_EQ(digital_values[1].command, "stop");
}

TEST(DigitalValuesTests, ExpectForwardRight) {
    auto digital_values = convert_to_digital_values(10.0, 10.0);

    EXPECT_EQ(digital_values.size(), 2);
    EXPECT_EQ(digital_values[0].forward, false);
    EXPECT_EQ(digital_values[0].backward, false);
    EXPECT_EQ(digital_values[0].left_turn, false);
    EXPECT_EQ(digital_values[0].right_turn, true);
    EXPECT_EQ(digital_values[0].timeout_ms, 500);
    EXPECT_EQ(digital_values[0].command, "right");

    EXPECT_EQ(digital_values[1].forward, true);
    EXPECT_EQ(digital_values[1].backward, false);
    EXPECT_EQ(digital_values[1].left_turn, false);
    EXPECT_EQ(digital_values[1].right_turn, false);
    EXPECT_EQ(digital_values[1].timeout_ms, 500);
    EXPECT_EQ(digital_values[1].command, "forward");
}

TEST(DigitalValuesTests, ExpectBackwardRight) {
    auto digital_values = convert_to_digital_values(-10.0, 10.0);

    EXPECT_EQ(digital_values.size(), 2);
    EXPECT_EQ(digital_values[0].forward, false);
    EXPECT_EQ(digital_values[0].backward, false);
    EXPECT_EQ(digital_values[0].left_turn, false);
    EXPECT_EQ(digital_values[0].right_turn, true);
    EXPECT_EQ(digital_values[0].timeout_ms, 500);
    EXPECT_EQ(digital_values[0].command, "right");

    EXPECT_EQ(digital_values[1].forward, false);
    EXPECT_EQ(digital_values[1].backward, true);
    EXPECT_EQ(digital_values[1].left_turn, false);
    EXPECT_EQ(digital_values[1].right_turn, false);
    EXPECT_EQ(digital_values[1].timeout_ms, 500);
    EXPECT_EQ(digital_values[1].command, "backward");
}

int main(int argc, char ** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
