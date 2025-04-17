#include <motion-calibration/digital_values.h>

#include <gtest/gtest.h>

TEST(DigitalValuesTests, ExpectStop) {
    auto digital_values = convert_to_digital_values(0.0, 0.0);

    EXPECT_EQ(digital_values.size(), 1);
    EXPECT_EQ(digital_values[0].forward, false);
    EXPECT_EQ(digital_values[0].backward, false);
    EXPECT_EQ(digital_values[0].left_turn, false);
    EXPECT_EQ(digital_values[0].right_turn, false);
    EXPECT_EQ(digital_values[0].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[0].command, "stop");
}

TEST(DigitalValuesTests, ExpectForward) {
    auto digital_values = convert_to_digital_values(10.0, 0.0);

    EXPECT_EQ(digital_values.size(), 1);
    EXPECT_EQ(digital_values[0].forward, true);
    EXPECT_EQ(digital_values[0].backward, false);
    EXPECT_EQ(digital_values[0].left_turn, false);
    EXPECT_EQ(digital_values[0].right_turn, false);
    EXPECT_EQ(digital_values[0].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[0].command, "forward");
}

TEST(DigitalValuesTests, ExpectBackward) {
    auto digital_values = convert_to_digital_values(-10.0, 0.0);

    EXPECT_EQ(digital_values.size(), 1);
    EXPECT_EQ(digital_values[0].forward, false);
    EXPECT_EQ(digital_values[0].backward, true);
    EXPECT_EQ(digital_values[0].left_turn, false);
    EXPECT_EQ(digital_values[0].right_turn, false);
    EXPECT_EQ(digital_values[0].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[0].command, "backward");
}

TEST(DigitalValuesTests, ExpectLeft) {
    auto digital_values = convert_to_digital_values(0.0, -CALIBRATED_ANGLE * 2.0);
    
    EXPECT_EQ(digital_values.size(), 5);
    EXPECT_EQ(digital_values[0].command, "stop");
    EXPECT_EQ(digital_values[0].timeout_ms, BETWEEN_TURN_TIMEOUT_MS);
    EXPECT_EQ(digital_values[1].command, "left");
    EXPECT_EQ(digital_values[2].command, "stop");
    EXPECT_EQ(digital_values[3].command, "left");
    EXPECT_EQ(digital_values[4].command, "stop");

    EXPECT_EQ(digital_values[1].forward, false);
    EXPECT_EQ(digital_values[1].backward, false);
    EXPECT_EQ(digital_values[1].left_turn, true);
    EXPECT_EQ(digital_values[1].right_turn, false);
    EXPECT_EQ(digital_values[1].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[1].command, "left");
}

TEST(DigitalValuesTests, ExpectForwardLeft) {
    auto digital_values = convert_to_digital_values(10.0, -CALIBRATED_ANGLE);

    EXPECT_EQ(digital_values.size(), 4);
    EXPECT_EQ(digital_values[0].command, "stop");
    EXPECT_EQ(digital_values[0].timeout_ms, BETWEEN_TURN_TIMEOUT_MS);
    EXPECT_EQ(digital_values[1].command, "left");
    EXPECT_EQ(digital_values[2].command, "stop");
    EXPECT_EQ(digital_values[3].command, "forward");

    EXPECT_EQ(digital_values[1].forward, false);
    EXPECT_EQ(digital_values[1].backward, false);
    EXPECT_EQ(digital_values[1].left_turn, true);
    EXPECT_EQ(digital_values[1].right_turn, false);
    EXPECT_EQ(digital_values[1].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[1].command, "left");

    EXPECT_EQ(digital_values[3].forward, true);
    EXPECT_EQ(digital_values[3].backward, false);
    EXPECT_EQ(digital_values[3].left_turn, false);
    EXPECT_EQ(digital_values[3].right_turn, false);
    EXPECT_EQ(digital_values[3].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[3].command, "forward");
}

TEST(DigitalValuesTests, ExpectBackwardLeft) {
    auto digital_values = convert_to_digital_values(-10.0, -CALIBRATED_ANGLE);

    EXPECT_EQ(digital_values.size(), 4);
    EXPECT_EQ(digital_values[0].command, "stop");
    EXPECT_EQ(digital_values[0].timeout_ms, BETWEEN_TURN_TIMEOUT_MS);
    EXPECT_EQ(digital_values[1].command, "left");
    EXPECT_EQ(digital_values[2].command, "stop");
    EXPECT_EQ(digital_values[3].command, "backward");

    EXPECT_EQ(digital_values[1].forward, false);
    EXPECT_EQ(digital_values[1].backward, false);
    EXPECT_EQ(digital_values[1].left_turn, true);
    EXPECT_EQ(digital_values[1].right_turn, false);
    EXPECT_EQ(digital_values[1].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[1].command, "left");

    EXPECT_EQ(digital_values[3].forward, false);
    EXPECT_EQ(digital_values[3].backward, true);
    EXPECT_EQ(digital_values[3].left_turn, false);
    EXPECT_EQ(digital_values[3].right_turn, false);
    EXPECT_EQ(digital_values[3].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[3].command, "backward");
}

TEST(DigitalValuesTests, ExpectRight) {
    auto digital_values = convert_to_digital_values(0.0, CALIBRATED_ANGLE * 2.0);

    EXPECT_EQ(digital_values.size(), 5);
    EXPECT_EQ(digital_values[0].command, "stop");
    EXPECT_EQ(digital_values[0].timeout_ms, BETWEEN_TURN_TIMEOUT_MS);
    EXPECT_EQ(digital_values[1].command, "right");
    EXPECT_EQ(digital_values[2].command, "stop");
    EXPECT_EQ(digital_values[3].command, "right");
    EXPECT_EQ(digital_values[4].command, "stop");

    EXPECT_EQ(digital_values[1].forward, false);
    EXPECT_EQ(digital_values[1].backward, false);
    EXPECT_EQ(digital_values[1].left_turn, false);
    EXPECT_EQ(digital_values[1].right_turn, true);
    EXPECT_EQ(digital_values[1].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[1].command, "right");
}

TEST(DigitalValuesTests, ExpectForwardRight) {
    auto digital_values = convert_to_digital_values(10.0, CALIBRATED_ANGLE);

    EXPECT_EQ(digital_values.size(), 4);
    EXPECT_EQ(digital_values[0].command, "stop");
    EXPECT_EQ(digital_values[0].timeout_ms, BETWEEN_TURN_TIMEOUT_MS);
    EXPECT_EQ(digital_values[1].command, "right");
    EXPECT_EQ(digital_values[2].command, "stop");
    EXPECT_EQ(digital_values[3].command, "forward");

    EXPECT_EQ(digital_values[1].forward, false);
    EXPECT_EQ(digital_values[1].backward, false);
    EXPECT_EQ(digital_values[1].left_turn, false);
    EXPECT_EQ(digital_values[1].right_turn, true);
    EXPECT_EQ(digital_values[1].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[1].command, "right");

    EXPECT_EQ(digital_values[3].forward, true);
    EXPECT_EQ(digital_values[3].backward, false);
    EXPECT_EQ(digital_values[3].left_turn, false);
    EXPECT_EQ(digital_values[3].right_turn, false);
    EXPECT_EQ(digital_values[3].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[3].command, "forward");
}

TEST(DigitalValuesTests, ExpectBackwardRight) {
    auto digital_values = convert_to_digital_values(-10.0, CALIBRATED_ANGLE);

    EXPECT_EQ(digital_values.size(), 4);
    EXPECT_EQ(digital_values[0].command, "stop");
    EXPECT_EQ(digital_values[0].timeout_ms, BETWEEN_TURN_TIMEOUT_MS);
    EXPECT_EQ(digital_values[1].command, "right");
    EXPECT_EQ(digital_values[2].command, "stop");
    EXPECT_EQ(digital_values[3].command, "backward");

    EXPECT_EQ(digital_values[1].forward, false);
    EXPECT_EQ(digital_values[1].backward, false);
    EXPECT_EQ(digital_values[1].left_turn, false);
    EXPECT_EQ(digital_values[1].right_turn, true);
    EXPECT_EQ(digital_values[1].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[1].command, "right");

    EXPECT_EQ(digital_values[3].forward, false);
    EXPECT_EQ(digital_values[3].backward, true);
    EXPECT_EQ(digital_values[3].left_turn, false);
    EXPECT_EQ(digital_values[3].right_turn, false);
    EXPECT_EQ(digital_values[3].timeout_ms, DEFAULT_TIMEOUT_MS);
    EXPECT_EQ(digital_values[3].command, "backward");
}

int main(int argc, char ** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
