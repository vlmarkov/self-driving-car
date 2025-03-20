#include "motor_commands.h"

MotorCommands get_motor_commands(int binary_command) {
    MotorCommands mc;
    mc.engine_left_high = 1;
    mc.engine_left_low = 0;
    mc.engine_right_high = 1;
    mc.engine_right_low = 0;
        
    mc.engine_left_voltage = 0;
    mc.engine_right_voltage = 0;

    if (binary_command == FORWARD) {
        mc.engine_left_high = 0;
        mc.engine_left_low = 1;
        mc.engine_right_high = 0;
        mc.engine_right_low = 1;

        mc.engine_left_voltage = 250;
        mc.engine_right_voltage = 250;
    } else if (binary_command == BACKWARD) {
        mc.engine_left_high = 1;
        mc.engine_left_low = 0;
        mc.engine_right_high = 1;
        mc.engine_right_low = 0;
    
        mc.engine_left_voltage = 250;
        mc.engine_right_voltage = 250;
    } else if (binary_command == LEFT_TURN) {
        mc.engine_left_high = 0;
        mc.engine_left_low = 1;
        mc.engine_right_high = 0;
        mc.engine_right_low = 1;

        mc.engine_left_voltage = 0;
        mc.engine_right_voltage = 230;
    } else if (binary_command == RIGHT_TURN) {
        mc.engine_left_high = 0;
        mc.engine_left_low = 1;
        mc.engine_right_high = 1;
        mc.engine_right_low = 0;

        mc.engine_left_voltage = 230;
        mc.engine_right_voltage = 0;
    }

    return mc;
}
