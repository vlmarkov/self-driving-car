#include <motor_commands.h>

MotorCommands get_motor_values(int is_stop, int is_forward, int is_left_turn) {
    MotorCommands mc;

  if (is_stop == 1) {
        mc.engine_left_high = 1;
        mc.engine_left_low = 0;
        mc.engine_right_high = 1;
        mc.engine_right_low = 0;
        
        mc.engine_left_voltage = 0;
        mc.engine_right_voltage = 0;
    } else if (is_forward == 1) {
        mc.engine_left_high = 0;
        mc.engine_left_low = 1;
        mc.engine_right_high = 0;
        mc.engine_right_low = 1;

        mc.engine_left_voltage = 250;
        mc.engine_right_voltage = 250;
    } else if (is_forward == 0) {
        mc.engine_left_high = 1;
        mc.engine_left_low = 0;
        mc.engine_right_high = 1;
        mc.engine_right_low = 0;
    
        mc.engine_left_voltage = 250;
        mc.engine_right_voltage = 250;
    } else if (is_left_turn == 1) {
        mc.engine_left_high = 0;
        mc.engine_left_low = 1;
        mc.engine_right_high = 0;
        mc.engine_right_low = 1;

        mc.engine_left_voltage = 0;
        mc.engine_right_voltage = 230;
    } else if (is_left_turn == 0) {
        mc.engine_left_high = 0;
        mc.engine_left_low = 1;
        mc.engine_right_high = 1;
        mc.engine_right_low = 0;

        mc.engine_left_voltage = 230;
        mc.engine_right_voltage = 0;
    }

    return mc;
}
