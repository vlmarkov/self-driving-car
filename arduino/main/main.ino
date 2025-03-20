#include "pins.h"
#include "motor_commands.h"

void setup() {
    pinMode(ENGINE_LEFT_PIN, OUTPUT);
    pinMode(ENGINE_LEFT_HIGH_PIN, OUTPUT);
    pinMode(ENGINE_LEFT_LOW_PIN, OUTPUT);
    
    pinMode(ENGINE_RIGHT_PIN, OUTPUT);
    pinMode(ENGINE_RIGHT_HIGH_PIN, OUTPUT);
    pinMode(ENGINE_RIGHT_LOW_PIN, OUTPUT);

    pinMode(FROM_RAPSBERRY_PIN_0, INPUT_PULLUP);
    pinMode(FROM_RAPSBERRY_PIN_1, INPUT_PULLUP);
    pinMode(FROM_RAPSBERRY_PIN_2, INPUT_PULLUP);
    pinMode(FROM_RAPSBERRY_PIN_3, INPUT_PULLUP);
}
  
void loop() {
    int binary_command = 0;
    binary_command |= (digitalRead(FROM_RAPSBERRY_PIN_0) << FROM_RAPSBERRY_PIN_0);
    binary_command |= (digitalRead(FROM_RAPSBERRY_PIN_1) << FROM_RAPSBERRY_PIN_1);
    binary_command |= (digitalRead(FROM_RAPSBERRY_PIN_2) << FROM_RAPSBERRY_PIN_2);
    binary_command |= (digitalRead(FROM_RAPSBERRY_PIN_3) << FROM_RAPSBERRY_PIN_3);

    MotorCommands mc = get_motor_commands(binary_command);

    digitalWrite(ENGINE_LEFT_HIGH_PIN, mc.engine_left_high);
    digitalWrite(ENGINE_LEFT_LOW_PIN, mc.engine_left_low);
    digitalWrite(ENGINE_RIGHT_HIGH_PIN, mc.engine_right_high);
    digitalWrite(ENGINE_RIGHT_LOW_PIN, mc.engine_right_low);
    
    analogWrite(ENGINE_LEFT_PIN, mc.engine_left_voltage);
    analogWrite(ENGINE_RIGHT_PIN, mc.engine_right_voltage);
}
