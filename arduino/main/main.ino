#include "pins.h"
#include "motor_commands.h"

void setup() {
    pinMode(ENGINE_LEFT_PWM_PIN,     OUTPUT);
    pinMode(ENGINE_LEFT_FORWARD_PIN, OUTPUT);
    pinMode(ENGINE_LEFT_REVERSE_PIN, OUTPUT);
    
    pinMode(ENGINE_RIGHT_PWM_PIN,     OUTPUT);
    pinMode(ENGINE_RIGHT_FORWARD_PIN, OUTPUT);
    pinMode(ENGINE_RIGHT_REVERSE_PIN, OUTPUT);

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

    digitalWrite(ENGINE_LEFT_FORWARD_PIN,  mc.engine_left_forward);
    digitalWrite(ENGINE_LEFT_REVERSE_PIN,  mc.engine_left_reverse);
    digitalWrite(ENGINE_RIGHT_FORWARD_PIN, mc.engine_right_forward);
    digitalWrite(ENGINE_RIGHT_REVERSE_PIN, mc.engine_right_reverse);

    analogWrite(ENGINE_LEFT_PWM_PIN,  mc.engine_left_pwm);
    analogWrite(ENGINE_RIGHT_PWM_PIN, mc.engine_right_pwm);
}
