#include <pins.h>
#include <motor_commands.h>

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
}
  
void loop() {
    int is_stop = digitalRead(FROM_RAPSBERRY_PIN_0);
    int is_forward = digitalRead(FROM_RAPSBERRY_PIN_1);
    int is_left_turn = digitalRead(FROM_RAPSBERRY_PIN_0);

    MotorCommands mc = get_motor_commands(is_stop, is_forward, is_left_turn);

    digitalWrite(ENGINE_LEFT_HIGH_PIN, mc.engine_left_high);
    digitalWrite(ENGINE_LEFT_LOW_PIN, mc.engine_left_low);
    digitalWrite(ENGINE_RIGHT_HIGH_PIN, mc.engine_right_high);
    digitalWrite(ENGINE_RIGHT_LOW_PIN, mc.engine_right_low);
    
    analogWrite(ENGINE_LEFT_PIN, mc.engine_left_voltage);
    analogWrite(ENGINE_RIGHT_PIN, mc.engine_right_voltage);
}
